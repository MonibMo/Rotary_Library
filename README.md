# Rotary_Library

A `no_std`, interrupt-safe Rust driver for **quadrature rotary encoders** with an integrated push-button switch. Built on top of [`embedded-hal 1.0`](https://docs.rs/embedded-hal/1.0.0/embedded_hal/), designed for STM32 and other ARM Cortex-M targets.

---

## Features

- `no_std` compatible — no heap allocation
- Hardware-agnostic via `embedded-hal` `InputPin` trait
- Two-stage ISR / polling split for interrupt-safe use
- Integrating debounce filter on the push-button (glitch rejection)
- Full quadrature decoding with a 4×4 transition table
- Short press (`Select`) and long press (`SelectHold`) events
- Fast-rotation events (`FastUp` / `FastDown`) via consecutive-tick accumulator
- `invert_rotation` Cargo feature to reverse direction without rewiring

---

## Architecture: Two-Stage Design

The driver splits responsibility cleanly across two calling contexts:

| Method   | Where to call                | Typical period | What it does                                      |
|----------|------------------------------|----------------|---------------------------------------------------|
| `update` | Periodic timer ISR           | 1–2 ms         | Samples pins, runs debounce, accumulates steps    |
| `poll`   | Main loop / UI task          | 10–50 ms       | Decodes accumulated state → returns `InputEvent`  |

`update` must be called **faster** than `poll` so that debouncing settles before events are consumed and no quadrature edges are missed.

---

## Struct Fields & Configuration

`RotaryEncoder` owns all runtime state. The values that control timing behaviour are passed at construction and stored inside the struct — no external constants file needed.

```rust
pub struct RotaryEncoder<CLK, DT, SW> {
    clk_pin: CLK,   // CLK / phase-A input pin
    dt_pin:  DT,    // DT  / phase-B input pin
    sw_pin:  SW,    // SW  / switch input pin (active LOW, pull-up)

    // ── Debounce ──────────────────────────────────────────────────────────
    // Integrating filter counter.
    // Increments toward ROTARY_DEBOUNCE_TICKS when the pin is LOW (pressed),
    // decrements by 2 when HIGH (released).
    // `is_pressed` only latches when the counter reaches either extreme,
    // rejecting glitches shorter than ROTARY_DEBOUNCE_TICKS ticks.
    button: Button,
    // ROTARY_DEBOUNCE_TICKS — number of consecutive identical
    // readings required to settle the button state. Recommended: 3–5.

    // ── Button press / hold state machine ────────────────────────────────
    // Advances through Idle → Counting(n) → WaitingRelease → Idle.
    // The sentinel Counting(u16::MAX) signals a short press (released
    // before the hold threshold).
    button_state: ButtonState,
    // ROTARY_LONG_PRESS_TICKS — number of `update` ticks the button must
    // be continuously held before `InputEvent::SelectHold` is emitted.
    // Example: at a 1 ms update period, set 500 for a 500 ms long-press.

    // ── Quadrature accumulator ────────────────────────────────────────────
    // Running step count since the last `poll`. +1 per CW edge, -1 per CCW.
    // When |accumulator| reaches STEPS_PER_CLICK (= 4), one logical click
    // is reported and the accumulator resets.
    encoder_accumulator: i8,
    last_quad_state: u8,

    // ── Fast-rotation tracking ────────────────────────────────────────────
    // rotate_counter counts consecutive clicks in the same direction.
    // When it exceeds ROTATE_MULTI_COUNTER, `poll` emits FastUp / FastDown
    // instead of Up / Down, allowing the UI to scroll faster.
    // The counter resets after ROTARY_RESET_TIME_MILLIS idle poll cycles
    // or on a direction change.
    rotate_counter:   u8,
    last_accumulator: RotaryAccumulatorMode,
    reset_counter:    u8,
    hold_generated:   bool,
}
```

### Configuration at a Glance

| Field / Constant            | Type  | What it controls                                                              |
|-----------------------------|-------|-------------------------------------------------------------------------------|
| `ROTARY_DEBOUNCE_TICKS`     | `u8`  | Glitch filter depth. Raise to reject more noise; lower for faster response. Recommended: **3–5**. |
| `ROTARY_LONG_PRESS_TICKS`   | `u16` | Hold threshold in `update` ticks. At 1 ms period → `500` = 500 ms long-press. |
| `ROTATE_MULTI_COUNTER`      | `u8`  | Consecutive same-direction clicks before fast-scroll activates.               |
| `ROTARY_RESET_TIME_MILLIS`  | `u8`  | Idle `poll` cycles before the fast-scroll streak resets.                      |

---

## Button State Machine

```
                 [press detected]
  Idle ──────────────────────────────────────► Counting(0)

  Counting(n) ──[held, n < LONG_PRESS_TICKS]──► Counting(n+1)
  Counting(n) ──[released < LONG_PRESS_TICKS]──► Counting(u16::MAX)  ← short-press sentinel
  Counting(n) ──[n >= LONG_PRESS_TICKS]────────► WaitingRelease

  WaitingRelease ──[poll reads, still held]────► emit SelectHold  (exactly once)
  WaitingRelease ──[released]──────────────────► Idle             (no extra event)
  Counting(u16::MAX) ──[poll reads]────────────► emit Select ───► Idle
```

**Guarantees:**
- `SelectHold` fires exactly once per long press, the moment `poll` first sees `WaitingRelease`.
- No extra event is emitted when the button is released after a long press.
- `Select` fires only for genuine short presses (released before the hold threshold).

---

## Cargo.toml

```toml
[dependencies]
Rotary_Library = { path = "../Rotary_Library" }
embedded-hal = "1.0.0"
```

The crate also depends on `general_core` (workspace-internal) for the
`InputEvent`, `InputSource`, and `RotaryAccumulatorMode` types.

---

## Feature Flags

| Feature           | Default | Effect                                                                |
|-------------------|---------|-----------------------------------------------------------------------|
| `invert_rotation` | off     | Swaps `Up`↔`Down` and `FastUp`↔`FastDown` without changing wiring    |

```toml
[features]
invert_rotation = ["Rotary_Library/invert_rotation"]
```

---

## Usage Example (bare-metal, no RTOS)

```rust
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*, timer};
use Rotary_Library::RotaryEncoder;
use general_core::{InputEvent, InputSource};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpiob = dp.GPIOB.split();
    let clk_pin = gpiob.pb10.into_pull_up_input(&mut gpiob.crh);
    let dt_pin  = gpiob.pb11.into_pull_up_input(&mut gpiob.crh);
    let sw_pin  = gpiob.pb12.into_pull_up_input(&mut gpiob.crh);

    let mut encoder = RotaryEncoder::new(clk_pin, dt_pin, sw_pin);

    // Simple 1 ms tick using SysTick delay
    let mut delay = cp.SYST.delay(&clocks);
    let mut poll_divider: u8 = 0;

    loop {
        // ── 1 ms update tick ─────────────────────────────────────────────
        encoder.update();
        delay.delay_ms(1u16);

        // ── Poll every 10 ms ─────────────────────────────────────────────
        poll_divider = poll_divider.wrapping_add(1);
        if poll_divider >= 10 {
            poll_divider = 0;

            match encoder.poll() {
                InputEvent::Up         => { /* scroll up        */ }
                InputEvent::Down       => { /* scroll down      */ }
                InputEvent::FastUp     => { /* fast scroll up   */ }
                InputEvent::FastDown   => { /* fast scroll down */ }
                InputEvent::Select     => { /* short press      */ }
                InputEvent::SelectHold => { /* long press       */ }
                InputEvent::None       => {}
            }
        }
    }
}
```

---

## Internals

### Debounce Filter

A per-pin **integrating counter** advances toward `ROTARY_DEBOUNCE_TICKS` while the pin is LOW and retreats by 2 per tick while HIGH. `is_pressed` latches only when the counter saturates (pressed) or reaches zero (released). This means a glitch must be sustained for the full debounce window to change the output state.

### Quadrature Decoder

Both CLK and DT are packed into a 2-bit value (`CLK_low << 1 | DT_low`). The 4×4 transition table maps every `(previous, current)` pair to `+1` (CW), `-1` (CCW), or `0` (invalid):

```
             current state
prev state  00   01   10   11
  00      [  0,  -1,  +1,   0 ]
  01      [ +1,   0,   0,  -1 ]
  10      [ -1,   0,   0,  +1 ]
  11      [  0,  +1,  -1,   0 ]
```

Steps accumulate in `encoder_accumulator`. One logical click = **4 steps**, matching the 4 quadrature edges per physical detent of most mechanical encoders.

### Fast Rotation

`rotate_counter` tracks consecutive same-direction clicks. Once it exceeds `ROTATE_MULTI_COUNTER`, `poll` upgrades `Up`→`FastUp` or `Down`→`FastDown`. It resets after `ROTARY_RESET_TIME_MILLIS` idle cycles or on a direction reversal, so the fast mode naturally expires when the user slows down.

---

## Hardware Wiring

| Encoder Pin | MCU Pin        | Notes                                 |
|-------------|----------------|---------------------------------------|
| CLK (A)     | Any GPIO input | Enable internal pull-up               |
| DT  (B)     | Any GPIO input | Enable internal pull-up               |
| SW          | Any GPIO input | Active LOW — enable internal pull-up  |
| GND         | GND            |                                       |

> On STM32F1 with `stm32f1xx-hal`, configure CLK, DT, and SW as `into_pull_up_input()`.

---

## License

MIT License.

---

## Author

**Monib Mokhtari** — Embedded Systems Engineer  
[GitHub: MonibMo](https://github.com/MonibMo)
