# Rotary_Library

A `no_std`, interrupt-safe driver for **quadrature rotary encoders** with an integrated push-button.

Built on top of `embedded-hal 1.0`, designed for deterministic behavior in bare-metal embedded systems (e.g. STM32).

---

## Overview

This driver follows a **two-stage design**:

- `update()` → fast, deterministic, ISR-safe
- `poll()`   → slower, user-facing event decoding

This separation ensures:
- No missed quadrature edges
- Stable debounce behavior
- Minimal ISR workload

---

## Architecture

| Method   | Context            | Typical Rate | Responsibility                         |
|----------|--------------------|--------------|----------------------------------------|
| `update` | Timer ISR          | 1–2 ms       | Sample pins, debounce, accumulate data |
| `poll`   | Main loop / task   | 10–50 ms     | Convert state → `InputEvent`           |

> `update()` must run faster than `poll()`.

---

## RotaryEncoder Structure

The driver stores all runtime state internally. Timing and behavior are controlled through these fields:

```rust
pub struct RotaryEncoder<CLK, DT, SW>
where
    CLK: InputPin,
    DT: InputPin,
    SW: InputPin,
{
    // ── Hardware pins ────────────────────────────────────────────────────
    clk_pin: CLK,
    dt_pin:  DT,
    sw_pin:  SW,

    // ── Button debounce (integrating filter) ─────────────────────────────
    // Counter moves toward:
    //   - MAX when pressed (LOW)
    //   - 0   when released (HIGH)
    //
    // Only when the counter saturates do we accept a stable state.
    // This rejects short glitches.
    //
    // Equivalent concept to:
    //   "debounce window length" = ROTARY_DEBOUNCE_TICKS
    button: Button,

    // ── Button state machine ─────────────────────────────────────────────
    //
    // Idle → Counting → WaitingRelease → Idle
    //
    // - Counts how long button is held
    // - Detects short vs long press
    //
    // Long press threshold:
    //   ROTARY_LONG_PRESS_TICKS (in update ticks)
    //
    button_state: ButtonState,

    // ── Quadrature decoding ──────────────────────────────────────────────
    //
    // Accumulates +1 / -1 per valid transition.
    // One full detent = 4 steps.
    //
    encoder_accumulator: i8,
    last_quad_state: u8,

    // ── Fast rotation detection ──────────────────────────────────────────
    //
    // Counts consecutive same-direction clicks.
    //
    // If threshold exceeded:
    //   → FastUp / FastDown is emitted
    //
    // Threshold:
    //   ROTATE_MULTI_COUNTER
    //
    rotate_counter: u8,

    // Last direction (for streak tracking)
    last_accumulator: RotaryAccumulatorMode,

    // Idle timeout counter:
    //
    // If no movement occurs for:
    //   ROTARY_RESET_TIME_MILLIS (poll cycles)
    //
    // then fast-rotation streak resets.
    reset_counter: u8,

    // Ensures SelectHold is emitted only once
    hold_generated: bool,
}
```

---

## Configuration Parameters (Conceptual)

These are not part of the struct API but define behavior:

- **Debounce window**
  - Number of stable samples required before accepting a button state
  - Typical: `3–5` ticks

- **Long press threshold**
  - Duration (in `update()` ticks) required to trigger `SelectHold`
  - Example: `500` at 1 ms → 500 ms

- **Fast rotation threshold**
  - Number of consecutive same-direction clicks before switching to fast mode

- **Reset timeout**
  - Idle time before fast-rotation streak resets

---

## Button State Machine

```
Idle
 └─(press)────────────► Counting(0)

Counting(n)
 ├─(held < threshold)─► Counting(n+1)
 ├─(released early)───► Counting(u16::MAX) → Select
 └─(held ≥ threshold)► WaitingRelease → SelectHold

WaitingRelease
 └─(release)─────────► Idle
```

### Guarantees

- `SelectHold` fires **once only**
- No extra event on release after hold
- `Select` only fires for short presses

---

## Quadrature Decoding

Each pin pair is encoded into a 2-bit state:

```
state = (CLK_low << 1) | DT_low
```

A 4×4 transition table determines direction:

```
prev → current

00 → 01 = -1
00 → 10 = +1
...
```

- Invalid transitions → ignored
- Accumulator collects steps
- 4 steps = 1 click

---

## Fast Rotation

When rotation continues in the same direction:

- `rotate_counter` increments
- After threshold → emits `FastUp` / `FastDown`

Resets when:
- Direction changes
- Idle timeout expires

---

## Usage Example (Bare-Metal)

```rust
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use Rotary_Library::RotaryEncoder;
use general_core::{InputEvent, InputSource};

#[entry]
fn main() -> ! {
    // init clocks, GPIO, etc...

    let mut encoder = RotaryEncoder::new(clk, dt, sw);

    let mut tick: u8 = 0;

    loop {
        // ── Fast update (~1 ms) ───────────────────────────
        encoder.update();

        delay_ms(1);

        // ── Slower poll (~10 ms) ──────────────────────────
        tick += 1;
        if tick >= 10 {
            tick = 0;

            match encoder.poll() {
                InputEvent::Up         => {}
                InputEvent::Down       => {}
                InputEvent::FastUp     => {}
                InputEvent::FastDown   => {}
                InputEvent::Select     => {}
                InputEvent::SelectHold => {}
                InputEvent::None       => {}
            }
        }
    }
}
```

---

## Hardware Notes

- All inputs should be **pull-up**
- Button is **active LOW**
- Typical encoder: 4 edges per detent → matches internal step logic

---

## Features

- `no_std`
- ISR-safe design
- Glitch-resistant debounce
- Deterministic behavior
- Fast rotation detection
- Optional `invert_rotation` feature flag

---

## License

MIT

---

## Author

Monib Mokhtari  
https://github.com/MonibMo
