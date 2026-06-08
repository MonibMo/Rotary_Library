# Rotary_Library

A `no_std`, interrupt-safe Rust driver for **quadrature rotary encoders** with an integrated push-button switch. Designed for embedded systems (e.g. STM32F1 + RTIC), built on top of [`embedded-hal 1.0`](https://docs.rs/embedded-hal/1.0.0/embedded_hal/).

---

## Features

- ✅ Hardware-agnostic via `embedded-hal` `InputPin` trait
- ✅ `no_std` compatible — no heap allocation
- ✅ Two-stage ISR / UI-task split for safe RTIC integration
- ✅ Integrating debounce filter for the push-button (glitch rejection)
- ✅ Full quadrature decoding using a 4×4 transition table
- ✅ Short press (`Select`) and long press (`SelectHold`) events
- ✅ Fast-rotation events (`FastUp` / `FastDown`) via multi-tick accumulator
- ✅ Optional `invert_rotation` Cargo feature to reverse direction without rewiring

---

## Architecture: Two-Stage Design

The driver separates **pin sampling** from **event decoding** into two distinct calling contexts:

| Method   | Calling context           | Frequency | What it does                                      |
|----------|---------------------------|-----------|---------------------------------------------------|
| `update` | Timer ISR (hardware task) | ~1–2 ms   | Samples pins, runs debounce, accumulates steps    |
| `poll`   | UI task / main loop       | ~10 ms    | Decodes accumulated state into `InputEvent`       |

> **Rule:** `update` **must** be called faster than `poll`. This ensures debouncing settles before events are consumed, and fast quadrature pulses are never missed.

---

## Button State Machine

```
                 [press detected]
  Idle ─────────────────────────────► Counting(0)

  Counting(n) ──[still held, n < HOLD_TICKS]──► Counting(n+1)
  Counting(n) ──[released before HOLD_TICKS]──► Counting(u16::MAX)  ← Select sentinel
  Counting(n) ──[n >= HOLD_TICKS]─────────────► WaitingRelease

  WaitingRelease ──[poll consumes]──► emit SelectHold  (exactly once)
  WaitingRelease ──[released]───────► Idle             (no extra event)
  Counting(u16::MAX) ──[poll reads]──► emit Select ──► Idle
```

**Guarantees:**
- `SelectHold` fires **exactly once** per long press.
- No event is emitted on release after a long press.
- `Select` only fires on genuine short presses.

---

## Configuration Constants

The driver reads from `crate::constants::tick_time`:

| Constant                  | Type  | Description                                                                          |
|---------------------------|-------|--------------------------------------------------------------------------------------|
| `ROTARY_DEBOUNCE_TICKS`   | `u8`  | Consecutive identical readings to settle a pin. Recommended: **3–5**                |
| `ROTARY_LONG_PRESS_TICKS` | `u16` | `update` ticks button must be held for `SelectHold`. At 1 ms → set `500` for 500 ms |

And from the crate root:

| Constant                   | Type  | Description                                                         |
|----------------------------|-------|---------------------------------------------------------------------|
| `ROTATE_MULTI_COUNTER`     | `u8`  | Consecutive same-direction ticks before `FastUp`/`FastDown` fires  |
| `ROTARY_RESET_TIME_MILLIS` | `u8`  | Idle ticks before the rotation streak counter resets               |

---

## Dependencies

```toml
[dependencies]
Rotary_Library = { path = "../Rotary_Library" }
embedded-hal = "1.0.0"
```

The crate also depends on `general_core` (workspace-internal) for the `InputEvent`, `InputSource`, and `RotaryAccumulatorMode` types.

---

## Feature Flags

| Feature           | Default | Effect                                                               |
|-------------------|---------|----------------------------------------------------------------------|
| `invert_rotation` | off     | Swaps `Up`/`Down` (and `FastUp`/`FastDown`) without changing wiring |

Enable in `Cargo.toml`:

```toml
[features]
invert_rotation = ["Rotary_Library/invert_rotation"]
```

---

## Usage Example (RTIC)

```rust
use Rotary_Library::RotaryEncoder;
use general_core::{InputEvent, InputSource};

// ── Initialization (in `init`) ──────────────────────────────────────────────
let encoder = RotaryEncoder::new(clk_pin, dt_pin, sw_pin);

// ── Timer ISR — called every 1 ms ───────────────────────────────────────────
#[task(binds = TIM2, shared = [encoder])]
fn tim2_isr(cx: tim2_isr::Context) {
    cx.shared.encoder.lock(|enc| enc.update());
}

// ── UI task — called every 10 ms ────────────────────────────────────────────
#[task(shared = [encoder])]
fn ui_task(cx: ui_task::Context) {
    let event = cx.shared.encoder.lock(|enc| enc.poll());

    match event {
        InputEvent::Up         => { /* scroll up        */ }
        InputEvent::Down       => { /* scroll down      */ }
        InputEvent::FastUp     => { /* fast scroll up   */ }
        InputEvent::FastDown   => { /* fast scroll down */ }
        InputEvent::Select     => { /* short press      */ }
        InputEvent::SelectHold => { /* long press       */ }
        InputEvent::None       => { /* idle             */ }
    }
}
```

---

## Internals

### Debounce Filter

The switch pin uses an **integrating debounce filter**:
- Pin LOW (pressed): counter increments toward `ROTARY_DEBOUNCE_TICKS`.
- Pin HIGH (released): counter decrements by 2 (faster release response).
- `is_pressed` latches to `true` only at saturation, and `false` only at zero.

Glitches shorter than `ROTARY_DEBOUNCE_TICKS` ticks are completely ignored.

### Quadrature Decoder

CLK and DT are encoded into a 2-bit state (`CLK_low << 1 | DT_low`). A 4×4 transition table maps every `(previous, current)` pair to a direction:

```
             current
prev    00   01   10   11
  00  [  0,  -1,  +1,   0 ]
  01  [ +1,   0,   0,  -1 ]
  10  [ -1,   0,   0,  +1 ]
  11  [  0,  +1,  -1,   0 ]
```

Steps accumulate in `encoder_accumulator`. One logical click = **4 steps** (`STEPS_PER_CLICK = 4`), matching the typical 4 edges per detent of most mechanical encoders.

### Fast Rotation

When `rotate_counter` (consecutive same-direction clicks) exceeds `ROTATE_MULTI_COUNTER`, `poll()` emits `FastUp` or `FastDown` instead of `Up`/`Down`. The counter resets after `ROTARY_RESET_TIME_MILLIS` idle ticks or on a direction change.

---

## Hardware Wiring

| Encoder Pin | MCU Pin        | Notes                                |
|-------------|----------------|--------------------------------------|
| CLK (A)     | Any GPIO input | Enable internal pull-up              |
| DT  (B)     | Any GPIO input | Enable internal pull-up              |
| SW          | Any GPIO input | Active LOW — enable internal pull-up |
| GND         | GND            |                                      |

> On STM32F1, configure CLK, DT, and SW as `Input<PullUp>` using the HAL.

---

## License

This project is licensed under the MIT License.

---

## Author

**Monib Mokhtari** — Embedded Systems Engineer  
[GitHub: MonibMo](https://github.com/MonibMo)
