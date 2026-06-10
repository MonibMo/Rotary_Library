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
