//! # Rotary Encoder Driver
//!
//! An interrupt-safe driver for a standard **quadrature rotary encoder**
//! with an integrated push-button switch.
//!
//! ## Architecture: Two-Stage Design
//!
//! The driver is split across two calling contexts:
//!
//! | Method   | Calling context              | What it does                          |
//! |----------|------------------------------|---------------------------------------|
//! | `update` | Fast timer ISR (~1–2 ms)     | Samples pins, debounces, counts ticks |
//! | `poll`   | UI task / main loop (~10 ms) | Decodes accumulated state into events |
//!
//! `update` **must** be called faster than `poll` so that debouncing settles
//! before events are read and short quadrature pulses are never missed.
//!
//! ## Button State Machine
//!
//! The push-button lifecycle is modelled as an explicit linear state machine:
//!
//! ```text
//!                  [press detected]
//!   Idle ──────────────────────────────► Counting
//!
//!   Counting ──[released < HOLD_TICKS]──► emit Select  ──► Idle
//!   Counting ──[counter >= HOLD_TICKS]──► emit SelectHold ──► WaitingRelease
//!
//!   WaitingRelease ──[released]──────────────────────────► Idle  (no event)
//! ```
//!
//! This guarantees:
//! - `SelectHold` fires **exactly once** per long press.
//! - No event is emitted during the release after a long press.
//! - `Select` only fires for genuine short presses.
//!
//! ## Configuration Constants
//!
//! Provided by `crate::dd::constants::dd`:
//!
//! - `ROTARY_DEBOUNCE_TICKS` – consecutive identical readings required to
//!   settle a pin state. Recommended: 3–5.
//! - `long_press_tick` – number of `update` ticks the button must be
//!   held before [`InputEvent::SelectHold`] is generated. At a 1 ms `update`
//!   period, set this to `500` for a 500 ms long-press threshold.
//!
//! ## Feature Flag: `invert_rotation`
//!
//! Enable the `invert_rotation` Cargo feature to swap `Up`/`Down` if the
//! encoder feels reversed, without changing hardware wiring.
//!

mod types;
use embedded_hal::digital::InputPin;
use types::{InputEvent, InputSource, RotaryAccumulatorMode};
// ============================================================================
// Internal: integrating debounce filter
// ============================================================================

/// Integrating debounce filter for the push-button switch.
///
/// On every [`Button::update`] call the counter moves one step towards its
/// saturation point:
/// - Pin LOW  (pressed)  → counter increments up to `ROTARY_DEBOUNCE_TICKS`.
/// - Pin HIGH (released) → counter decrements down to `0`.
///
/// The debounced state only latches when the counter reaches either extreme,
/// so short glitches that reverse direction before saturation are ignored.
#[derive(Clone, Copy, Default)]
struct Button {
    /// Integration counter in `[0, ROTARY_DEBOUNCE_TICKS]`.
    /// `0` = definitely released, `ROTARY_DEBOUNCE_TICKS` = definitely pressed.
    counter: u8,

    /// The settled, glitch-free pressed state.
    is_pressed: bool,

    /// Debounce Tick
    button_debounce_tick: u8,
}

impl Button {
    pub fn new(button_debounce_tick: u8) -> Self {
        Self {
            counter: 0,
            is_pressed: false,
            button_debounce_tick,
        }
    }
    /// Advance the debounce filter with one raw pin sample.
    ///
    /// Call every timer ISR tick_idle, inside [`RotaryEncoder::update`].
    ///
    /// # Parameters
    /// - `pin_is_low`: `true` when the switch pin reads LOW (button pressed,
    ///   active-low with pull-up resistor).
    fn update(&mut self, pin_is_low: bool) {
        if pin_is_low {
            // Integrate towards "pressed", saturate at the threshold.
            self.counter = self
                .counter
                .saturating_add(1)
                .min(self.button_debounce_tick);
        } else {
            // Integrate towards "released".
            self.counter = self.counter.saturating_sub(2);
        }

        // Only latch a state change when the counter reaches an extreme,
        // ensuring the signal has been stable for ROTARY_DEBOUNCE_TICKS ticks.
        if self.counter >= self.button_debounce_tick {
            self.is_pressed = true;
        } else if self.counter == 0 {
            self.is_pressed = false;
        }
    }
}

// ============================================================================
// Internal: button press/hold/release state machine
// ============================================================================

/// States of the button press lifecycle.
///
/// The machine advances strictly forward per press cycle:
/// `Idle → Counting → WaitingRelease → Idle`
/// or for a short press:
/// `Idle → Counting(u16::MAX sentinel) → Idle`
#[derive(Clone, Copy, PartialEq, Eq)]
enum ButtonState {
    /// Button is not pressed. Waiting for the next press.
    Idle,

    /// Button is held. The inner value counts elapsed ticks since the press.
    ///
    /// The special sentinel `u16::MAX` means the button was **released before
    /// the hold threshold** was reached. `poll()` uses this to emit `Select`.
    Counting(u16),
    /// Hold threshold crossed, SelectHold not yet emitted.
    // HoldPending,
    /// SelectHold already emitted, silently waiting for physical release.
    WaitingRelease,
}

// ============================================================================
// RotaryEncoder
// ============================================================================

/// Required quadrature step count for one logical encoder "click".
///
/// Most mechanical encoders produce 4 quadrature edges per physical detent,
/// so `STEPS_PER_CLICK = 4` maps exactly one detent to one [`InputEvent`].
const STEPS_PER_CLICK: i8 = 4;

/// Quadrature rotary encoder driver with integrated push-button.
///
/// # Type Parameters
///
/// - `CLK`: CLK / phase-A input pin, implementing [`InputPin`].
/// - `DT`:  DT  / phase-B input pin, implementing [`InputPin`].
/// - `SW`:  SW  / switch  input pin, implementing [`InputPin`].
///
/// # Usage
///
/// ```rust,ignore
/// let mut encoder = RotaryEncoder::new(clk_pin, dt_pin, sw_pin);
///
/// // Inside a 1 ms periodic timer ISR:
/// encoder.update();
///
/// // Inside the UI main loop (~10 ms period):
/// match encoder.poll() {
///     InputEvent::Up         => { /* scroll up   */ }
///     InputEvent::Down       => { /* scroll down */ }
///     InputEvent::Select     => { /* short press */ }
///     InputEvent::SelectHold => { /* long press  */ }
///     InputEvent::None       => {}
/// }
/// ```
pub struct RotaryEncoder<CLK, DT, SW>
where
    CLK: InputPin,
    DT: InputPin,
    SW: InputPin,
{
    clk_pin: CLK,
    dt_pin: DT,
    sw_pin: SW,

    /// Low-level integrating debounce filter for the switch pin.
    button: Button,

    /// High-level press/hold/release state machine driven by `button`.
    button_state: ButtonState,

    /// A flag for state managing of hold event
    hold_generated: bool,

    /// Long press ticks
    long_press_threshold_tick: u16,

    /// Last sampled quadrature state (2 bits: CLK_low << 1 | DT_low).
    last_quad_state: u8,

    /// Running total of quadrature steps since the last `poll`.
    /// Positive = clockwise, negative = counter-clockwise.
    encoder_accumulator: i8,

    /// An accumulator for saving rotation direction independent of rotary model. clockwise is + and
    /// anti-clockwise is -
    last_accumulator: RotaryAccumulatorMode,

    /// A counter for resetting the rotary when there is no change in input pins
    reset_counter: u8,

    /// Reset time for fast moving up or down
    rotary_reset_time_threshold_ms: u16,

    /// A counter for rotation steps
    rotate_counter: u8,

    /// A counter for dedicating when the rotary is going to fast mode rotating
    rotary_multi_step_threshold: u8
}

impl<CLK, DT, SW> RotaryEncoder<CLK, DT, SW>
where
    CLK: InputPin,
    DT: InputPin,
    SW: InputPin,
{
    /// Construct a new [`RotaryEncoder`], capturing the initial pin state.
    ///
    /// Reading the initial quadrature state here prevents a spurious first
    /// edge from being registered on the first `update` call.
    ///
    /// # Parameters
    /// - `clk_pin`: CLK / phase-A input.
    /// - `dt_pin`:  DT  / phase-B input.
    /// - `sw_pin`:  SW  / switch input (active LOW, external pull-up assumed).
    pub fn new(
        mut clk_pin: CLK,
        mut dt_pin: DT,
        sw_pin: SW,
        button_debounce_tick: u8,
        long_press_tick: u16,
        rotary_reset_time_ms: u16,
        rotary_multi_step_threshold: u8
    ) -> Self {
        let clk_low = clk_pin.is_low().unwrap_or(false);
        let dt_low = dt_pin.is_low().unwrap_or(false);
        let initial_state = (clk_low as u8) << 1 | (dt_low as u8);

        Self {
            clk_pin,
            dt_pin,
            sw_pin,
            long_press_threshold_tick: long_press_tick,
            button: Button::new(button_debounce_tick),
            button_state: ButtonState::Idle,
            last_quad_state: initial_state,
            encoder_accumulator: 0,
            hold_generated: false,
            rotate_counter: 0,
            last_accumulator: RotaryAccumulatorMode::None,
            reset_counter: 0,
            rotary_reset_time_threshold_ms: rotary_reset_time_ms,
            rotary_multi_step_threshold
        }
    }

    /// Sample all encoder pins and advance internal state.
    ///
    /// **Call from a periodic 1–2 ms timer interrupt.**
    ///
    /// Each call performs three independent tasks:
    /// 1. Advances the debounce integrator for the switch pin.
    /// 2. Advances the press/hold/release state machine.
    /// 3. Decodes one quadrature step and adds it to the accumulator.
    pub fn update(&mut self) {
        // ── 1. Debounce the switch ────────────────────────────────────────────
        let sw_low = self.sw_pin.is_low().unwrap_or(false);
        self.button.update(sw_low);

        // ── 2. Button state machine ───────────────────────────────────────────
        //
        // Advance based on the settled debounced press signal.
        // Events are NOT emitted here — update() only builds state.
        // poll() reads and consumes it, keeping ISR work minimal.
        self.button_state = match self.button_state {
            ButtonState::Idle => {
                if self.button.is_pressed {
                    // Press edge: begin counting hold ticks.
                    ButtonState::Counting(0)
                } else {
                    ButtonState::Idle
                }
            }

            ButtonState::Counting(ticks) => {
                if self.button.is_pressed {
                    if ticks >= self.long_press_threshold_tick {
                        // Hold threshold crossed — poll() will emit SelectHold.
                        // Move to WaitingRelease to block any further events
                        // until the button is physically released.
                        // ButtonState::HoldPending
                        ButtonState::WaitingRelease
                    } else {
                        // Still within the hold window — keep counting.
                        ButtonState::Counting(ticks.saturating_add(1))
                    }
                } else {
                    // Button released before the hold threshold was reached.
                    // Signal this to poll() using the u16::MAX sentinel so it
                    // knows to emit Select and then return to Idle.
                    ButtonState::Counting(u16::MAX)
                }
            }

            // ButtonState::HoldPending => {
            //     if self.button.is_pressed {
            //         ButtonState::HoldPending   // stay until poll() consumes it
            //     } else {
            //         ButtonState::Idle          // released before poll() even saw it — skip event
            //     }
            // }
            ButtonState::WaitingRelease => {
                if self.button.is_pressed {
                    ButtonState::WaitingRelease
                } else {
                    self.hold_generated = false;
                    ButtonState::Idle
                }
            }
        };

        // ── 3. Quadrature decoding ────────────────────────────────────────────
        //
        // Encode both pin levels into a 2-bit state:
        //   bit 1 = CLK_low,  bit 0 = DT_low
        //
        // The 4×4 transition table maps every (prev, current) pair to a
        // direction: +1 (CW), -1 (CCW), or 0 (invalid/noise).
        // Index = prev (high 2 bits) | current (low 2 bits).
        #[rustfmt::skip]
        const QUAD_TABLE: [i8; 16] = [
             0, -1,  1,  0,   // prev = 00
             1,  0,  0, -1,   // prev = 01
            -1,  0,  0,  1,   // prev = 10
             0,  1, -1,  0,   // prev = 11
        ];

        let clk_low = self.clk_pin.is_low().unwrap_or(false);
        let dt_low = self.dt_pin.is_low().unwrap_or(false);
        let current = (clk_low as u8) << 1 | (dt_low as u8);

        if current != self.last_quad_state {
            let index = ((self.last_quad_state << 2) | current) as usize;
            self.encoder_accumulator = self.encoder_accumulator.saturating_add(QUAD_TABLE[index]);
            self.last_quad_state = current;
        }
    }
}

// ============================================================================
// InputSource impl  (poll)
// ============================================================================

impl<CLK, DT, SW> InputSource for RotaryEncoder<CLK, DT, SW>
where
    CLK: InputPin,
    DT: InputPin,
    SW: InputPin,
{
    /// Decode accumulated encoder and button state into the next [`InputEvent`].
    ///
    /// **Call from the UI task / main loop — never from an ISR.**
    ///
    /// Returns at most one event per call. Priority order:
    ///
    /// 1. **Rotation** — if `|accumulator| >= STEPS_PER_CLICK`, emits `Up` or
    ///    `Down` and clears the accumulator.
    /// 2. **SelectHold** — emitted once when `button_state` is `WaitingRelease`
    ///    and the button is still physically held.
    /// 3. **Select** — emitted when `button_state` is `Counting(u16::MAX)`,
    ///    meaning the button was released before the hold threshold.
    /// 4. **None** — nothing to report this cycle.
    fn poll(&mut self) -> InputEvent {
        // ── 1. Rotation ───────────────────────────────────────────────────────
        if self.encoder_accumulator >= STEPS_PER_CLICK {
            self.encoder_accumulator = 0;
            self.reset_counter = 0;
            if self.last_accumulator == RotaryAccumulatorMode::Positive {
                self.rotate_counter += 1;
            } else {
                self.rotate_counter = 1;
            }
            self.last_accumulator = RotaryAccumulatorMode::Positive;
            return if cfg!(feature = "invert_rotation") {
                if self.rotate_counter > self.rotary_multi_step_threshold {
                    InputEvent::FastDown
                } else {
                    InputEvent::Down
                }
            } else {
                if self.rotate_counter > self.rotary_multi_step_threshold {
                    InputEvent::FastUp
                } else {
                    InputEvent::Up
                }
            };
        }
        if self.encoder_accumulator <= -STEPS_PER_CLICK {
            self.encoder_accumulator = 0;
            self.reset_counter = 0;
            if self.last_accumulator == RotaryAccumulatorMode::Negative {
                self.rotate_counter += 1;
            } else {
                self.rotate_counter = 1;
            }
            self.last_accumulator = RotaryAccumulatorMode::Negative;
            return if cfg!(feature = "invert_rotation") {
                if self.rotate_counter >= self.rotary_multi_step_threshold {
                    InputEvent::FastUp
                } else {
                    InputEvent::Up
                }
            } else {
                if self.rotate_counter >= self.rotary_multi_step_threshold {
                    InputEvent::FastDown
                } else {
                    InputEvent::Down
                }
            };
        };

        // ── 2. Long press ─────────────────────────────────────────────────────
        if self.button_state == ButtonState::WaitingRelease && !self.hold_generated {
            // Consume the pending hold event exactly once,
            // then move to WaitingRelease which emits nothing.
            self.last_accumulator = RotaryAccumulatorMode::None;
            self.rotate_counter = 0;
            self.reset_counter = 0;
            self.hold_generated = true;
            return InputEvent::SelectHold;
        }

        // ── 3. Short press — emit Select on release ───────────────────────────
        //
        // Counting(u16::MAX) is the sentinel set by update() when the button
        // was released before the hold threshold. Consume it here and reset.
        if self.button_state == ButtonState::Counting(u16::MAX) {
            self.last_accumulator = RotaryAccumulatorMode::None;
            self.rotate_counter = 0;
            self.reset_counter = 0;
            self.button_state = ButtonState::Idle;
            return InputEvent::Select;
        }
        self.reset_counter += 1;
        if self.reset_counter > self.rotary_reset_time_threshold_ms as u8 {
            self.reset_counter = 0;
            self.last_accumulator = RotaryAccumulatorMode::None;
            self.rotate_counter = 0;
        }
        // ── 4. Nothing ────────────────────────────────────────────────────────
        InputEvent::None
    }
}
