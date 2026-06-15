#![no_std]
//! # Rotary Encoder Driver
//!
//! An interrupt-safe driver for a standard quadrature rotary encoder
//! with an integrated push-button switch.
//!
//! ## Architecture
//!
//! The driver is split into two layers:
//!
//! - [`RotaryEncoder::update`] is called from a fast periodic context,
//!   typically a timer interrupt.
//! - [`InputSource::poll`] is called from a slower UI task or main loop.
//!
//! The rotary part accumulates quadrature movement in `update()` and
//! converts it into high-level events in `poll()`.
//!
//! The button part is fully self-contained:
//!
//! - [`Button`] owns its own switch pin.
//! - It performs raw sampling, debouncing, press/hold/release tracking,
//!   and event buffering internally.
//! - [`RotaryEncoder`] only delegates button sampling to `self.button.update()`
//!   and later consumes buffered button events through `self.button.poll()`.
//!
//! ## Typical timing
//!
//! - `update()` period: about 1–2 ms
//! - `poll()` period: about 5–20 ms
//!
//! `update()` should run faster than `poll()` so that debounce filtering
//! settles correctly and short quadrature pulses are not missed.

mod types;

use embedded_hal::digital::InputPin;
pub use types::{InputEvent, InputSource, RotaryAccumulatorMode,ButtonDefaultState, ButtonState};

/// Number of quadrature steps required to produce one logical rotary event.
///
/// Many mechanical encoders generate four valid transitions per detent,
/// so `4` maps one physical click to one UI movement event.
const STEPS_PER_CLICK: i8 = 4;

// ============================================================================
// Button
// ============================================================================

/// Self-contained push-button handler with debounce and hold detection.
///
/// This type owns the switch pin and performs:
///
/// - raw pin sampling,
/// - integrating debounce,
/// - short press detection,
/// - long press detection,
/// - event buffering for later consumption.
///
/// The electrical meaning of “pressed” depends on [`ButtonDefaultState`]:
///
/// - [`ButtonDefaultState::PulledUp`]   => pin LOW means pressed
/// - [`ButtonDefaultState::PulledDown`] => pin HIGH means pressed
///
/// # Timing model
///
/// Call [`Button::update`] from a fast periodic task, typically every 1 ms.
/// Then call [`Button::poll`] from a slower task to retrieve generated events.
///
/// # Event behavior
///
/// - A short press generates [`InputEvent::Select`] once, on release.
/// - A long press generates [`InputEvent::SelectHold`] once, when the hold
///   threshold is crossed.
/// - Releasing after a long press produces no extra event.
pub struct Button<PIN>
where
    PIN: InputPin,
{
    /// Physical switch pin.
    pin: PIN,

    /// Electrical idle configuration of the switch input.
    ///
    /// This determines which raw logic level is interpreted as a press.
    button_default_state: ButtonDefaultState,

    /// Integration counter used for debounce filtering.
    ///
    /// Range: `0..=button_debounce_tick`
    ///
    /// - `0` means confidently released
    /// - `button_debounce_tick` means confidently pressed
    debounce_counter: u8,

    /// Current debounced logical button state.
    is_pressed: bool,

    /// Number of stable update ticks required to settle a pressed state.
    button_debounce_tick: u8,

    /// Number of update ticks required to classify a press as a hold.
    long_press_threshold_tick: u16,

    /// High-level press lifecycle state.
    state: ButtonState,

    /// Buffered event produced by [`Button::update`] and consumed by
    /// [`Button::poll`].
    pending_event: InputEvent,
}

impl<PIN> Button<PIN>
where
    PIN: InputPin,
{
    /// Creates a new button handler.
    ///
    /// # Parameters
    ///
    /// - `pin`: Physical button input pin
    /// - `button_default_state`: Electrical idle configuration of the pin
    /// - `button_debounce_tick`: Debounce threshold in update ticks, should be non-zero
    /// - `long_press_threshold_tick`: Hold threshold in update ticks
    ///
    /// # Notes
    ///
    /// The initial debounced state starts as released.
    pub fn new(
        pin: PIN,
        button_default_state: ButtonDefaultState,
        button_debounce_tick: u8,
        long_press_threshold_tick: u16,
    ) -> Self {
        Self {
            pin,
            button_default_state,
            debounce_counter: 0,
            is_pressed: false,
            button_debounce_tick,
            long_press_threshold_tick,
            state: ButtonState::default(),
            pending_event: InputEvent::None,
        }
    }

    /// Interprets the raw GPIO level as a logical “pressed” state.
    ///
    /// The meaning of HIGH/LOW depends on the configured button wiring.
    fn examine_pressed(&mut self) -> bool {
        match self.button_default_state {
            ButtonDefaultState::PulledDown => self.pin.is_high().unwrap_or(false),
            ButtonDefaultState::PulledUp => self.pin.is_low().unwrap_or(false),
        }
    }

    /// Samples the switch pin and advances all button logic by one tick.
    ///
    /// This method performs three steps:
    ///
    /// 1. Reads the raw switch pin
    /// 2. Applies integrating debounce
    /// 3. Advances the press/hold/release state machine
    ///
    /// Any generated event is buffered internally and can later be
    /// retrieved using [`Button::poll`].
    ///
    /// # Calling context
    ///
    /// This method is intended for a fast periodic context, such as
    /// a timer ISR running every 1–2 ms.
    pub fn update(&mut self) {
        let key_pressed = self.examine_pressed();

        // --------------------------------------------------------------------
        // 1. Debounce integrator
        // --------------------------------------------------------------------
        //
        // This is an integrating debounce filter:
        //
        // - If the raw input says "pressed", the counter moves upward
        //   toward the pressed threshold.
        // - If the raw input says "released", the counter moves downward
        //   toward zero.
        //
        // The debounced state only changes when the counter reaches one
        // extreme or the other, which suppresses short glitches.
        if key_pressed {
            self.debounce_counter = self
                .debounce_counter
                .saturating_add(1)
                .min(self.button_debounce_tick);
        } else {
            self.debounce_counter = self.debounce_counter.saturating_sub(2);
        }

        if self.debounce_counter >= self.button_debounce_tick {
            self.is_pressed = true;
        } else if self.debounce_counter == 0 {
            self.is_pressed = false;
        }

        // --------------------------------------------------------------------
        // 2. High-level button state machine
        // --------------------------------------------------------------------
        //
        // This logic translates the debounced boolean state into high-level
        // user events:
        //
        // - short press => Select
        // - long press  => SelectHold
        //
        // The event is stored into `pending_event` and later consumed by poll().
        self.state = match self.state {
            ButtonState::Idle => {
                if self.is_pressed {
                    ButtonState::Counting(0)
                } else {
                    ButtonState::Idle
                }
            }

            ButtonState::Counting(ticks) => {
                if self.is_pressed {
                    if ticks >= self.long_press_threshold_tick {
                        // Generate the hold event exactly once.
                        if self.pending_event == InputEvent::None {
                            self.pending_event = InputEvent::SelectHold;
                        }
                        ButtonState::WaitingRelease
                    } else {
                        ButtonState::Counting(ticks.saturating_add(1))
                    }
                } else {
                    // Released before the hold threshold was reached:
                    // this is a short press.
                    if self.pending_event == InputEvent::None {
                        self.pending_event = InputEvent::Select;
                    }
                    ButtonState::Idle
                }
            }

            ButtonState::WaitingRelease => {
                if self.is_pressed {
                    ButtonState::WaitingRelease
                } else {
                    // Release after a long press generates no extra event.
                    ButtonState::Idle
                }
            }
        };
    }

    pub fn is_idle(&self) -> bool {
        self.state == ButtonState::Idle
    }

    /// Returns the next buffered button event.
    ///
    /// This consumes any pending event generated by [`Button::update`].
    ///
    /// # Returns
    ///
    /// - [`InputEvent::Select`] for a short press
    /// - [`InputEvent::SelectHold`] for a long press
    /// - [`InputEvent::None`] if no button event is pending
    pub fn poll(&mut self) -> InputEvent {
        let event = self.pending_event;
        self.pending_event = InputEvent::None;
        event
    }

    /// Returns the current debounced pressed state.
    ///
    /// This is useful when the caller needs the current stable button level
    /// rather than edge-triggered events.
    pub fn is_pressed(&self) -> bool {
        self.is_pressed
    }
}

// ============================================================================
// Rotary encoder
// ============================================================================

/// Quadrature rotary encoder driver with an internal push-button handler.
///
/// # Type Parameters
///
/// - `CLK`: Phase-A / CLK input pin
/// - `DT`: Phase-B / DT input pin
/// - `SW`: Push-button switch input pin
///
/// # Design
///
/// The rotary encoder owns the quadrature pins directly and owns a
/// [`Button<SW>`] instance for the switch.
///
/// Responsibilities are split cleanly:
///
/// - [`Button`] handles switch sampling, debounce, and button events
/// - [`RotaryEncoder`] handles quadrature decoding and merges all sources
///   into one [`InputEvent`] stream
pub struct RotaryEncoder<CLK, DT, SW>
where
    CLK: InputPin,
    DT: InputPin,
    SW: InputPin,
{
    /// Quadrature phase-A pin.
    clk_pin: CLK,

    /// Quadrature phase-B pin.
    dt_pin: DT,

    /// Self-contained push-button handler.
    button: Button<SW>,

    /// Last sampled quadrature state encoded as two bits.
    ///
    /// Encoding:
    /// - bit 1 = CLK low
    /// - bit 0 = DT low
    last_quad_state: u8,

    /// Accumulated quadrature delta since the last emitted rotation event.
    ///
    /// Positive values represent one direction and negative values the other.
    encoder_accumulator: i8,

    /// Remembers the last movement direction for fast-rotation logic.
    last_accumulator: RotaryAccumulatorMode,

    /// Counter used to reset fast-rotation tracking after inactivity.
    reset_counter: u8,

    /// Inactivity threshold after which the fast-rotation streak is cleared.
    rotary_reset_time_threshold_ms: u16,

    /// Number of consecutive steps in the same direction.
    rotate_counter: u8,

    /// Threshold after which normal Up/Down becomes FastUp/FastDown.
    rotary_multi_step_threshold: u8,
}

impl<CLK, DT, SW> RotaryEncoder<CLK, DT, SW>
where
    CLK: InputPin,
    DT: InputPin,
    SW: InputPin,
{
    /// Creates a new rotary encoder driver.
    ///
    /// # Parameters
    ///
    /// - `clk_pin`: Encoder CLK / phase-A input
    /// - `dt_pin`: Encoder DT / phase-B input
    /// - `sw_pin`: Encoder push-button input
    /// - `button_default_state`: Electrical idle configuration of the button input
    /// - `button_debounce_tick`: Debounce threshold for the button, should be non-zero
    /// - `long_press_tick`: Long-press threshold for the button
    /// - `rotary_reset_time_ms`: Idle time before fast-rotation tracking resets
    /// - `rotary_multi_step_threshold`: Number of same-direction steps before generating fast rotation events
    ///
    /// # Notes
    ///
    /// The initial quadrature state is captured immediately to avoid a false
    /// first transition on the first update cycle.
    pub fn new(
        mut clk_pin: CLK,
        mut dt_pin: DT,
        sw_pin: SW,
        button_default_state: ButtonDefaultState,
        button_debounce_tick: u8,
        long_press_tick: u16,
        rotary_reset_time_ms: u16,
        rotary_multi_step_threshold: u8,
    ) -> Self {
        let clk_low = clk_pin.is_low().unwrap_or(false);
        let dt_low = dt_pin.is_low().unwrap_or(false);
        let initial_state = (clk_low as u8) << 1 | (dt_low as u8);

        Self {
            clk_pin,
            dt_pin,
            button: Button::new(
                sw_pin,
                button_default_state,
                button_debounce_tick,
                long_press_tick,
            ),
            last_quad_state: initial_state,
            encoder_accumulator: 0,
            last_accumulator: RotaryAccumulatorMode::None,
            reset_counter: 0,
            rotary_reset_time_threshold_ms: rotary_reset_time_ms,
            rotate_counter: 0,
            rotary_multi_step_threshold,
        }
    }

    /// Samples all hardware inputs and advances internal state by one tick.
    ///
    /// This method should be called from a fast periodic context, such as
    /// a timer ISR.
    ///
    /// It performs two independent jobs:
    ///
    /// 1. Updates the internal button handler
    /// 2. Samples and decodes one quadrature transition
    pub fn update(&mut self) {
        // --------------------------------------------------------------------
        // 1. Delegate all switch logic to the button object
        // --------------------------------------------------------------------
        self.button.update();

        // --------------------------------------------------------------------
        // 2. Decode quadrature movement
        // --------------------------------------------------------------------
        //
        // The encoder state is encoded as:
        //
        //   bit 1 = CLK_low
        //   bit 0 = DT_low
        //
        // The transition table converts (previous_state, current_state)
        // into:
        //
        // - +1 for one valid direction
        // - -1 for the other valid direction
        // -  0 for invalid, noisy, or unchanged transitions
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
            self.encoder_accumulator =
                self.encoder_accumulator.saturating_add(QUAD_TABLE[index]);
            self.last_quad_state = current;
        }
    }

    /// Returns the current debounced button pressed state.
    ///
    /// This forwards to the internal [`Button`] object.
    pub fn button_is_pressed(&self) -> bool {
        self.button.is_pressed()
    }
}

impl<CLK, DT, SW> InputSource for RotaryEncoder<CLK, DT, SW>
where
    CLK: InputPin,
    DT: InputPin,
    SW: InputPin,
{
    /// Returns the next high-level input event.
    ///
    /// Priority order:
    ///
    /// 1. Rotation events
    /// 2. Button events
    /// 3. No event
    ///
    /// Rotation is prioritized so that rapid movement is not delayed
    /// behind button handling.
    fn poll(&mut self) -> InputEvent {
        // --------------------------------------------------------------------
        // 1. Rotation events
        // --------------------------------------------------------------------
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
            } else if self.rotate_counter > self.rotary_multi_step_threshold {
                InputEvent::FastUp
            } else {
                InputEvent::Up
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
            } else if self.rotate_counter >= self.rotary_multi_step_threshold {
                InputEvent::FastDown
            } else {
                InputEvent::Down
            };
        }

        // --------------------------------------------------------------------
        // 2. Button events
        // --------------------------------------------------------------------
        let button_event = self.button.poll();
        if button_event != InputEvent::None {
            self.last_accumulator = RotaryAccumulatorMode::None;
            self.rotate_counter = 0;
            self.reset_counter = 0;
            return button_event;
        }

        // --------------------------------------------------------------------
        // 3. Reset fast-rotation tracking after inactivity
        // --------------------------------------------------------------------
        self.reset_counter = self.reset_counter.saturating_add(1);

        if self.reset_counter > self.rotary_reset_time_threshold_ms as u8 {
            self.reset_counter = 0;
            self.last_accumulator = RotaryAccumulatorMode::None;
            self.rotate_counter = 0;
        }

        InputEvent::None
    }
}