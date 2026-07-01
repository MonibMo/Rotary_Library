#![no_std]

mod types;

pub use crate::types::ButtonState;
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
pub use types::{InputEvent, InputSource, RotaryAccumulatorMode};

const STEPS_PER_CLICK: i8 = 4;

// ─── ButtonBehaviour trait ────────────────────────────────────────────────────

pub trait ButtonBehaviour: PartialEq {
    fn emit_select_on_hold() -> bool;
    /// Called when the hold threshold is crossed.
    /// `repeat_count` = how many holds have already fired this press (0 = first).
    fn on_hold(repeat_count: u16) -> HoldAction;

    /// Called on physical release.
    /// Return true to emit Select, false to swallow it.
    fn emit_select_on_release(hold_was_emitted: bool) -> bool;

    /// The interval used for the 2nd, 3rd, ... holds.
    /// Only consulted by EmitAndReset behaviours.
    fn repeat_threshold_millis(long_press_threshold_millis: u16) -> u16;
}

pub enum HoldAction {
    /// Fire hold once, then wait silently for release.
    EmitAndWaitRelease,
    /// Fire hold, reset counter to 0, stay in Counting for next repeat.
    EmitAndReset,
}

// ─── Marker structs ───────────────────────────────────────────────────────────

#[derive(PartialEq)]
pub struct RotaryButton;
#[derive(PartialEq)]
pub struct HoodButton1;
#[derive(PartialEq)]
pub struct PushButton;

#[derive(PartialEq)]
pub struct HoodButton2;

impl ButtonBehaviour for RotaryButton {
    fn emit_select_on_hold() -> bool {
        false
    }

    fn on_hold(_: u16) -> HoldAction {
        HoldAction::EmitAndWaitRelease
    }
    fn emit_select_on_release(_: bool) -> bool {
        true
    }
    fn repeat_threshold_millis(t: u16) -> u16 {
        t
    }
}

impl ButtonBehaviour for HoodButton1 {
    fn emit_select_on_hold() -> bool {
        false
    }
    fn on_hold(_: u16) -> HoldAction {
        HoldAction::EmitAndReset
    }
    fn emit_select_on_release(hold_emitted: bool) -> bool {
        !hold_emitted
    }
    /// Repeats fire at 1/3 of the initial hold threshold → faster feel.
    fn repeat_threshold_millis(t: u16) -> u16 {
        (t / 3).max(1)
    }
}

impl ButtonBehaviour for HoodButton2 {
    fn emit_select_on_hold() -> bool {
        true
    }
    fn on_hold(_: u16) -> HoldAction {
        HoldAction::EmitAndReset
    }
    fn emit_select_on_release(hold_emitted: bool) -> bool {
        !hold_emitted
    }
    /// Repeats fire at 1/5 of the initial hold threshold → faster feel.
    fn repeat_threshold_millis(t: u16) -> u16 {
        (t / 5).max(1)
    }
}

impl ButtonBehaviour for PushButton {
    fn emit_select_on_hold() -> bool {
        false
    }
    fn on_hold(_: u16) -> HoldAction {
        HoldAction::EmitAndWaitRelease
    }
    fn emit_select_on_release(_: bool) -> bool {
        true
    }
    fn repeat_threshold_millis(t: u16) -> u16 {
        t
    }
}

// ─── Button ───────────────────────────────────────────────────────────────────

pub struct Button<SW, BB>
where
    SW: InputPin,
    BB: ButtonBehaviour,
{
    /// Physical switch pin, active-low.
    sw_pin: SW,

    /// Integration counter used for debounce filtering.
    ///
    /// Range: `0..=button_debounce_tick`
    ///
    /// - `0` means confidently released
    /// - `button_debounce_tick` means confidently pressed
    debounce_counter_millis: u16,

    ///Refresh time of the button reading
    refresh_time_millis: u16,

    /// Current debounced logical button state.
    is_pressed: bool,

    /// Number of stable samples required to settle a pressed state.
    button_debounce_threshold_millis: u16,

    /// Number of update ticks required to classify the press as a hold.
    long_press_threshold_millis: u16,

    /// High-level press lifecycle state.
    state: ButtonState,

    /// Buffered event produced by `update()` and consumed by `poll()`.
    ///
    /// Only one event is stored at a time. If no event is pending,
    /// this field contains [`InputEvent::None`].
    pending_event: InputEvent,
    /// True once any hold has fired during this physical press.
    /// Cleared only on release.
    hold_consumed: bool,
    /// How many hold events have fired this press.
    /// 0 → use long_press_threshold; ≥1 → use repeat_threshold.
    hold_repeat_count: u16,
    _behaviour: PhantomData<BB>,
}

impl<SW, BB> Button<SW, BB>
where
    SW: InputPin,
    BB: ButtonBehaviour,
{
    pub fn new(
        sw_pin: SW,
        button_debounce_threshold_millis: u16,
        long_press_threshold_millis: u16,
        refresh_time_millis: u16,
    ) -> Self {
        Self {
            sw_pin,
            debounce_counter_millis: 0,
            is_pressed: false,
            button_debounce_threshold_millis,
            long_press_threshold_millis,
            refresh_time_millis,
            state: ButtonState::default(),
            pending_event: InputEvent::None,
            hold_consumed: false,
            hold_repeat_count: 0,
            _behaviour: PhantomData,
        }
    }

    pub fn update(&mut self) {
        let pin_is_low = self.sw_pin.is_low().unwrap_or(false);

        // ── Debounce integrator ───────────────────────────────────────────────
        if pin_is_low {
            self.debounce_counter_millis = self
                .debounce_counter_millis
                .saturating_add(self.refresh_time_millis)
                .min(self.button_debounce_threshold_millis);
        } else {
            self.debounce_counter_millis = self
                .debounce_counter_millis
                .saturating_sub(self.refresh_time_millis / 2);
        }

        if self.debounce_counter_millis >= self.button_debounce_threshold_millis {
            self.is_pressed = true;
        } else if self.debounce_counter_millis == 0 {
            self.is_pressed = false;
        }

        // ── State machine ─────────────────────────────────────────────────────
        self.state = match self.state {
            ButtonState::Idle => {
                if self.is_pressed {
                    ButtonState::Counting(0)
                } else {
                    ButtonState::Idle
                }
            }
            ButtonState::Counting(ticks) => {
                if ticks == 0 && BB::emit_select_on_hold() && !self.hold_consumed {
                    self.pending_event = InputEvent::Select;
                }
                if self.is_pressed {
                    // After first hold, switch to the shorter repeat threshold.
                    let threshold = if self.hold_repeat_count == 0 {
                        self.long_press_threshold_millis
                    } else {
                        BB::repeat_threshold_millis(self.long_press_threshold_millis)
                    };

                    if ticks >= threshold {
                        if self.pending_event == InputEvent::None {
                            self.pending_event = InputEvent::SelectHold;
                        }
                        // Compile-time dispatch — dead branch eliminated by compiler.
                        match BB::on_hold(self.hold_repeat_count) {
                            HoldAction::EmitAndWaitRelease => {
                                self.hold_consumed = true;
                                self.hold_repeat_count += 1;
                                ButtonState::WaitingRelease
                            }
                            HoldAction::EmitAndReset => {
                                self.hold_consumed = true;
                                self.hold_repeat_count += 1;
                                ButtonState::Counting(0) // reset for next repeat
                            }
                        }
                    } else {
                        ButtonState::Counting(ticks.saturating_add(1))
                    }
                } else {
                    // Released before hold threshold → short press.
                    if BB::emit_select_on_release(self.hold_consumed) && !BB::emit_select_on_hold() {
                        if self.pending_event == InputEvent::None {
                            self.pending_event = InputEvent::Select;
                        }
                    }
                    self.hold_consumed = false;
                    self.hold_repeat_count = 0;
                    ButtonState::Idle
                }
            }

            ButtonState::WaitingRelease => {
                if self.is_pressed {
                    ButtonState::WaitingRelease
                } else {
                    // Release after single hold.
                    // RotaryButton/PushButton: emit_select_on_release returns true
                    //   even when hold_consumed = true → emits Select.
                    // HoodButton: returns !hold_consumed = false → suppressed.
                    if BB::emit_select_on_release(self.hold_consumed) {
                        if self.pending_event == InputEvent::None {
                            self.pending_event = InputEvent::Select;
                        }
                    }
                    self.hold_consumed = false;
                    self.hold_repeat_count = 0;
                    ButtonState::Idle
                }
            }
        };
    }

    pub fn poll(&mut self) -> InputEvent {
        let event = self.pending_event;
        self.pending_event = InputEvent::None;
        event
    }

    pub fn is_pressed(&self) -> bool {
        self.is_pressed
    }
}

// ─── RotaryEncoder ────────────────────────────────────────────────────────────

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
    button: Button<SW, RotaryButton>,
    last_quad_state: u8,

    /// Accumulated quadrature delta since the last emitted rotation event.
    ///
    /// Positive values represent one direction, negative values represent
    /// the opposite direction.
    encoder_accumulator: i8,

    /// Remembers the last movement direction for fast-rotation logic.
    last_accumulator: RotaryAccumulatorMode,
    reset_counter: u8,
    rotary_reset_time_threshold_ms: u16,
    rotate_counter: u8,
    rotary_multi_step_threshold: u8,
}

impl<CLK, DT, SW> RotaryEncoder<CLK, DT, SW>
where
    CLK: InputPin,
    DT: InputPin,
    SW: InputPin,
{
    pub fn new(
        mut clk_pin: CLK,
        mut dt_pin: DT,
        sw_pin: SW,
        button_debounce_threshold_millis: u16,
        long_press_threshold_millis: u16,
        refresh_time_millis: u16,
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
                button_debounce_threshold_millis,
                long_press_threshold_millis,
                refresh_time_millis,
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

    pub fn update(&mut self) {
        self.button.update();

        #[rustfmt::skip]
        const QUAD_TABLE: [i8; 16] = [
             0, -1,  1,  0,
             1,  0,  0, -1,
            -1,  0,  0,  1,
             0,  1, -1,  0,
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
    fn poll(&mut self) -> InputEvent {
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
        }

        let button_event = self.button.poll();
        if button_event != InputEvent::None {
            self.last_accumulator = RotaryAccumulatorMode::None;
            self.rotate_counter = 0;
            self.reset_counter = 0;
            return button_event;
        }

        self.reset_counter = self.reset_counter.saturating_add(1);
        if self.reset_counter > self.rotary_reset_time_threshold_ms as u8 {
            self.reset_counter = 0;
            self.last_accumulator = RotaryAccumulatorMode::None;
            self.rotate_counter = 0;
        }

        InputEvent::None
    }
}
