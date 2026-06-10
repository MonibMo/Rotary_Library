#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputEvent {
    /// No action occurred in this polling period.
    None,
    /// Upwards navigation or scroll left.
    Up,
    /// Upwards with multiplication of MULTI_STEP_TEMP const
    FastUp,
    /// Downwards navigation or scroll right.
    Down,
    /// Downwards with multiplication of MULTI_STEP_TEMP const
    FastDown,
    /// Button click or confirmation.
    Select,
    /// Button held down (typically for exits/saves).
    SelectHold,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RotaryAccumulatorMode {
    /// Positive
    Positive,
    /// Negative
    Negative,
    /// None
    None,
}

/// Abstract representation of an input driver.
///
/// Implementors are responsible for polling hardware signals (e.g., rotary encoder pins),
/// handling debounce filtering, and returning discrete [`InputEvent`]s.
pub trait InputSource {
    /// Polls the physical interface and returns a debounced input event.
    fn poll(&mut self) -> InputEvent;
}

#[derive(Clone, Copy, Default)]
pub enum ButtonDefaultState{
    #[default]
    PulledUp,
    PulledDown,
}

/// States of the button press lifecycle.
///
/// The machine advances strictly forward per press cycle:
/// `Idle → Counting → WaitingRelease → Idle`
/// or for a short press:
/// `Idle → Counting(u16::MAX sentinel) → Idle`
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum ButtonState {
    /// Button is not pressed. Waiting for the next press.
    #[default]
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
