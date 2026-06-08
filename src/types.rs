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
