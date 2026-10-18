//! The gravity a filter believes in when its config names none.
//!
//! Dynamics and measurement components both model gravity (the INS removes it
//! from specific force; an accelerometer model predicts it), so the default is
//! defined once for both.

/// Earth's standard gravity in world ENU `[east, north, up]` (m/s²), pointing
/// down.
pub(crate) const STANDARD_GRAVITY_ENU: [f64; 3] = [0.0, 0.0, -9.81];

/// Serde default for a `gravity_enu` key.
pub(crate) fn default_gravity_enu() -> [f64; 3] {
    STANDARD_GRAVITY_ENU
}
