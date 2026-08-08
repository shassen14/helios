use serde::Deserialize;

/// How operator intent is mapped into a command. One variant per command
/// family: the variant fixes both the intent type the host publishes and the
/// command type the mapper produces, and carries that family's per-DOF tuning.
///
/// Single-variant today, an enum on purpose — a second family (e.g. `Surface`
/// for a fixed-wing, mapping surface + throttle) adds an arm rather than
/// reshaping this one, and the assembler's match over it becomes a compile error
/// until the new arm is written. Mirrors [`ControllerConfig`] and
/// [`EstimatorConfig`].
///
/// [`ControllerConfig`]: crate::config::ControllerConfig
/// [`EstimatorConfig`]: crate::config::EstimatorConfig
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields, tag = "kind")]
pub enum TeleopMapperConfig {
    /// `TwistIntent` → `BodyTwist`. Per-DOF scale magnitudes: translation
    /// (`surge`/`sway`/`heave`) in m/s, rotation (`roll`/`pitch`/`yaw`) in rad/s,
    /// applied to the matching normalised deflection. An axis omitted from TOML
    /// scales at `0.0` — a true statement that the body cannot drive it (a car
    /// names only `surge` and `yaw`), not a dead field.
    Twist {
        #[serde(default)]
        surge: f64,
        #[serde(default)]
        sway: f64,
        #[serde(default)]
        heave: f64,
        #[serde(default)]
        roll: f64,
        #[serde(default)]
        pitch: f64,
        #[serde(default)]
        yaw: f64,
    },
}
