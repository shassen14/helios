use serde::Deserialize;

/// The scene camera's starting vantage, authored in the scenario file's
/// `[camera]` block. A scenario-level initial condition (a spawn-seed), not an
/// operator preference: it seeds the camera entity's rig once and then stops
/// being config. Omitting `[camera]` yields the [`Default`] framing.
///
/// Pure scenario data — angles in degrees, no runtime types. The conversion to
/// the radian `CameraRig` (and its validation) lives with `CameraRig` in
/// `viz::interaction::camera::rig`, so `config` never depends on `viz`.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraVantage {
    /// Orbit distance from the focus point, meters.
    pub distance: f32,
    /// Elevation above the horizontal plane, degrees. Must be in (0, 90).
    pub pitch_deg: f32,
    /// Rotation about the vertical axis, degrees.
    pub yaw_deg: f32,
}

impl Default for CameraVantage {
    fn default() -> Self {
        // Reproduces the starting view that was compiled into `spawn_camera`
        // before this surface existed: ~45° behind, ~30° up, 49.24 m back,
        // orbiting the world origin.
        Self {
            distance: 49.24,
            pitch_deg: 30.5,
            yaw_deg: -45.0,
        }
    }
}
