use crate::viz::interaction::{
    camera::{CameraActions, CameraDriveIntent},
    sampling::ActionState,
    tuning::{require_positive, InteractionTuningError},
};

use bevy::prelude::*;
use serde::Deserialize;

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct CameraKeyboardTuningFile {
    pub orbit_rate_deg: Option<f32>,
    pub pitch_rate_deg: Option<f32>,
    pub zoom_rate: Option<f32>,
}

/// Per-second gains for keyboard camera control, read by
/// [`keyboard_camera_intent`]. A `Resource`, not per-camera state: these are one
/// operator preference for the whole session. Defaults reproduce the values that
/// were compiled in before the tuning surface existed.
#[derive(Resource, Debug, Clone)]
pub struct CameraKeyboardTuning {
    /// Yaw sweep speed while an orbit key is held, in radians per second.
    pub orbit_rate: f32,
    /// Pitch speed while a pitch key is held, in radians per second.
    pub pitch_rate: f32,
    /// Orbit-distance change speed while a zoom key is held, in meters per second.
    pub zoom_rate: f32,
}

impl Default for CameraKeyboardTuning {
    fn default() -> Self {
        Self {
            orbit_rate: 1.0,
            pitch_rate: 1.0,
            zoom_rate: 25.0,
        }
    }
}

impl CameraKeyboardTuning {
    pub(crate) fn resolve(
        overrides: &CameraKeyboardTuningFile,
    ) -> Result<Self, InteractionTuningError> {
        let mut t: CameraKeyboardTuning = Self::default();

        if let Some(deg) = overrides.orbit_rate_deg {
            t.orbit_rate = deg.to_radians();
        }
        if let Some(deg) = overrides.pitch_rate_deg {
            t.pitch_rate = deg.to_radians();
        }
        if let Some(v) = overrides.zoom_rate {
            t.zoom_rate = v;
        }

        require_positive("camera.keyboard.orbit_rate", t.orbit_rate)?;
        require_positive("camera.keyboard.pitch_rate", t.pitch_rate)?;
        require_positive("camera.keyboard.zoom_rate", t.zoom_rate)?;

        Ok(t)
    }
}

/// Translate the held camera keys into this frame's [`CameraDriveIntent`]. Each
/// opposing pair (`right`−`left`, `up`−`down`, `out`−`in`) collapses to a signed
/// direction, scaled by its per-second rate and `dt` so motion is frame-rate
/// independent, then *added* to the intent. Writes no rig state — `apply_camera_intent`
/// owns the clamps and the actual mutation.
pub(super) fn keyboard_camera_intent(
    time: Res<Time>,
    state: Res<ActionState>,
    actions: Res<CameraActions>,
    tuning: Res<CameraKeyboardTuning>,
    mut intent: ResMut<CameraDriveIntent>,
) {
    let dt = time.delta_secs();

    let yaw_input = f32::from(state.is_active(actions.orbit_right))
        - f32::from(state.is_active(actions.orbit_left));
    let pitch_input = f32::from(state.is_active(actions.pitch_up))
        - f32::from(state.is_active(actions.pitch_down));
    // Zoom *in* shortens the orbit distance, so it is the negative direction.
    let zoom_input =
        f32::from(state.is_active(actions.zoom_out)) - f32::from(state.is_active(actions.zoom_in));

    // Nothing held this frame: contribute nothing to the intent.
    if yaw_input == 0.0 && pitch_input == 0.0 && zoom_input == 0.0 {
        return;
    }

    intent.yaw_delta += yaw_input * tuning.orbit_rate * dt;
    intent.pitch_delta += pitch_input * tuning.pitch_rate * dt;
    intent.zoom_delta += zoom_input * tuning.zoom_rate * dt;
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f32::consts::{FRAC_PI_2, PI};

    /// The sparse-overlay contract: a file that names nothing resolves to exactly
    /// the compiled-in defaults, so an absent field can never move a value.
    #[test]
    fn empty_file_resolves_to_defaults() {
        let t = CameraKeyboardTuning::resolve(&CameraKeyboardTuningFile::default()).unwrap();
        let d = CameraKeyboardTuning::default();
        assert_eq!(t.orbit_rate, d.orbit_rate);
        assert_eq!(t.pitch_rate, d.pitch_rate);
        assert_eq!(t.zoom_rate, d.zoom_rate);
    }

    /// The angular rates are authored in degrees and stored in radians; the
    /// un-overridden linear rate stays put.
    #[test]
    fn angular_rates_convert_from_degrees() {
        let file = CameraKeyboardTuningFile {
            orbit_rate_deg: Some(90.0),
            pitch_rate_deg: Some(180.0),
            ..Default::default()
        };
        let t = CameraKeyboardTuning::resolve(&file).unwrap();
        assert!(
            (t.orbit_rate - FRAC_PI_2).abs() < 1e-6,
            "got {}",
            t.orbit_rate
        );
        assert!((t.pitch_rate - PI).abs() < 1e-6, "got {}", t.pitch_rate);
        assert_eq!(t.zoom_rate, CameraKeyboardTuning::default().zoom_rate);
    }

    /// A linear rate (m/s) passes through untouched — no conversion applied.
    #[test]
    fn zoom_rate_passes_through() {
        let file = CameraKeyboardTuningFile {
            zoom_rate: Some(50.0),
            ..Default::default()
        };
        let t = CameraKeyboardTuning::resolve(&file).unwrap();
        assert_eq!(t.zoom_rate, 50.0);
    }

    /// A non-positive rate is a startup misconfiguration, surfaced as an error the
    /// loader turns into a panic — never silently accepted.
    #[test]
    fn non_positive_rate_is_rejected() {
        let file = CameraKeyboardTuningFile {
            zoom_rate: Some(0.0),
            ..Default::default()
        };
        let err = CameraKeyboardTuning::resolve(&file).unwrap_err();
        assert!(matches!(err, InteractionTuningError::NonPositive { .. }));
    }
}
