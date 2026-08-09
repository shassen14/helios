use crate::viz::interaction::{
    camera::{CameraActions, CameraDriveIntent},
    sampling::ActionState,
};

use bevy::prelude::*;

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
