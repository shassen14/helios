use crate::viz::interaction::{
    camera::{CameraActions, CameraDriveIntent},
    sampling::ActionState,
};

use bevy::prelude::*;

/// Yaw sweep speed while an orbit key is held, in radians per second.
// TODO: pull from the viz config surface once it exists.
const ORBIT_RATE: f32 = 1.0;

/// Pitch speed while a pitch key is held, in radians per second.
// TODO: pull from the viz config surface once it exists.
const PITCH_RATE: f32 = 1.0;

/// Orbit-distance change speed while a zoom key is held, in meters per second.
// TODO: pull from the viz config surface once it exists.
const ZOOM_RATE: f32 = 25.0;

/// Translate the held camera keys into this frame's [`CameraDriveIntent`]. Each
/// opposing pair (`right`−`left`, `up`−`down`, `out`−`in`) collapses to a signed
/// direction, scaled by its per-second rate and `dt` so motion is frame-rate
/// independent, then *added* to the intent. Writes no rig state — `apply_camera_intent`
/// owns the clamps and the actual mutation.
pub(super) fn keyboard_camera_intent(
    time: Res<Time>,
    state: Res<ActionState>,
    actions: Res<CameraActions>,
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

    intent.yaw_delta += yaw_input * ORBIT_RATE * dt;
    intent.pitch_delta += pitch_input * PITCH_RATE * dt;
    intent.zoom_delta += zoom_input * ZOOM_RATE * dt;
}
