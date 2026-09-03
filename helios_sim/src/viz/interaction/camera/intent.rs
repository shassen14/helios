use crate::viz::interaction::camera::{
    rig::{ground_pan_to_world, CameraRig, CameraRigTuning},
    CameraDriveIntent, CameraTarget, TargetRequest,
};

use bevy::prelude::*;

/// The camera's sole rig mutator: drains this frame's accumulated
/// [`CameraDriveIntent`] onto every [`CameraRig`] and its [`CameraTarget`], then
/// clears the intent for next frame.
///
/// Centralizing the write is the point of the intent buffer — the clamps, the
/// ground-plane pan projection, and the focus-mode transition all live here, once,
/// instead of being duplicated across every input source. `yaw` wraps freely;
/// `pitch` and `distance` pass through their clamps and apply in any focus mode
/// (they orbit *around* the focus); a pan shift is oriented by the current `yaw`
/// before it moves `focus`, and a pan while following drops back to `Fixed`.
pub(super) fn apply_camera_intent(
    tuning: Res<CameraRigTuning>,
    mut query: Query<(&mut CameraRig, &mut CameraTarget)>,
    mut intent: ResMut<CameraDriveIntent>,
) {
    // Nothing accumulated this frame: leave the rig untouched so it stays
    // un-`Changed`. The intent is already at rest, so there is nothing to reset.
    if intent.yaw_delta == 0.0
        && intent.pitch_delta == 0.0
        && intent.zoom_delta == 0.0
        && intent.pan_delta == Vec2::ZERO
        && intent.target_request == TargetRequest::Keep
    {
        return;
    }

    for (mut rig, mut target) in &mut query {
        // Orient the pan by the heading held during the drag, before yaw changes.
        let pan = ground_pan_to_world(intent.pan_delta, rig.yaw);

        rig.yaw += intent.yaw_delta;
        rig.pitch = tuning.clamp_pitch(rig.pitch + intent.pitch_delta);
        rig.distance = tuning.clamp_distance(rig.distance + intent.zoom_delta);
        rig.focus += pan;

        if intent.target_request == TargetRequest::ReleaseToFixed {
            *target = CameraTarget::Fixed;
        }
    }

    *intent = CameraDriveIntent::default();
}
