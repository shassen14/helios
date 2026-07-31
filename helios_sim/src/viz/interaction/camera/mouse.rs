use crate::viz::interaction::camera::{CameraDriveIntent, TargetRequest};

use bevy::input::gestures::PinchGesture;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::prelude::*;

/// Orbit gain: radians of yaw/pitch per pixel of drag.
const ORBIT_SENSITIVITY: f32 = 0.005;
/// Pan gain: ground meters per pixel of drag.
const PAN_SENSITIVITY: f32 = 0.05;
/// Zoom gain per wheel *notch* (Line unit).
const ZOOM_PER_LINE: f32 = 2.0;
/// Zoom gain per *pixel* of trackpad scroll (Pixel unit).
const ZOOM_PER_PIXEL: f32 = 0.05;
/// Zoom gain per unitless pinch delta.
const ZOOM_PER_PINCH: f32 = 40.0;
// TODO: pull all from the viz config surface once it exists.

pub(super) fn mouse_camera_intent(
    buttons: Res<ButtonInput<MouseButton>>,
    mut motion: MessageReader<MouseMotion>,
    mut wheel: MessageReader<MouseWheel>,
    mut pinch: MessageReader<PinchGesture>,
    mut intent: ResMut<CameraDriveIntent>,
) {
    let drag: Vec2 = motion.read().map(|e| e.delta).sum();

    if buttons.pressed(MouseButton::Left) {
        intent.yaw_delta += -drag.x * ORBIT_SENSITIVITY;
        intent.pitch_delta += drag.y * ORBIT_SENSITIVITY
    } else if buttons.pressed(MouseButton::Middle) {
        intent.pan_delta += Vec2::new(-drag.x, drag.y) * PAN_SENSITIVITY;
        intent.target_request = TargetRequest::ReleaseToFixed;
    }

    for e in wheel.read() {
        let step = match e.unit {
            MouseScrollUnit::Line => e.y * ZOOM_PER_LINE,
            MouseScrollUnit::Pixel => e.y * ZOOM_PER_PIXEL,
        };

        intent.zoom_delta -= step;
    }

    for e in pinch.read() {
        intent.zoom_delta += -e.0 * ZOOM_PER_PINCH;
    }
}
