use crate::viz::interaction::camera::{CameraDriveIntent, TargetRequest};

use bevy::input::gestures::PinchGesture;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::prelude::*;

/// Per-pixel / per-notch gains for mouse and trackpad camera control, read by
/// [`mouse_camera_intent`]. A `Resource`, not per-camera state: these are one
/// operator preference for the whole session. Defaults reproduce the values that
/// were compiled in before the tuning surface existed.
#[derive(Resource, Debug, Clone)]
pub struct CameraMouseTuning {
    /// Orbit gain: radians of yaw/pitch per pixel of drag.
    pub orbit_sensitivity: f32,
    /// Pan gain: ground meters per pixel of drag.
    pub pan_sensitivity: f32,
    /// Zoom gain per wheel *notch* (Line unit).
    pub zoom_per_line: f32,
    /// Zoom gain per *pixel* of trackpad scroll (Pixel unit).
    pub zoom_per_pixel: f32,
    /// Zoom gain per unitless pinch delta.
    pub zoom_per_pinch: f32,
}

impl Default for CameraMouseTuning {
    fn default() -> Self {
        Self {
            orbit_sensitivity: 0.005,
            pan_sensitivity: 0.05,
            zoom_per_line: 2.0,
            zoom_per_pixel: 0.05,
            zoom_per_pinch: 40.0,
        }
    }
}

pub(super) fn mouse_camera_intent(
    buttons: Res<ButtonInput<MouseButton>>,
    tuning: Res<CameraMouseTuning>,
    mut motion: MessageReader<MouseMotion>,
    mut wheel: MessageReader<MouseWheel>,
    mut pinch: MessageReader<PinchGesture>,
    mut intent: ResMut<CameraDriveIntent>,
) {
    let drag: Vec2 = motion.read().map(|e| e.delta).sum();

    if buttons.pressed(MouseButton::Left) {
        intent.yaw_delta += -drag.x * tuning.orbit_sensitivity;
        intent.pitch_delta += drag.y * tuning.orbit_sensitivity
    } else if buttons.pressed(MouseButton::Middle) {
        intent.pan_delta += Vec2::new(-drag.x, drag.y) * tuning.pan_sensitivity;
        intent.target_request = TargetRequest::ReleaseToFixed;
    }

    for e in wheel.read() {
        let step = match e.unit {
            MouseScrollUnit::Line => e.y * tuning.zoom_per_line,
            MouseScrollUnit::Pixel => e.y * tuning.zoom_per_pixel,
        };

        intent.zoom_delta -= step;
    }

    for e in pinch.read() {
        intent.zoom_delta += -e.0 * tuning.zoom_per_pinch;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A bare app holding `mouse_camera_intent`, the three message queues it reads
    /// (only some are fed per test, but all must exist or the reader panics), a
    /// mouse-button resource, and the given tuning. No `Time`: mouse deltas are
    /// per-event, not per-second.
    fn mouse_app(tuning: CameraMouseTuning) -> App {
        let mut app = App::new();
        app.add_message::<MouseMotion>()
            .add_message::<MouseWheel>()
            .add_message::<PinchGesture>()
            .insert_resource(ButtonInput::<MouseButton>::default())
            .insert_resource(tuning)
            .init_resource::<CameraDriveIntent>()
            .add_systems(Update, mouse_camera_intent);
        app
    }

    /// A left-drag orbits, and the yaw it produces is the drag scaled by
    /// `orbit_sensitivity` — proof the gain is read from the tuning, not a
    /// compiled-in constant. Reverting the field to a literal would compile clean
    /// and silently break config-drivability; this is the guard against that.
    #[test]
    fn left_drag_orbits_by_the_configured_sensitivity() {
        let mut app = mouse_app(CameraMouseTuning {
            orbit_sensitivity: 0.01,
            ..Default::default()
        });
        app.world_mut()
            .resource_mut::<ButtonInput<MouseButton>>()
            .press(MouseButton::Left);
        app.world_mut()
            .write_message(MouseMotion { delta: Vec2::new(10.0, 0.0) });

        app.update();

        // yaw_delta = -drag.x * orbit_sensitivity = -10 * 0.01
        let intent = app.world().resource::<CameraDriveIntent>();
        assert!(
            (intent.yaw_delta - -0.1).abs() < 1e-6,
            "got {}",
            intent.yaw_delta
        );
    }

    /// A pinch zooms by `zoom_per_pinch`; the same wiring guard on the gesture
    /// path. No button is held — a pinch stands alone.
    #[test]
    fn pinch_zooms_by_the_configured_gain() {
        let mut app = mouse_app(CameraMouseTuning {
            zoom_per_pinch: 30.0,
            ..Default::default()
        });
        app.world_mut().write_message(PinchGesture(0.5));

        app.update();

        // zoom_delta += -pinch * zoom_per_pinch = -0.5 * 30
        let intent = app.world().resource::<CameraDriveIntent>();
        assert!(
            (intent.zoom_delta - -15.0).abs() < 1e-6,
            "got {}",
            intent.zoom_delta
        );
    }
}
