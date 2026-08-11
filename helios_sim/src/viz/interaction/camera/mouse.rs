use crate::viz::interaction::camera::{CameraDriveIntent, TargetRequest};
use crate::viz::interaction::tuning::{require_positive, InteractionTuningError};

use bevy::input::gestures::PinchGesture;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::prelude::*;
use serde::Deserialize;

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct CameraMouseTuningFile {
    pub orbit_sensitivity_deg: Option<f32>,
    pub pan_sensitivity: Option<f32>,
    pub zoom_per_line: Option<f32>,
    pub zoom_per_pixel: Option<f32>,
    pub zoom_per_pinch: Option<f32>,
}

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

impl CameraMouseTuning {
    pub(crate) fn resolve(
        overrides: &CameraMouseTuningFile,
    ) -> Result<Self, InteractionTuningError> {
        let mut t = Self::default();
        if let Some(deg) = overrides.orbit_sensitivity_deg {
            t.orbit_sensitivity = deg.to_radians();
        }
        if let Some(v) = overrides.pan_sensitivity {
            t.pan_sensitivity = v;
        }
        if let Some(v) = overrides.zoom_per_line {
            t.zoom_per_line = v;
        }
        if let Some(v) = overrides.zoom_per_pixel {
            t.zoom_per_pixel = v;
        }
        if let Some(v) = overrides.zoom_per_pinch {
            t.zoom_per_pinch = v;
        }

        require_positive("camera.mouse.orbit_sensitivity", t.orbit_sensitivity)?;
        require_positive("camera.mouse.pan_sensitivity", t.pan_sensitivity)?;
        require_positive("camera.mouse.zoom_per_line", t.zoom_per_line)?;
        require_positive("camera.mouse.zoom_per_pixel", t.zoom_per_pixel)?;
        require_positive("camera.mouse.zoom_per_pinch", t.zoom_per_pinch)?;
        Ok(t)
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
        app.world_mut().write_message(MouseMotion {
            delta: Vec2::new(10.0, 0.0),
        });

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

    /// A file with no overrides resolves to exactly the compiled-in defaults.
    #[test]
    fn empty_file_resolves_to_defaults() {
        let t = CameraMouseTuning::resolve(&CameraMouseTuningFile::default()).unwrap();
        let d = CameraMouseTuning::default();
        assert_eq!(t.orbit_sensitivity, d.orbit_sensitivity);
        assert_eq!(t.pan_sensitivity, d.pan_sensitivity);
        assert_eq!(t.zoom_per_pinch, d.zoom_per_pinch);
    }

    /// The one angular gain is authored in degrees per pixel and stored in radians;
    /// the linear gains pass through unchanged.
    #[test]
    fn orbit_sensitivity_converts_from_degrees() {
        let file = CameraMouseTuningFile {
            orbit_sensitivity_deg: Some(90.0),
            zoom_per_line: Some(3.0),
            ..Default::default()
        };
        let t = CameraMouseTuning::resolve(&file).unwrap();
        assert!(
            (t.orbit_sensitivity - std::f32::consts::FRAC_PI_2).abs() < 1e-6,
            "got {}",
            t.orbit_sensitivity
        );
        assert_eq!(t.zoom_per_line, 3.0);
    }

    /// A non-positive gain is rejected rather than silently accepted.
    #[test]
    fn non_positive_gain_is_rejected() {
        let file = CameraMouseTuningFile {
            pan_sensitivity: Some(-1.0),
            ..Default::default()
        };
        let err = CameraMouseTuning::resolve(&file).unwrap_err();
        assert!(matches!(err, InteractionTuningError::NonPositive { .. }));
    }
}
