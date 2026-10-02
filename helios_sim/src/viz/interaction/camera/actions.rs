//! The camera's action vocabulary: one id per motion, the registration that
//! declares them, and the handle cache the drive systems read.
//!
//! An action is device-neutral: its default binding is a key, but a rebound
//! gamepad button could trigger the same `camera.orbit_left`. So this lives
//! beside the camera's other modules rather than inside `keyboard`.

use crate::viz::interaction::actions::{
    handle::{ActionHandle, ActionId, ActionMetadata, InputKind},
    registry::ActionRegistry,
};

use bevy::prelude::*;

/// Camera action ids, one per motion.
pub const ORBIT_LEFT: ActionId = ActionId("camera.orbit_left");
pub const ORBIT_RIGHT: ActionId = ActionId("camera.orbit_right");
pub const PITCH_UP: ActionId = ActionId("camera.pitch_up");
pub const PITCH_DOWN: ActionId = ActionId("camera.pitch_down");
pub const ZOOM_IN: ActionId = ActionId("camera.zoom_in");
pub const ZOOM_OUT: ActionId = ActionId("camera.zoom_out");

/// The group the camera actions are listed under.
pub const CAMERA_GROUP: &str = "camera";

/// Cached [`ActionHandle`]s for the camera's motions, one field per action.
///
/// Populated once by [`register_camera_actions`] and read every frame by
/// [`keyboard_camera_intent`](super::keyboard::keyboard_camera_intent), so the
/// drive system resolves "is this motion active?" by handle instead of
/// re-hashing an [`ActionId`] string each frame — the whole point of the handle
/// indirection.
#[derive(Resource)]
pub(crate) struct CameraActions {
    pub(super) orbit_left: ActionHandle,
    pub(super) orbit_right: ActionHandle,
    pub(super) pitch_up: ActionHandle,
    pub(super) pitch_down: ActionHandle,
    pub(super) zoom_in: ActionHandle,
    pub(super) zoom_out: ActionHandle,
}

/// Register the camera's six motions into the shared [`ActionRegistry`] and cache
/// their handles in [`CameraActions`].
///
/// Runs once at `Startup` in `InteractionSet::Registration`, a sibling of
/// `register_viz_actions` — the camera brings its own registration rather than
/// extending the viz one, so no single file lists every action. Each motion is an
/// [`InputKind::Axis`]: sampled while its key is *held*, which is what continuous
/// orbit/zoom needs.
pub(crate) fn register_camera_actions(
    mut registry: ResMut<ActionRegistry>,
    mut commands: Commands,
) {
    let orbit_left = registry.register(
        ORBIT_LEFT,
        ActionMetadata {
            label: "Orbit left",
            group: CAMERA_GROUP,
            kind: InputKind::Axis,
            default_key: KeyCode::KeyA,
        },
    );

    let orbit_right = registry.register(
        ORBIT_RIGHT,
        ActionMetadata {
            label: "Orbit right",
            group: CAMERA_GROUP,
            kind: InputKind::Axis,
            default_key: KeyCode::KeyD,
        },
    );

    let pitch_up = registry.register(
        PITCH_UP,
        ActionMetadata {
            label: "Pitch up",
            group: CAMERA_GROUP,
            kind: InputKind::Axis,
            default_key: KeyCode::KeyW,
        },
    );

    let pitch_down = registry.register(
        PITCH_DOWN,
        ActionMetadata {
            label: "Pitch down",
            group: CAMERA_GROUP,
            kind: InputKind::Axis,
            default_key: KeyCode::KeyS,
        },
    );

    let zoom_in = registry.register(
        ZOOM_IN,
        ActionMetadata {
            label: "Zoom in",
            group: CAMERA_GROUP,
            kind: InputKind::Axis,
            default_key: KeyCode::Equal,
        },
    );

    let zoom_out = registry.register(
        ZOOM_OUT,
        ActionMetadata {
            label: "Zoom out",
            group: CAMERA_GROUP,
            kind: InputKind::Axis,
            default_key: KeyCode::Minus,
        },
    );

    commands.insert_resource(CameraActions {
        orbit_left,
        orbit_right,
        pitch_up,
        pitch_down,
        zoom_in,
        zoom_out,
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tier-3 wiring guard: the failure this catches is someone dropping the
    /// `add_systems(Startup, register_camera_actions…)` line or the resource
    /// insert — the code still compiles, but no camera action is declared and
    /// `keyboard_camera_intent` finds no `CameraActions`. Stands up only the
    /// registry and this one system, deliberately *not* the whole `CameraPlugin`,
    /// which would drag in the sampler, keybinding loader, and a window — none of
    /// them the thing under test.
    #[test]
    fn register_camera_actions_declares_all_motions_at_startup() {
        let mut app = App::new();
        app.init_resource::<ActionRegistry>();
        app.add_systems(Startup, register_camera_actions);

        // The first `update()` runs the `Startup` schedule exactly once.
        app.update();

        let registry = app.world().resource::<ActionRegistry>();
        for id in [
            ORBIT_LEFT,
            ORBIT_RIGHT,
            PITCH_UP,
            PITCH_DOWN,
            ZOOM_IN,
            ZOOM_OUT,
        ] {
            assert!(
                registry.handle(id).is_some(),
                "register_camera_actions must declare {} at Startup",
                id.0,
            );
        }

        assert!(
            app.world().get_resource::<CameraActions>().is_some(),
            "register_camera_actions must insert the CameraActions handle cache",
        );
    }
}
