//! The interactive camera plugin: spawns the `Camera3d` with its [`CameraRig`]
//! and drives the rig onto the camera's `Transform` every frame.
//!
//! Added only by the `helios_play` bin, alongside `VizPlugin` and
//! `ActionRegistryPlugin` — never by `HeliosSimulationPlugin`, whose body is
//! shared with the headless test bin. Owning the camera's *spawn* (not just its
//! motion) is deliberate: the module that controls a thing also creates it, so
//! the camera's birth and behavior have one home. The sun stays in `world/` — it
//! is scene; the camera stops being scene the moment it is driven.
//!
//! Motion flows in three stages: each input source (keyboard here, mouse in
//! `mouse`) reads its device and *accumulates* into a shared [`CameraDriveIntent`];
//! [`apply_camera_intent`](intent::apply_camera_intent) drains that intent onto the
//! rig once; [`sync_camera_transform`] writes the rig onto the `Transform`. The
//! intent buffer is why clamps and the focus-mode transition live in exactly one
//! place instead of being duplicated per input source.

use crate::{
    prelude::*,
    viz::interaction::{
        actions::{
            handle::{ActionHandle, ActionId, ActionMetadata, InputKind},
            registry::ActionRegistry,
        },
        camera::{
            follow::{resolve_focus, retarget_on_selection},
            intent::apply_camera_intent,
            keyboard::{keyboard_camera_intent, CameraKeyboardTuning},
            mouse::{mouse_camera_intent, CameraMouseTuning},
            rig::{rig_to_transform, CameraRig},
        },
        InteractionSet,
    },
};

pub mod follow;
pub mod intent;
pub mod keyboard;
pub mod mouse;
pub mod rig;

/// Schedule anchor for the camera's per-frame control systems. Ordered after
/// `InteractionSet::Sampling` so the systems here read the current frame's action
/// state rather than last frame's.
#[derive(SystemSet, Clone, PartialEq, Eq, Hash, Debug)]
pub enum CameraSet {
    Control,
}

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraDriveIntent>();
        app.init_resource::<CameraKeyboardTuning>();
        app.init_resource::<CameraMouseTuning>();

        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_camera.in_set(SceneBuildSet::ProcessWorldObjects),
        );

        app.add_systems(
            Startup,
            register_camera_actions.in_set(InteractionSet::Registration),
        );

        // The camera consumes the sampled action state, so it declares its own
        // dependency on the producer set — the input infra stays ignorant of it.
        app.configure_sets(Update, CameraSet::Control.after(InteractionSet::Sampling));

        // Input sources accumulate into the intent; `apply_camera_intent` is the
        // sole rig mutator; `sync_camera_transform` writes the rig onto the
        // `Transform`. Chained so each stage sees the previous stage's writes this
        // frame. Mouse input joins this chain before `apply_camera_intent`.
        app.add_systems(
            Update,
            (
                keyboard_camera_intent,
                mouse_camera_intent,
                retarget_on_selection,
                resolve_focus,
                apply_camera_intent,
                sync_camera_transform,
            )
                .chain()
                .in_set(CameraSet::Control),
        );
    }
}

/// Spawns the single scene camera with its rig, seeded to a fixed starting
/// vantage. The `Transform` is computed from the seed rig up front so frame zero
/// is already correct, before [`sync_camera_transform`] first runs. Starts in
/// [`CameraTarget::Fixed`]: its focus is a free world point until a follow
/// retargets it.
fn spawn_camera(mut commands: Commands) {
    // Seed vantage: reproduces the (-30, 25, 30) starting view.
    // TODO: pull from the viz config surface once it exists.
    let rig = CameraRig {
        focus: Vec3::ZERO,
        distance: 49.24,
        pitch: 0.532,
        yaw: -0.785,
    };

    let transform = rig_to_transform(&rig);

    commands.spawn((Camera3d::default(), rig, transform, CameraTarget::Fixed));
}

/// What feeds the rig's focus point — a *mode*, not a position.
///
/// The rig always orbits a plain `Vec3` ([`CameraRig::focus`]); this component
/// decides where that point comes from. `Fixed` leaves focus under manual (pan)
/// control; `Follow` tracks an entity whose position is copied into the focus by a
/// focus-resolution system before the orbit systems run. Orbit/zoom still apply in
/// `Follow` — they swing the camera *around* the followed focus, which is how you
/// change your POV while staying locked on a target. Only a pan, which moves the
/// focus *point*, conflicts with following. Kept off the rig so [`rig_to_transform`]
/// stays a pure fn that never reaches for an `Entity`.
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CameraTarget {
    /// Focus is a free world point, moved by pan gestures.
    #[default]
    Fixed,
    /// Focus tracks an entity; a pan gesture drops back to `Fixed`.
    Follow(Entity),
}

/// Per-frame accumulated request to move the camera, written by every input
/// source and drained once by [`apply_camera_intent`](intent::apply_camera_intent).
/// Device-agnostic: nothing here records whether a key, a drag, or a stick
/// produced it.
#[derive(Resource, Default)]
pub struct CameraDriveIntent {
    /// Orbit yaw delta, radians. Summed across sources this frame.
    yaw_delta: f32,
    /// Orbit pitch delta, radians.
    pitch_delta: f32,
    /// Orbit-distance delta, meters. Negative = zoom in.
    zoom_delta: f32,
    /// Focus shift in the **ground frame** (x = right, y = forward), meters —
    /// NOT screen pixels. Each source converts its device units to this before
    /// accumulating, so `apply` only has to orient by yaw.
    pan_delta: Vec2,
    /// What this frame's input asks of the focus *mode*.
    target_request: TargetRequest,
}

/// What this frame's input requests of the camera's focus *mode* — distinct
/// from the continuous deltas, which move the rig within whatever mode holds.
/// Defaults to `Keep`: most frames touch focus geometry, not the mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum TargetRequest {
    /// Leave [`CameraTarget`] untouched.
    #[default]
    Keep,
    /// A pan "grabbed the world" — drop `Follow` back to `Fixed`.
    ReleaseToFixed,
}

/// Cached [`ActionHandle`]s for the camera's motions, one field per action.
///
/// Populated once by [`register_camera_actions`] and read every frame by
/// [`keyboard_camera_intent`](keyboard::keyboard_camera_intent), so the drive
/// system resolves "is this motion active?" by handle instead of re-hashing an
/// [`ActionId`] string each frame — the whole point of the handle indirection.
///
/// Lives here, not in `keyboard`, because an action is device-neutral: its default
/// binding is a key, but a rebound gamepad button could trigger the same
/// `camera.orbit_left`. This is the camera's action vocabulary, not keyboard code.
#[derive(Resource)]
pub(crate) struct CameraActions {
    orbit_left: ActionHandle,
    orbit_right: ActionHandle,
    pitch_up: ActionHandle,
    pitch_down: ActionHandle,
    zoom_in: ActionHandle,
    zoom_out: ActionHandle,
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
        ActionId("camera.orbit_left"),
        ActionMetadata {
            label: "Orbit left",
            group: "camera",
            kind: InputKind::Axis,
            default_key: KeyCode::KeyA,
        },
    );

    let orbit_right = registry.register(
        ActionId("camera.orbit_right"),
        ActionMetadata {
            label: "Orbit right",
            group: "camera",
            kind: InputKind::Axis,
            default_key: KeyCode::KeyD,
        },
    );

    let pitch_up = registry.register(
        ActionId("camera.pitch_up"),
        ActionMetadata {
            label: "Pitch up",
            group: "camera",
            kind: InputKind::Axis,
            default_key: KeyCode::KeyW,
        },
    );

    let pitch_down = registry.register(
        ActionId("camera.pitch_down"),
        ActionMetadata {
            label: "Pitch down",
            group: "camera",
            kind: InputKind::Axis,
            default_key: KeyCode::KeyS,
        },
    );

    let zoom_in = registry.register(
        ActionId("camera.zoom_in"),
        ActionMetadata {
            label: "Zoom in",
            group: "camera",
            kind: InputKind::Axis,
            default_key: KeyCode::Equal,
        },
    );

    let zoom_out = registry.register(
        ActionId("camera.zoom_out"),
        ActionMetadata {
            label: "Zoom out",
            group: "camera",
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

/// Writes each rig's computed `Transform` onto its camera every frame, reflecting
/// whatever [`apply_camera_intent`](intent::apply_camera_intent) wrote to the rig
/// this frame. Ungated by `AppState` — the view is controllable whenever the
/// window is open.
fn sync_camera_transform(mut query: Query<(&CameraRig, &mut Transform)>) {
    for (rig, mut transform) in &mut query {
        *transform = rig_to_transform(rig);
    }
}

#[cfg(test)]
mod tests {
    use super::{register_camera_actions, CameraActions};

    use crate::viz::interaction::actions::{handle::ActionId, registry::ActionRegistry};

    use bevy::prelude::*;

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
            "camera.orbit_left",
            "camera.orbit_right",
            "camera.pitch_up",
            "camera.pitch_down",
            "camera.zoom_in",
            "camera.zoom_out",
        ] {
            assert!(
                registry.handle(ActionId(id)).is_some(),
                "register_camera_actions must declare {id} at Startup",
            );
        }

        assert!(
            app.world().get_resource::<CameraActions>().is_some(),
            "register_camera_actions must insert the CameraActions handle cache",
        );
    }
}
