//! The interactive camera plugin: spawns the `Camera3d` with its [`CameraRig`]
//! and drives the rig onto the camera's `Transform` every frame.
//!
//! Added only by the `helios_play` bin, alongside `VizPlugin` and
//! `ActionRegistryPlugin` — never by `HeliosSimulationPlugin`, whose body is
//! shared with the headless test bin. Owning the camera's *spawn* (not just its
//! motion) is deliberate: the module that controls a thing also creates it, so
//! the camera's birth and behavior have one home. The sun stays in `world/` — it
//! is scene; the camera stops being scene the moment it is driven.

use crate::{
    prelude::*,
    viz::interaction::{
        actions::{
            handle::{ActionHandle, ActionId, ActionMetadata, InputKind},
            registry::ActionRegistry,
        },
        camera::rig::{clamp_distance, clamp_pitch, rig_to_transform, CameraRig},
        sampling::ActionState,
        InteractionSet,
    },
};

pub mod rig;

/// Schedule anchor for the camera's per-frame control systems. Ordered after
/// `InteractionSet::Sampling` so the systems here read the current frame's
/// [`ActionState`] rather than last frame's.
#[derive(SystemSet, Clone, PartialEq, Eq, Hash, Debug)]
pub enum CameraSet {
    Control,
}

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_camera.in_set(SceneBuildSet::ProcessWorldObjects),
        );

        app.add_systems(
            Startup,
            register_camera_actions.in_set(InteractionSet::Registration),
        );

        // The camera consumes `ActionState`, so it declares its own dependency on
        // the producer set — the input infra stays ignorant of the camera.
        app.configure_sets(Update, CameraSet::Control.after(InteractionSet::Sampling));

        // `orbit_camera` mutates the rig; `sync_camera_transform` reads it onto the
        // `Transform`. Chain so the write is visible to the read this frame.
        app.add_systems(
            Update,
            (orbit_camera, sync_camera_transform)
                .chain()
                .in_set(CameraSet::Control),
        );
    }
}

/// Spawns the single scene camera with its rig, seeded to a fixed starting
/// vantage. The `Transform` is computed from the seed rig up front so frame zero
/// is already correct, before [`sync_camera_transform`] first runs.
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

    commands.spawn((Camera3d::default(), rig, transform));
}

/// Cached [`ActionHandle`]s for the camera's motions, one field per action.
///
/// Populated once by [`register_camera_actions`] and read every frame by
/// [`orbit_camera`], so the drive system resolves "is this motion active?" by
/// handle instead of re-hashing an [`ActionId`] string each frame — the whole
/// point of the handle indirection.
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
pub(crate) fn register_camera_actions(mut registry: ResMut<ActionRegistry>, mut commands: Commands) {
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

/// Yaw sweep speed while an orbit key is held, in radians per second.
// TODO: pull from the viz config surface once it exists.
const ORBIT_RATE: f32 = 1.0;

/// Pitch speed while a pitch key is held, in radians per second.
// TODO: pull from the viz config surface once it exists.
const PITCH_RATE: f32 = 1.0;

/// Orbit-distance change speed while a zoom key is held, in meters per second.
// TODO: pull from the viz config surface once it exists.
const ZOOM_RATE: f32 = 25.0;

/// Drive each rig from the actions active this frame: held orbit/pitch keys sweep
/// `yaw`/`pitch`, held zoom keys change `distance`. Each opposing pair collapses to
/// a signed direction (`right − left`), scaled by its rate and `dt` so motion is
/// frame-rate independent. `pitch` and `distance` are written through their clamps;
/// `yaw` wraps freely.
fn orbit_camera(
    time: Res<Time>,
    state: Res<ActionState>,
    actions: Res<CameraActions>,
    mut rigs: Query<&mut CameraRig>,
) {
    let dt = time.delta_secs();

    let yaw_input = f32::from(state.is_active(actions.orbit_right))
        - f32::from(state.is_active(actions.orbit_left));
    let pitch_input = f32::from(state.is_active(actions.pitch_up))
        - f32::from(state.is_active(actions.pitch_down));
    // Zoom *in* shortens the orbit distance, so it is the negative direction.
    let zoom_input = f32::from(state.is_active(actions.zoom_out))
        - f32::from(state.is_active(actions.zoom_in));

    // Nothing held: skip the loop so the rig isn't marked changed every idle frame.
    if yaw_input == 0.0 && pitch_input == 0.0 && zoom_input == 0.0 {
        return;
    }

    for mut rig in &mut rigs {
        rig.yaw += yaw_input * ORBIT_RATE * dt;
        rig.pitch = clamp_pitch(rig.pitch + pitch_input * PITCH_RATE * dt);
        rig.distance = clamp_distance(rig.distance + zoom_input * ZOOM_RATE * dt);
    }
}

/// Writes each rig's computed `Transform` onto its camera every frame, reflecting
/// whatever [`orbit_camera`] wrote to the rig this frame. Ungated by `AppState` —
/// the view is controllable whenever the window is open.
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
    /// `orbit_camera` finds no `CameraActions`. Stands up only the registry and
    /// this one system, deliberately *not* the whole `CameraPlugin`, which would
    /// drag in the sampler, keybinding loader, and a window — none of them the
    /// thing under test.
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
