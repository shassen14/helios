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
    viz::interaction::camera::rig::{rig_to_transform, CameraRig},
};

pub mod rig;

/// Schedule anchor for the camera's per-frame systems. One variant today; later
/// commits read `ActionState` here and order this set after input sampling.
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

        app.add_systems(Update, sync_camera_transform.in_set(CameraSet::Control));
    }
}

/// Spawns the single scene camera with its rig, seeded to the vantage the scene
/// opened on before the camera became rig-driven. The `Transform` is computed
/// from the seed rig up front so frame zero is already correct, before
/// [`sync_camera_transform`] first runs.
fn spawn_camera(mut commands: Commands) {
    // Seed vantage reproduces the old fixed camera position (-30, 25, 30).
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

/// Writes each rig's computed `Transform` onto its camera every frame. Ungated
/// by `AppState` — the view is controllable whenever the window is open. In this
/// commit the rig never changes, so this restates the same transform; later
/// commits mutate the rig from input first and this reflects it.
fn sync_camera_transform(mut query: Query<(&CameraRig, &mut Transform)>) {
    for (rig, mut transform) in &mut query {
        *transform = rig_to_transform(rig);
    }
}
