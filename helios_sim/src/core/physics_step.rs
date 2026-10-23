//! Where physics runs: inside each fixed step, in `FixedUpdate`, so the
//! `SimulationSet` chain is the order that really runs, and every value read
//! after physics in step `k` describes the instant [`RunClock`] gives it.
//!
//! [`RunClock`]: crate::core::run_clock::RunClock

use avian3d::prelude::PhysicsPlugins;
use bevy::prelude::*;
use bevy::transform::systems::{
    mark_dirty_trees, propagate_parent_transforms, sync_simple_transforms,
};
use bevy::transform::TransformSystems;

/// Avian, stepped once per fixed step, and `GlobalTransform` propagated from
/// what it wrote back.
///
/// The host and every test that steps physics add this, not
/// `PhysicsPlugins::default()`, which runs in `FixedPostUpdate`, after every
/// sim set.
///
/// Avian's writeback sets `Transform` only, and Bevy propagates once a
/// frame, so without the second pass every step of a batch but its first
/// would read a `GlobalTransform` a step behind `Position`. Avian's own pass
/// before each step stays on: it catches transforms written outside physics
/// (spawns, `Update` writes), and a pass over clean trees is cheap.
pub struct PhysicsStepPlugin;

impl Plugin for PhysicsStepPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(PhysicsPlugins::new(FixedUpdate));

        // Bevy's own systems in Bevy's own set, as Avian chains them, so they
        // take no `SimulationSet`: the set chain places
        // `TransformSystems::Propagate` between physics and `StateSync`.
        app.add_systems(
            FixedUpdate,
            (
                mark_dirty_trees,
                propagate_parent_transforms,
                sync_simple_transforms,
            )
                .chain()
                .in_set(TransformSystems::Propagate),
        );
    }
}

#[cfg(test)]
#[path = "physics_step_tests.rs"]
mod tests;
