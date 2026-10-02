//! Interactive teleop: reads the bound drive keys and publishes a device- and
//! morphology-agnostic [`TwistIntent`](helios_core::control::commands::TwistIntent)
//! onto the piloted agent's pipeline bus. The runtime's `TwistTeleopNode` scales
//! that intent into a body command — the host does device I/O only and knows
//! nothing about which axes a body drives.
//!
//! Added by the `helios_play` bin, never `HeliosSimulationPlugin`: the headless
//! test bin shares that body and must stay input-free, the same rule the camera
//! follows.
//!
//! The mapping runs in one stage — [`publish_teleop_intent`] folds the held keys
//! into a [`TwistIntent`] and publishes it. Unlike the camera, there is no shared
//! intent buffer: the value crosses the host↔brain boundary immediately, and the
//! per-DOF tuning and axis selection live downstream in the runtime node.

pub mod actions;
pub mod intent;

use crate::prelude::AppState;
use crate::viz::interaction::{
    teleop::{actions::register_teleop_actions, intent::publish_teleop_intent},
    InteractionSet,
};

use bevy::prelude::*;

/// Schedule anchor for the teleop publish, ordered after `InteractionSet::Sampling`
/// so it reads this frame's action state rather than last frame's.
#[derive(SystemSet, Clone, PartialEq, Eq, Hash, Debug)]
pub enum TeleopSet {
    Publish,
}

pub struct TeleopPlugin;

impl Plugin for TeleopPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Startup,
            register_teleop_actions.in_set(InteractionSet::Registration),
        );

        // Teleop consumes the sampled action state, so it declares its own
        // dependency on the producer set — the input infra stays ignorant of it,
        // matching how the camera and viz layers order themselves.
        app.configure_sets(Update, TeleopSet::Publish.after(InteractionSet::Sampling));

        app.add_systems(
            Update,
            publish_teleop_intent
                .in_set(TeleopSet::Publish)
                .run_if(in_state(AppState::Running)),
        );
    }
}
