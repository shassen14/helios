//! The host↔brain boundary: glue between the Bevy simulation and each agent's
//! `AutonomyPipeline`.
//!
//! The pipeline is a black box with three openings, and every module here
//! serves exactly one of them:
//!
//! - **ingress** — host state and intent flow *in*. [`spawn`] assembles the
//!   pipeline and its odom/control components at scene-build; [`goal_input`]
//!   feeds the mission goal each tick.
//! - **tick** — [`tick`] advances every pipeline one step per `FixedUpdate`.
//! - **egress** — pipeline results flow *out*. [`control_output`] copies the
//!   latest control into the actuation component; [`odom_output`] writes the
//!   pose estimate onto the odom frame.
//!
//! Each runtime module is named for the `SimulationSet` it runs in, so this
//! module list mirrors the schedule. [`BrainBridgePlugin`] registers them all.

pub mod components;
pub mod control_output;
pub mod goal_input;
pub mod odom_output;
pub mod sensor_publisher;
pub mod spawn;
pub mod tick;

pub use components::{
    AgentIdComponent, AutonomyPipelineComponent, OdomFrameOf, SensorPublishChannel,
};
pub use control_output::publish_pipeline_control;
pub use goal_input::{dispatch_configured_goals, forward_goal_events};
pub use odom_output::update_odom_frames;
pub use sensor_publisher::SensorPublisher;
pub use spawn::{spawn_autonomy_pipeline, spawn_control_output, spawn_odom_frames};
pub use tick::run_pipeline_tick;

use crate::core::events::GoalCommandEvent;

use crate::prelude::*;

/// Registers every host↔pipeline system into its set.
///
/// This plugin only assigns systems to sets; it does not order them. Execution
/// order comes from the `SimulationSet` / `SceneBuildSet` graph configured once
/// in `SimulationSetupPlugin`. Registration order here is irrelevant.
pub struct BrainBridgePlugin;

impl Plugin for BrainBridgePlugin {
    fn build(&self, app: &mut App) {
        // Scene-build spawns. `SpawnPipeline` runs first (assembly reads the
        // sensors' channels), then odom + control bind to the fresh pipeline.
        // Odom and control are independent, so they share the `BindPipeline` pass.
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            (
                spawn_autonomy_pipeline.in_set(SceneBuildSet::SpawnPipeline),
                spawn_odom_frames.in_set(SceneBuildSet::BindPipeline),
                spawn_control_output.in_set(SceneBuildSet::BindPipeline),
            ),
        );

        // The whole-brain tick, then the odom write. Chained: `update_odom_frames`
        // must see the pose `run_pipeline_tick` just produced (see odom_output).
        app.add_systems(
            FixedUpdate,
            (run_pipeline_tick, update_odom_frames)
                .chain()
                .in_set(SimulationSet::BrainTick)
                .run_if(in_state(AppState::Running)),
        );

        // Goal ingress. `add_message` is the *sole* registration of
        // `GoalCommandEvent` now that the old PlanningPlugin is gone — dropping
        // it breaks goal forwarding silently, so a test guards it below. Chained:
        // dispatch writes an event that forward reads the same tick, and a Bevy
        // message is only readable this tick if its writer ran first.
        app.add_message::<GoalCommandEvent>().add_systems(
            FixedUpdate,
            (dispatch_configured_goals, forward_goal_events)
                .chain()
                .in_set(SimulationSet::BrainInput)
                .run_if(in_state(AppState::Running)),
        );

        // Control egress.
        app.add_systems(
            FixedUpdate,
            publish_pipeline_control
                .in_set(SimulationSet::BrainOutput)
                .run_if(in_state(AppState::Running)),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Guards the `add_message::<GoalCommandEvent>()` call in `build`: it is the
    /// only registration of that message, and losing it during a refactor would
    /// break goal forwarding at runtime while still compiling.
    #[test]
    fn plugin_registers_goal_command_event() {
        let mut app = App::new();
        app.add_plugins(BrainBridgePlugin);

        assert!(
            app.world()
                .get_resource::<Messages<GoalCommandEvent>>()
                .is_some(),
            "BrainBridgePlugin must register GoalCommandEvent via add_message"
        );
    }
}
