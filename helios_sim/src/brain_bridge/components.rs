use helios_core::data::messages::TrajectoryPoint;
use helios_runtime::pipeline::AutonomyPipeline;

use bevy::prelude::{Component, Entity};

/// Holds the fully-assembled autonomy pipeline for an agent.
/// Written at spawn time by `spawn_autonomy_pipeline`. Ticked every frame
/// by `run_pipeline_tick` in `SimulationSet::BrainTick`.
#[derive(Component)]
pub struct AutonomyPipelineComponent(pub AutonomyPipeline);

/// The agent's stable, human-meaningful identity — the bare name from `agent_config.name()` (e.g. `"rover_1"`), *not* the composite `Name`
/// (`"rover_1/base_link"`). Written at spawn time by `spawn_autonomy_pipeline`
/// onto every agent it processes, in the same insert as the outcome of the
/// build — [`AutonomyPipelineComponent`] on success, [`PipelineBuildFailed`]
/// on failure. So a query for `(&AgentIdComponent, &AutonomyPipelineComponent)`
/// never sees a pipeline without its identity, and a failed agent is still
/// nameable in a report.
///
/// This is the external identity used in reports, assertion targets, and (later)
/// determinism hashing — distinct from the Bevy `Entity`, which is an internal
/// handle: generational, run-unstable, and meaningless on hardware.
///
/// String-backed is fine while assertion targets are few. Interning the name
/// for cheap per-tick comparison at swarm scale is deliberately out of scope.
// TODO: Make String Arc<str> instead
#[derive(Component)]
pub struct AgentIdComponent(pub String);

/// Records that this agent's autonomy pipeline could not be assembled, and why.
///
/// The counterpart to [`AutonomyPipelineComponent`]: exactly one of the two
/// lands on every agent. Without it, a failed build is invisible — the agent
/// simply lacks a pipeline, and every downstream system that queries for one
/// finds nothing and does nothing, so the run proceeds as if the agent had no
/// autonomy to begin with.
///
/// The host decides what a failure means. This crate only records it, because
/// the right response differs by host: a test harness fails the run, an
/// interactive session might want the scene anyway to inspect it. Recording
/// per agent rather than aborting on the first also lets a multi-agent
/// scenario report every broken stack in one pass.
#[derive(Component)]
pub struct PipelineBuildFailed {
    /// Every error `build_pipeline` reported, already rendered for display.
    pub errors: Vec<String>,
}

/// Declares the bus channel name this sensor publishes on.
/// Attached to GPS, magnetometer, and LiDAR sensor child entities at spawn time.
/// Queried by `spawn_autonomy_pipeline` to build `sensor_frame_handles` for `build_pipeline`.
#[derive(Component, Clone)]
pub struct SensorPublishChannel(pub String);

/// The TrajectoryPoint produced by path following this tick.
/// Kept as a component for downstream consumers (vehicle adapters, debug gizmos).
#[derive(Component, Default)]
pub struct PathFollowingOutputComponent(pub Option<TrajectoryPoint>);

/// Marker on a virtual "odom frame" entity that links it back to its agent.
///
/// The odom entity carries `TrackedFrame` + `Transform` + `GlobalTransform`.
/// Its `Transform` is updated every tick by `update_odom_frames` to match
/// the pipeline's current pose estimate.
#[derive(Component)]
pub struct OdomFrameOf(pub Entity);
