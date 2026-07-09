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
/// in the same insert as `AutonomyPipelineComponent`, so the two always land
/// together: the test bridge can query `(&AgentId, &AutonomyPipelineComponent)`
/// and never see one without the other.
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
