// helios_sim/src/simulation/plugins/autonomy/components.rs

use bevy::prelude::{Component, Entity};
use helios_core::data::primitives::TrajectoryPoint;
use helios_runtime::pipeline::AutonomyPipeline;

/// Holds the fully-assembled autonomy pipeline for an agent.
/// Written at spawn time by `spawn_autonomy_pipeline`. Ticked every frame
/// by `run_pipeline_tick` in `SimulationSet::Estimation`.
#[derive(Component)]
pub struct AutonomyPipelineComponent(pub AutonomyPipeline);

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
