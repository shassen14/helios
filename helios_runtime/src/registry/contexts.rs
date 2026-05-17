//! Narrow build contexts for `AutonomyRegistry` factories.
//!
//! Each context carries only the fields the corresponding factory actually
//! reads. No Bevy `Entity`, no full `AgentConfig`. The host (sim or hw)
//! resolves the agent-specific values and passes them here.

use helios_core::data::primitives::FrameHandle;
use nalgebra::Isometry3;

use crate::config::{ControllerConfig, MapLayerConfig, PathFollowingConfig, SearchPlannerConfig};
use crate::pipeline::nodes::gaussian_estimator::AidingHandler;

/// Context for building a dynamics model (e.g. `IntegratedImuModel`).
pub struct DynamicsBuildContext {
    pub agent_handle: FrameHandle,
    pub gravity: f64,
}

/// Context for building a complete `GaussianEstimatorNode`.
///
/// The host derives `agent_handle` from whatever entity system it uses
/// (e.g. `FrameHandle::from_entity(entity)` in Bevy sim) and passes it
/// here so the factory never touches Bevy types.
///
/// `aiding` holds any pre-built aiding handlers for this agent's sensor suite.
/// The assembler (Step 7c) builds these from the `EkfConfig.aiding` list and
/// passes them here. For the Step 7b registry alone, pass `vec![]`.
pub struct GaussianEstimatorBuildContext {
    pub agent_handle: FrameHandle,
    /// Agent starting pose in the ENU world frame. Used to seed the filter's
    /// initial state so it begins with the correct heading.
    pub starting_pose: Isometry3<f64>,
    pub gravity: f64,
    pub aiding: Vec<Box<dyn AidingHandler>>,
}

/// Context for building a `MeasurementModel`.
///
/// All four currently registered models (GPS, accelerometer, gyroscope,
/// magnetometer) resolve their sensor geometry from the TF tree at
/// prediction time, so all require `agent_handle` and `sensor_handle`.
/// `gravity` and `world_magnetic_field` are physical constants used by the
/// accelerometer and magnetometer models respectively.
pub struct MeasurementModelBuildContext {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub gravity: f64,
    pub world_magnetic_field: Option<nalgebra::Vector3<f64>>,
}

/// Context for building an `OccupancyGridNode` (or any `Mapper`-backed node).
pub struct MapperBuildContext {
    pub agent_handle: FrameHandle,
    pub config: MapLayerConfig,
}

/// Context for building a `ControllerNode`.
pub struct ControllerBuildContext {
    pub agent_handle: FrameHandle,
    pub config: ControllerConfig,
}

/// Context for building a `SearchPlannerNode`.
pub struct SearchPlannerBuildContext {
    pub agent_handle: FrameHandle,
    pub config: SearchPlannerConfig,
    /// The bus channel on which the upstream mapper publishes `MapData`.
    pub map_channel: crate::port::ChannelKey,
    /// The bus channel on which this node will publish its `Path` output.
    pub path_channel: crate::port::ChannelKey,
}

/// Context for building a `PathFollowerNode`.
pub struct PathFollowerBuildContext {
    pub agent_handle: FrameHandle,
    pub config: PathFollowingConfig,
    /// The bus channel on which the upstream planner publishes `Path`.
    pub path_channel: crate::port::ChannelKey,
}
