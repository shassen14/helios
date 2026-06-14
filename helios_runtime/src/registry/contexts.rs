//! Narrow build contexts for `AutonomyRegistry` factories.
//!
//! Each context carries only the fields the corresponding factory actually
//! reads. No Bevy `Entity`, no full `AgentConfig`. The host (sim or hw)
//! resolves the agent-specific values and passes them here.

use crate::config::{
    ControllerConfig, MapLayerConfig, PathFollowingConfig, SearchPlannerConfig, SensorModelConfig,
};
use crate::pipeline::nodes::gaussian_estimator::AidingHandler;
use crate::port::InternalChannel;

use helios_core::data::primitives::FrameHandle;

/// Context for building a dynamics model (e.g. `IntegratedImuModel`).
///
/// `gravity` is sourced from the dynamics config (`EkfDynamicsConfig::gravity()`)
/// by the gaussian estimator factory — callers outside the assembler should not
/// need to supply it independently.
pub struct DynamicsBuildContext {
    pub agent_handle: FrameHandle,
    pub(crate) gravity: f64,
}

/// Context for building a complete `GaussianEstimatorNode`.
///
/// The host derives `agent_handle` from its entity system
/// (e.g. `FrameHandle::from_entity(entity)` in Bevy sim) and passes it here
/// so the factory never touches host-specific types.
///
/// Initial state (pose, uncertainty) is read from `EkfInitialStateConfig`
/// inside the `EkfConfig` by the factory — it is not a runtime parameter.
/// Aiding handlers are built by the assembler from `EkfConfig.aiding` and
/// passed here.
pub struct GaussianEstimatorBuildContext {
    pub agent_handle: FrameHandle,
    pub(crate) aiding: Vec<Box<dyn AidingHandler>>,
}

/// Context for building a `MeasurementModel`.
///
/// Physical constants the model needs (gravity for accelerometer, magnetic
/// field for magnetometer) are carried in `model_config` — sourced from the
/// `SensorModelConfig` in `AidingConfig`. This keeps world-level constants in
/// config rather than threaded through call sites.
pub struct MeasurementModelBuildContext {
    pub agent_handle: FrameHandle,
    pub(crate) sensor_handle: FrameHandle,
    pub(crate) model_config: SensorModelConfig,
}

/// Context for building an `OccupancyGridNode` (or any `Mapper`-backed node).
pub struct MapperBuildContext {
    pub agent_handle: FrameHandle,
    pub(crate) config: MapLayerConfig,
}

/// Context for building a `ControllerNode`.
pub struct ControllerBuildContext {
    pub agent_handle: FrameHandle,
    pub(crate) config: ControllerConfig,
}

/// Context for building a `SearchPlannerNode`.
pub struct SearchPlannerBuildContext {
    pub agent_handle: FrameHandle,
    pub(crate) config: SearchPlannerConfig,
    /// The bus channel on which the upstream mapper publishes `MapData`.
    /// Always internal (brain-produced).
    pub(crate) map_channel: InternalChannel,
    /// The bus channel on which this node will publish its `Path` output.
    pub(crate) path_channel: InternalChannel,
}

/// Context for building a `PathFollowerNode`.
pub struct PathFollowerBuildContext {
    pub agent_handle: FrameHandle,
    pub(crate) config: PathFollowingConfig,
    /// The bus channel on which the upstream planner publishes `Path`.
    /// Always internal (brain-produced).
    pub(crate) path_channel: InternalChannel,
}

// ------- Mocks --------

pub(crate) struct MockEstimatorBuildContext {
    pub agent_handle: FrameHandle,
}
