//! Narrow build contexts for `AutonomyRegistry` factories.
//!
//! Each context carries only the fields the corresponding factory actually
//! reads. No Bevy `Entity`, no full `AgentConfig`. The host (sim or hw)
//! resolves the agent-specific values and passes them here.

use crate::config::{AllocatorConfig, SensorModelConfig};
use crate::nodes::gaussian_estimator::AidingHandler;
use crate::port::InternalChannel;

use helios_core::estimation::schema::StateSchemaBlock;
use helios_core::prelude::AgentId;
use helios_core::spatial::FrameId;

/// Context for building a complete `GaussianEstimatorNode`.
///
/// The host resolves the agent's stable config identity (`AgentId`) and passes
/// it here so the factory never touches host-specific types. The factory builds
/// the agent's spine frames (`base_link`, `odom`) on demand from it.
///
/// Initial state (pose, uncertainty) is read from `EkfInitialStateConfig`
/// inside the `EkfConfig` by the factory — it is not a runtime parameter.
/// Aiding handlers are built by the assembler from `EkfConfig.aiding` and
/// passed here.
pub struct GaussianEstimatorBuildContext {
    pub agent: AgentId,
    /// Node name: the estimator's config-map key, so tooling keyed on the name
    /// distinguishes two estimators of the same kind.
    pub(crate) instance_name: String,
    pub(crate) aiding: Vec<Box<dyn AidingHandler>>,
    pub(crate) augmentation_blocks: Vec<StateSchemaBlock>,
}

/// Context for building a `MeasurementModel`.
///
/// Physical constants the model needs (gravity for accelerometer, magnetic
/// field for magnetometer) are carried in `model_config` — sourced from the
/// `SensorModelConfig` in `AidingConfig`. This keeps world-level constants in
/// config rather than threaded through call sites.
pub struct MeasurementModelBuildContext {
    pub agent: AgentId,
    /// The fully-resolved frame of the sensor this model observes, built by the
    /// assembler as `FrameId::sensor(agent, channel_name)`. The channel name is
    /// the single source the host also stamps its sensor entity with, so the
    /// model's TF lookups resolve against the same identity the host publishes.
    pub(crate) sensor: FrameId,
    pub(crate) model_config: SensorModelConfig,
}

/// Context for building an `AllocatorNode`.
pub struct AllocatorBuildContext {
    pub agent: AgentId,
    /// Node name: the allocator's config-map key, so tooling keyed on the name
    /// distinguishes two allocators of the same kind.
    pub(crate) instance_name: String,
    pub(crate) config: AllocatorConfig,
    /// The bus channel carrying the vehicle-level command this allocator
    /// consumes: the `[command]` fold its config names.
    pub(crate) input_channel: InternalChannel,
    /// The bus channel on which this node publishes its `ActuatorCommand`.
    pub(crate) output_channel: InternalChannel,
}

// ------- Mocks --------

pub(crate) struct MockEstimatorBuildContext {
    pub agent: AgentId,
    /// Node name: the estimator's config-map key, so tooling keyed on the name
    /// distinguishes two mocks of the same kind.
    pub(crate) instance_name: String,
}
