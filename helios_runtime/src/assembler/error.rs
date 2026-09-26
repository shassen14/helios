//! Errors surfaced while assembling a pipeline from config.

use crate::pipeline::build_error::PipelineBuildError;
use crate::validation::ConfigValidationError;

/// Errors that can occur while assembling a pipeline from config.
#[derive(Debug)]
pub enum PipelineAssemblyError {
    /// The config failed static validation before any node was built. Carries
    /// every [`ConfigValidationError`] found, so a caller sees all config
    /// problems at once rather than one factory failure at a time.
    InvalidConfig(Vec<ConfigValidationError>),
    /// The config references an algorithm or model not in the registry.
    FactoryFailure { node_kind: String, reason: String },
    /// The assembled node graph failed topological validation.
    PipelineBuild(Vec<PipelineBuildError>),
    /// An estimator's aiding, augmentation, or IMU prediction entry names a
    /// sensor channel the host does not publish for this agent (absent from
    /// `sensor_channels`), so its input would never arrive or no sensor frame
    /// would resolve for it.
    UnknownSensorChannel {
        estimator_instance: String,
        input_channel: String,
    },
    /// An aiding entry names a `sensor_payload` the assembler does not
    /// recognize. (Should be caught first by `validate_autonomy_config`.)
    UnknownSensorPayload {
        estimator_instance: String,
        payload_kind: String,
    },
    /// An augmentation entry could not be turned into a state block: its `kind`
    /// matches no registered augmentation, or its noise parameters are invalid.
    /// `reason` carries the underlying `AugmentationError`'s message.
    AugmentationFailure {
        estimator_instance: String,
        reason: String,
    },
    /// A node reads a sensor channel that neither the host publishes
    /// (absent from `sensor_channels`) nor any preprocessing node derives, so
    /// its slot would stay empty forever and the node would silently never run
    /// its work.
    UnpublishedSensorInput { node_name: String, channel: String },
    /// A preprocessing node's `output` reuses the name of a channel the host
    /// publishes. Both are sensor channels, so a same-typed output would share
    /// the host's slot; even differently typed, one name for two channels
    /// misleads anyone reading the graph.
    PreprocessingOutputShadowsSensor {
        node_name: String,
        node_kind: String,
        channel: String,
    },
    /// `path_following` is present but no planner was configured to produce a
    /// path, and no explicit `path_source` was given.
    NoPathSourceForFollower,
    /// `path_following` names a `path_source` planner key that does not exist
    /// in `search_planners`.
    UnknownPathSource { path_source: String },
}

impl std::fmt::Display for PipelineAssemblyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PipelineAssemblyError::InvalidConfig(errs) => {
                write!(f, "invalid config: ")?;
                for (i, e) in errs.iter().enumerate() {
                    if i > 0 {
                        write!(f, "; ")?;
                    }
                    write!(f, "{e}")?;
                }
                Ok(())
            }
            PipelineAssemblyError::FactoryFailure { node_kind, reason } => {
                write!(f, "factory '{node_kind}' failed: {reason}")
            }
            PipelineAssemblyError::PipelineBuild(errs) => {
                write!(f, "pipeline graph errors: ")?;
                for (i, e) in errs.iter().enumerate() {
                    if i > 0 {
                        write!(f, "; ")?;
                    }
                    write!(f, "{e}")?;
                }
                Ok(())
            }
            PipelineAssemblyError::UnknownSensorChannel {
                estimator_instance,
                input_channel,
            } => {
                write!(
                    f,
                    "estimator '{estimator_instance}' names sensor channel '{input_channel}', which the host does not publish for this agent"
                )
            }
            PipelineAssemblyError::UnknownSensorPayload {
                estimator_instance,
                payload_kind,
            } => {
                write!(
                    f,
                    "estimator '{estimator_instance}' aiding entry has unknown sensor_payload '{payload_kind}'"
                )
            }
            PipelineAssemblyError::AugmentationFailure {
                estimator_instance,
                reason,
            } => {
                write!(
                    f,
                    "estimator '{estimator_instance}' augmentation failed: {reason}"
                )
            }
            PipelineAssemblyError::UnpublishedSensorInput { node_name, channel } => {
                write!(
                    f,
                    "node '{node_name}' reads sensor channel '{channel}', which the host does not publish and no preprocessing node produces"
                )
            }
            PipelineAssemblyError::PreprocessingOutputShadowsSensor {
                node_name,
                node_kind,
                channel,
            } => {
                write!(
                    f,
                    "{node_kind} preprocessing node '{node_name}' writes '{channel}', which is already a host sensor channel; give the output its own name"
                )
            }
            PipelineAssemblyError::NoPathSourceForFollower => {
                write!(f, "path_following configured but no planner produces a path and no path_source specified")
            }
            PipelineAssemblyError::UnknownPathSource { path_source } => {
                write!(f, "path_following.path_source '{path_source}' does not match any key in search_planners")
            }
        }
    }
}
