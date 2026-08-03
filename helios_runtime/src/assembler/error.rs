//! Errors surfaced while assembling a pipeline from config.

use crate::pipeline::build_error::PipelineBuildError;

/// Errors that can occur while assembling a pipeline from config.
#[derive(Debug)]
pub enum PipelineAssemblyError {
    /// The config references an algorithm or model not in the registry.
    FactoryFailure { node_kind: String, reason: String },
    /// The assembled node graph failed topological validation.
    PipelineBuild(Vec<PipelineBuildError>),
    /// An aiding entry names a sensor channel with no corresponding
    /// [`FrameHandle`](helios_core::data::primitives::FrameHandle) in
    /// `sensor_frame_handles`.
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
                    "estimator '{estimator_instance}' aiding channel '{input_channel}' has no FrameHandle in sensor_frame_handles"
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
            PipelineAssemblyError::NoPathSourceForFollower => {
                write!(f, "path_following configured but no planner produces a path and no path_source specified")
            }
            PipelineAssemblyError::UnknownPathSource { path_source } => {
                write!(f, "path_following.path_source '{path_source}' does not match any key in search_planners")
            }
        }
    }
}
