//! Errors surfaced while assembling a pipeline from config.

use super::factory::FactoryError;

use crate::pipeline::PipelineBuildError;
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
    /// (absent from `sensor_channels`) nor any node in the graph derives, so
    /// its slot would stay empty forever and the node would silently never run
    /// its work.
    UnpublishedSensorInput { node_name: String, channel: String },
    /// Node `node_name` writes a sensor channel under the name of one the host
    /// publishes. Both are sensor channels, so a same-typed output would share
    /// the host's slot; even differently typed, one name for two channels
    /// misleads anyone reading the graph.
    SensorOutputShadowsHost { node_name: String, channel: String },
    /// The `kind` of node `node_name` matches no registered factory.
    /// `registered` lists the kinds that do exist, sorted.
    UnknownNodeKind {
        node_name: String,
        kind: String,
        registered: Vec<String>,
    },
    /// A node kind's factory rejected its config section or failed to build.
    /// The wrapped error names the node, the kind and the cause.
    Factory(FactoryError),
    /// Node `node_name`'s table has no `kind`, or its `kind` is not a string,
    /// so there is no factory to look up.
    MissingNodeKind { node_name: String },
    /// The factory for node `node_name` built a node named `built_name`. A
    /// node's name is its table key, since errors and wiring refer to it by
    /// that name; a factory that picks another is a bug in the factory.
    NodeNameMismatch {
        node_name: String,
        kind: String,
        built_name: String,
    },
    /// The `[seam]` section names `member`, which is not a `[nodes]` entry.
    UnknownSeamMember { seam: String, member: String },
    /// The `[seam]` section names `member` more than once, so its priority is
    /// ambiguous.
    DuplicateSeamMember { seam: String, member: String },
    /// Member `member` of `[seam]` has `matching` outputs of the seam's type
    /// `expected`. A member must have exactly one, so the seam knows which
    /// channel to read.
    SeamMemberOutputMismatch {
        seam: String,
        member: String,
        expected: &'static str,
        matching: usize,
    },
    /// The `[seam]` section names no members, so the node combining them
    /// would never publish.
    EmptySeam { seam: String },
    /// Command fold `fold` states a `type` no command type is registered
    /// under. `registered` lists the names that are, sorted.
    UnknownCommandType {
        fold: String,
        type_name: String,
        registered: Vec<String>,
    },
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
                    "node '{node_name}' reads sensor channel '{channel}', which the host does not publish and no node produces"
                )
            }
            PipelineAssemblyError::SensorOutputShadowsHost { node_name, channel } => {
                write!(
                    f,
                    "node '{node_name}' writes sensor channel '{channel}', which is already a host sensor channel; give the output its own name"
                )
            }
            PipelineAssemblyError::UnknownNodeKind {
                node_name,
                kind,
                registered,
            } => {
                let registered = if registered.is_empty() {
                    "(none)".to_string()
                } else {
                    registered.join(", ")
                };
                write!(
                    f,
                    "nodes.{node_name}: unknown kind '{kind}'; registered kinds: {registered}"
                )
            }
            PipelineAssemblyError::Factory(err) => write!(f, "{err}"),
            PipelineAssemblyError::MissingNodeKind { node_name } => write!(
                f,
                "nodes.{node_name}: needs a string 'kind' naming which node kind to build"
            ),
            PipelineAssemblyError::NodeNameMismatch {
                node_name,
                kind,
                built_name,
            } => write!(
                f,
                "nodes.{node_name}: the '{kind}' factory named its node '{built_name}'; a factory must name the node after its table key"
            ),
            PipelineAssemblyError::UnknownSeamMember { seam, member } => write!(
                f,
                "{seam}: names '{member}', which is not a [nodes] entry"
            ),
            PipelineAssemblyError::DuplicateSeamMember { seam, member } => write!(
                f,
                "{seam}: names '{member}' more than once; each member takes one place"
            ),
            PipelineAssemblyError::SeamMemberOutputMismatch {
                seam,
                member,
                expected,
                matching,
            } => write!(
                f,
                "{seam}: member '{member}' has {matching} outputs of type {expected}; a member needs exactly one"
            ),
            PipelineAssemblyError::EmptySeam { seam } => write!(
                f,
                "{seam}: names no members; list at least one, or remove the section"
            ),
            PipelineAssemblyError::UnknownCommandType {
                fold,
                type_name,
                registered,
            } => write!(
                f,
                "command.{fold}: unknown type '{type_name}'; registered command types: {}",
                if registered.is_empty() {
                    "(none)".to_string()
                } else {
                    registered.join(", ")
                }
            ),
        }
    }
}

impl From<FactoryError> for PipelineAssemblyError {
    fn from(value: FactoryError) -> Self {
        PipelineAssemblyError::Factory(value)
    }
}
