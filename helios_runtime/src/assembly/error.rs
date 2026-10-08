//! Errors surfaced while assembling a pipeline from config.

use super::factory::FactoryError;

use crate::pipeline::PipelineBuildError;

use helios_core::control::actuators::SetpointKind;

/// Errors that can occur while assembling a pipeline from config.
#[derive(Debug)]
pub enum PipelineAssemblyError {
    /// The assembled node graph failed topological validation.
    PipelineBuild(Vec<PipelineBuildError>),
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
    /// Member `member` of `[actuators]` declares no actuator it drives, so
    /// whether it collides with another member, or drives anything the body
    /// has, cannot be checked.
    UndeclaredDrives { member: String },
    /// `actuator` is driven by more than one `[actuators]` member. The merge
    /// would keep one member's setpoint and drop the others'. `members` is
    /// sorted.
    ActuatorDrivenTwice {
        actuator: String,
        members: Vec<String>,
    },
    /// Member `member` of `[actuators]` drives `actuator`, which body `body`
    /// does not have: usually a typo in the actuator name.
    ActuatorNotOnBody {
        member: String,
        actuator: String,
        body: String,
    },
    /// Member `member` of `[actuators]` writes a `writes` setpoint to
    /// `actuator`, which accepts only `accepts`: a torque into a
    /// velocity-driven wheel, which nothing downstream could detect.
    ActuatorKindMismatch {
        member: String,
        actuator: String,
        writes: SetpointKind,
        accepts: SetpointKind,
    },
}

impl std::fmt::Display for PipelineAssemblyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
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
            PipelineAssemblyError::UndeclaredDrives { member } => write!(
                f,
                "actuators: member '{member}' declares no actuator it drives; its factory must declare each one"
            ),
            PipelineAssemblyError::ActuatorDrivenTwice { actuator, members } => write!(
                f,
                "actuators: actuator '{actuator}' is driven by more than one member ({}); each actuator takes one",
                members.join(", ")
            ),
            PipelineAssemblyError::ActuatorNotOnBody {
                member,
                actuator,
                body,
            } => write!(
                f,
                "actuators: member '{member}' drives actuator '{actuator}', which body '{body}' does not have"
            ),
            PipelineAssemblyError::ActuatorKindMismatch {
                member,
                actuator,
                writes,
                accepts,
            } => write!(
                f,
                "actuators: member '{member}' writes {writes:?} to actuator '{actuator}', which accepts {accepts:?}"
            ),
        }
    }
}

impl From<FactoryError> for PipelineAssemblyError {
    fn from(value: FactoryError) -> Self {
        PipelineAssemblyError::Factory(value)
    }
}
