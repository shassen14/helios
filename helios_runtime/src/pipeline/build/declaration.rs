//! What the builder checks and orders: a node declared by its name and its
//! ports, either already built or a [`NodePlan`] built once the graph is
//! known to be sound.

use super::mounts::InputMounts;

use crate::{
    port::ChannelDescription, ChannelKey, PipelineBuildError, PipelineNode, PortDescriptor,
};

use std::{collections::HashMap, error::Error};

/// Builds a planned node from the ports it declared and the mounts of its
/// inputs. The node keeps the descriptor it is given, so what the builder
/// checked is what it runs.
pub type NodeConstructor = Box<
    dyn FnOnce(
            PortDescriptor,
            InputMounts,
        ) -> Result<Box<dyn PipelineNode>, Box<dyn Error + Send + Sync>>
        + Send,
>;

/// A node declared before it is built: its name, the ports it will have, and
/// how to build it.
///
/// The builder checks wiring and orders levels on the declared ports, and
/// builds the node only after both pass, handing it the mounts of its
/// inputs. A node whose construction needs something the graph knows only
/// once it is wired, such as the frame a sensor it reads is mounted in, is
/// declared this way.
pub struct NodePlan {
    name: String,
    descriptor: PortDescriptor,
    constructor: NodeConstructor,
}

impl NodePlan {
    /// A plan for the node `name`, with ports `descriptor`, built by
    /// `constructor`. The node it builds must carry the same name.
    pub fn new(
        name: impl Into<String>,
        descriptor: PortDescriptor,
        constructor: NodeConstructor,
    ) -> Self {
        Self {
            name: name.into(),
            descriptor,
            constructor,
        }
    }

    /// The name the planned node will carry.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The ports the planned node will have.
    pub fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    /// Builds the node with the mounts of its inputs, refusing one the
    /// constructor names differently.
    #[expect(
        clippy::result_large_err,
        reason = "runs once per node at build, and build collects its errors in a Vec anyway"
    )]
    fn construct(self, mounts: InputMounts) -> Result<Box<dyn PipelineNode>, PipelineBuildError> {
        let node = (self.constructor)(self.descriptor, mounts).map_err(|error| {
            PipelineBuildError::ConstructionFailed {
                node_name: self.name.clone(),
                error,
            }
        })?;

        if node.name() != self.name {
            return Err(PipelineBuildError::ConstructedNodeMisnamed {
                node_name: self.name,
                built_name: node.name().to_string(),
            });
        }

        Ok(node)
    }
}

/// One node as the builder sees it before building the pipeline: built, or
/// planned. Both answer the name and the ports, which is all the checks read.
pub(crate) enum NodeDeclaration {
    Built(Box<dyn PipelineNode>),
    Planned(NodePlan),
}

impl NodeDeclaration {
    /// The node's name.
    pub(crate) fn name(&self) -> &str {
        match self {
            Self::Built(node) => node.name(),
            Self::Planned(plan) => plan.name(),
        }
    }

    /// The node's declared ports.
    pub(crate) fn port_descriptor(&self) -> &PortDescriptor {
        match self {
            Self::Built(node) => node.port_descriptor(),
            Self::Planned(plan) => plan.port_descriptor(),
        }
    }

    /// The node, built if it was planned, with the mounts of its inputs from
    /// `descriptions`, every channel's resolved description by key.
    #[expect(
        clippy::result_large_err,
        reason = "runs once per node at build, and build collects its errors in a Vec anyway"
    )]
    pub(crate) fn construct(
        self,
        descriptions: &HashMap<&ChannelKey, &ChannelDescription>,
    ) -> Result<Box<dyn PipelineNode>, PipelineBuildError> {
        match self {
            Self::Built(node) => Ok(node),
            Self::Planned(plan) => {
                let mounts = InputMounts::new(plan.name(), plan.port_descriptor(), descriptions);
                plan.construct(mounts)
            }
        }
    }
}

impl From<Box<dyn PipelineNode>> for NodeDeclaration {
    fn from(node: Box<dyn PipelineNode>) -> Self {
        Self::Built(node)
    }
}

impl From<NodePlan> for NodeDeclaration {
    fn from(plan: NodePlan) -> Self {
        Self::Planned(plan)
    }
}
