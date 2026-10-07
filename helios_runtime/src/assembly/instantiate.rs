//! Builds every `[nodes.<name>]` entry through the factory registered for its
//! `kind`.
//!
//! All or nothing: either every node builds, or every node that failed is
//! reported and none are returned. A pipeline missing a node is never built,
//! and stopping here keeps one broken node from showing up again downstream
//! as unpublished inputs on every node that reads its outputs.

use super::error::PipelineAssemblyError;
use super::factory::BuildContext;
use super::registry::AutonomyRegistry;

use crate::config::AutonomyStack;
use crate::pipeline::node::PipelineNode;
use crate::port::ChannelKey;

use helios_core::prelude::AgentId;

use std::collections::HashSet;

/// Every `[nodes]` entry, built.
pub(super) struct Instantiated {
    /// The built nodes, in name order.
    pub(super) nodes: Vec<Box<dyn PipelineNode>>,
    /// Every outside input the factories declared, in node-name order. Two
    /// nodes reading one outside channel each declare it, so it can repeat.
    pub(super) outside_inputs: Vec<ChannelKey>,
}

/// Builds every node in `stack.nodes`, in name order.
///
/// Fails with one error per node that could not be built.
pub(super) fn instantiate(
    stack: &AutonomyStack,
    registry: &AutonomyRegistry,
    agent: &AgentId,
    sensor_channels: &HashSet<String>,
) -> Result<Instantiated, Vec<PipelineAssemblyError>> {
    let mut nodes = Vec::new();
    let mut outside_inputs = Vec::new();
    let mut errors = Vec::new();

    for (name, section) in &stack.nodes {
        let section = section.clone();
        match instantiate_node(name, section, registry, agent, sensor_channels) {
            Ok((node, node_outside_inputs)) => {
                nodes.push(node);
                outside_inputs.extend(node_outside_inputs);
            }
            Err(err) => errors.push(err),
        }
    }

    if errors.is_empty() {
        Ok(Instantiated {
            nodes,
            outside_inputs,
        })
    } else {
        Err(errors)
    }
}

/// Builds node `name` from its `section`: takes `kind` out of the section and
/// hands the rest to that kind's factory, then checks the factory named the
/// node `name`. Returns the node and the outside inputs its factory declared.
fn instantiate_node(
    name: &str,
    mut section: toml::Table,
    registry: &AutonomyRegistry,
    agent: &AgentId,
    sensor_channels: &HashSet<String>,
) -> Result<(Box<dyn PipelineNode>, Vec<ChannelKey>), PipelineAssemblyError> {
    let Some(toml::Value::String(kind)) = section.remove("kind") else {
        return Err(PipelineAssemblyError::MissingNodeKind {
            node_name: name.to_string(),
        });
    };

    let ctx = BuildContext::new(agent.clone(), name, sensor_channels);

    let built = registry.build_node(&kind, section, &ctx)?;
    let outside_inputs = built.output.outside_inputs().to_vec();
    let node = built.output.into_node();

    if node.name() != name {
        return Err(PipelineAssemblyError::NodeNameMismatch {
            node_name: name.to_string(),
            kind,
            built_name: node.name().to_string(),
        });
    }

    Ok((node, outside_inputs))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::factory::FactoryOutput;
    use crate::pipeline::node::TickContext;
    use crate::port::{AlgorithmNodePortDescriptor, InternalChannel, PortBus, PortDescriptor};

    use helios_core::prelude::TfProvider;

    use serde::{Deserialize, Serialize};

    const NAMED_KIND: &str = "TestNamed";
    const MISNAMED_KIND: &str = "TestMisnamed";
    const OUTSIDE_KIND: &str = "TestOutside";
    const WRONG_NAME: &str = "not_the_key";

    /// A node that does nothing; only its name matters here.
    struct StubNode {
        name: String,
        descriptor: PortDescriptor,
    }

    impl PipelineNode for StubNode {
        fn name(&self) -> &str {
            &self.name
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    #[derive(Deserialize, Serialize)]
    #[serde(deny_unknown_fields)]
    struct EmptyConfig {}

    fn stub(name: &str) -> Result<FactoryOutput, String> {
        Ok(FactoryOutput::new(Box::new(StubNode {
            name: name.to_string(),
            descriptor: AlgorithmNodePortDescriptor::new().build(),
        })))
    }

    /// Names the node after its table key, as every factory should.
    fn build_named(_config: EmptyConfig, ctx: &BuildContext<'_>) -> Result<FactoryOutput, String> {
        stub(ctx.node_name())
    }

    /// Ignores the table key and picks its own name.
    fn build_misnamed(
        _config: EmptyConfig,
        _ctx: &BuildContext<'_>,
    ) -> Result<FactoryOutput, String> {
        stub(WRONG_NAME)
    }

    /// Declares one outside input, named after the node.
    fn build_outside(
        _config: EmptyConfig,
        ctx: &BuildContext<'_>,
    ) -> Result<FactoryOutput, String> {
        Ok(stub(ctx.node_name())?.with_outside_input(outside_key(ctx.node_name())))
    }

    fn outside_key(name: &str) -> ChannelKey {
        InternalChannel::named::<f64>(name).into()
    }

    fn registry() -> AutonomyRegistry {
        let mut registry = AutonomyRegistry::default();
        registry
            .register_node(NAMED_KIND, build_named)
            .expect("test kind is new");
        registry
            .register_node(MISNAMED_KIND, build_misnamed)
            .expect("test kind is new");
        registry
            .register_node(OUTSIDE_KIND, build_outside)
            .expect("test kind is new");
        registry
    }

    fn run(stack_toml: &str) -> Result<Instantiated, Vec<PipelineAssemblyError>> {
        let stack: AutonomyStack = toml::from_str(stack_toml).expect("test TOML parses");
        instantiate(&stack, &registry(), &AgentId::new("car"), &HashSet::new())
    }

    /// Unwraps the failure of [`run`]; `dyn PipelineNode` has no `Debug`, so
    /// `unwrap_err` isn't available.
    fn run_err(stack_toml: &str) -> Vec<PipelineAssemblyError> {
        match run(stack_toml) {
            Ok(_) => panic!("expected instantiation to fail"),
            Err(errors) => errors,
        }
    }

    /// Every node builds, in table-key order, and the `kind` key never reaches
    /// the factory (whose config denies unknown fields).
    #[test]
    fn every_node_builds_in_name_order() {
        let built = match run(&format!(
            r#"
            [nodes.b]
            kind = "{NAMED_KIND}"

            [nodes.a]
            kind = "{NAMED_KIND}"
            "#
        )) {
            Ok(built) => built,
            Err(errors) => panic!("expected every node to build, got {errors:?}"),
        };
        let names: Vec<&str> = built.nodes.iter().map(|node| node.name()).collect();
        assert_eq!(names, ["a", "b"]);
        assert!(built.outside_inputs.is_empty());
    }

    /// The outside inputs every factory declared come back together, in
    /// node-name order, and a node that declares none adds nothing.
    #[test]
    fn outside_inputs_are_collected_in_name_order() {
        let built = match run(&format!(
            r#"
            [nodes.b]
            kind = "{OUTSIDE_KIND}"

            [nodes.c]
            kind = "{NAMED_KIND}"

            [nodes.a]
            kind = "{OUTSIDE_KIND}"
            "#
        )) {
            Ok(built) => built,
            Err(errors) => panic!("expected every node to build, got {errors:?}"),
        };
        assert_eq!(built.outside_inputs, [outside_key("a"), outside_key("b")]);
    }

    /// A table with no `kind`, or a `kind` that isn't a string, names the node.
    #[test]
    fn missing_or_non_string_kind_is_reported() {
        let errors = run_err(
            r#"
            [nodes.no_kind]

            [nodes.number_kind]
            kind = 5
            "#,
        );
        assert!(
            matches!(
                errors.as_slice(),
                [
                    PipelineAssemblyError::MissingNodeKind { node_name: first },
                    PipelineAssemblyError::MissingNodeKind { node_name: second },
                ] if first == "no_kind" && second == "number_kind"
            ),
            "got {errors:?}"
        );
    }

    /// A factory that names its node something other than the table key is
    /// rejected, naming the key, the kind and the name it chose.
    #[test]
    fn node_not_named_after_its_key_is_rejected() {
        let errors = run_err(&format!(
            r#"
            [nodes.front]
            kind = "{MISNAMED_KIND}"
            "#
        ));
        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::NodeNameMismatch { node_name, kind, built_name }]
                    if node_name == "front" && kind == MISNAMED_KIND && built_name == WRONG_NAME
            ),
            "got {errors:?}"
        );
    }

    /// One bad node fails the whole set: every failure is reported, and the
    /// nodes that did build are not returned.
    #[test]
    fn every_failure_is_reported_and_no_nodes_are_returned() {
        let errors = run_err(&format!(
            r#"
            [nodes.good]
            kind = "{NAMED_KIND}"

            [nodes.no_kind]

            [nodes.unknown]
            kind = "TestNoSuchKind"
            "#
        ));
        assert!(
            matches!(
                errors.as_slice(),
                [
                    PipelineAssemblyError::MissingNodeKind { .. },
                    PipelineAssemblyError::UnknownNodeKind { .. },
                ]
            ),
            "got {errors:?}"
        );
    }
}
