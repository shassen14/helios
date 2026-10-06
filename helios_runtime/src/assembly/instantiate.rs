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

use helios_core::prelude::AgentId;

use std::collections::HashSet;

/// Builds every node in `stack.nodes`, in name order.
///
/// Fails with one error per node that could not be built.
pub(super) fn instantiate(
    stack: &AutonomyStack,
    registry: &AutonomyRegistry,
    agent: &AgentId,
    sensor_channels: &HashSet<String>,
) -> Result<Vec<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>> {
    let mut nodes = Vec::new();
    let mut errors = Vec::new();

    for (name, section) in &stack.nodes {
        let section = section.clone();
        match instantiate_node(name, section, registry, agent, sensor_channels) {
            Ok(node) => nodes.push(node),
            Err(err) => errors.push(err),
        }
    }

    if errors.is_empty() {
        Ok(nodes)
    } else {
        Err(errors)
    }
}

/// Builds node `name` from its `section`: takes `kind` out of the section and
/// hands the rest to that kind's factory, then checks the factory named the
/// node `name`.
fn instantiate_node(
    name: &str,
    mut section: toml::Table,
    registry: &AutonomyRegistry,
    agent: &AgentId,
    sensor_channels: &HashSet<String>,
) -> Result<Box<dyn PipelineNode>, PipelineAssemblyError> {
    let Some(toml::Value::String(kind)) = section.remove("kind") else {
        return Err(PipelineAssemblyError::MissingNodeKind {
            node_name: name.to_string(),
        });
    };

    let ctx = BuildContext::new(agent.clone(), name, sensor_channels);

    let built = registry.build_node(&kind, section, &ctx)?;
    let node = built.output.into_node();

    if node.name() != name {
        return Err(PipelineAssemblyError::NodeNameMismatch {
            node_name: name.to_string(),
            kind,
            built_name: node.name().to_string(),
        });
    }

    Ok(node)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::factory::FactoryOutput;
    use crate::pipeline::node::TickContext;
    use crate::port::{AlgorithmNodePortDescriptor, PortBus, PortDescriptor};

    use helios_core::prelude::TfProvider;

    use serde::{Deserialize, Serialize};

    const NAMED_KIND: &str = "TestNamed";
    const MISNAMED_KIND: &str = "TestMisnamed";
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

    fn registry() -> AutonomyRegistry {
        let mut registry = AutonomyRegistry::default();
        registry
            .register_node(NAMED_KIND, build_named)
            .expect("test kind is new");
        registry
            .register_node(MISNAMED_KIND, build_misnamed)
            .expect("test kind is new");
        registry
    }

    fn run(stack_toml: &str) -> Result<Vec<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>> {
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
        let nodes = match run(&format!(
            r#"
            [nodes.b]
            kind = "{NAMED_KIND}"

            [nodes.a]
            kind = "{NAMED_KIND}"
            "#
        )) {
            Ok(nodes) => nodes,
            Err(errors) => panic!("expected every node to build, got {errors:?}"),
        };
        let names: Vec<&str> = nodes.iter().map(|node| node.name()).collect();
        assert_eq!(names, ["a", "b"]);
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
