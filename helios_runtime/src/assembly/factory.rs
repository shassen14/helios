//! What a node factory receives and returns.
//!
//! A node kind is built by a factory: given the node's own config section and
//! a [`BuildContext`], it returns a [`FactoryOutput`] holding the built node
//! and the outside inputs it reads, or a [`FactoryError`] naming the node and
//! kind that failed.

use crate::port::ChannelKey;
use crate::prelude::PipelineNode;

use helios_core::prelude::AgentId;
use serde::{de::DeserializeOwned, Serialize};

use std::{collections::HashSet, error::Error, fmt::Display};

/// The key naming a config section's kind: a `[nodes.<name>]` table's node
/// kind, or a component sub-table's component kind.
pub(crate) const KIND_KEY: &str = "kind";

/// What every factory receives besides its own config: the values that come
/// from the agent and the host rather than from the node's section.
///
/// Fields are private behind accessors, so a new field doesn't break the
/// factories that don't read it.
pub struct BuildContext<'a> {
    agent: AgentId,
    node_name: &'a str,
    sensor_channels: &'a HashSet<String>,
}

impl<'a> BuildContext<'a> {
    pub(crate) fn new(
        agent: AgentId,
        node_name: &'a str,
        sensor_channels: &'a HashSet<String>,
    ) -> Self {
        Self {
            agent,
            node_name,
            sensor_channels,
        }
    }

    /// The agent this node belongs to; scopes the node's frames.
    pub fn agent(&self) -> &AgentId {
        &self.agent
    }

    /// The node's name: its key in the config's node table.
    pub fn node_name(&self) -> &'a str {
        self.node_name
    }

    /// Names of the sensor channels the host publishes for this agent.
    pub fn sensor_channels(&self) -> &'a HashSet<String> {
        self.sensor_channels
    }
}

/// What a factory returns on success.
///
/// Fields are private behind a constructor, so adding a field doesn't break
/// factories written outside this crate.
pub struct FactoryOutput {
    node: Box<dyn PipelineNode>,
    outside_inputs: Vec<ChannelKey>,
}

impl FactoryOutput {
    /// Wraps the built node, declaring no outside inputs.
    pub fn new(node: Box<dyn PipelineNode>) -> Self {
        Self {
            node,
            outside_inputs: Vec::new(),
        }
    }

    /// Declares `key` as an outside input: a channel the node reads that an
    /// operator or mission system writes, rather than a node or the body.
    ///
    /// The factory declares it, not the node's port descriptor, because where
    /// an input comes from depends on the stack: the same node may read a goal
    /// the host sends in one stack and a goal another node produces in the
    /// next. A channel declared here must have no producer in the graph.
    pub fn with_outside_input(mut self, key: impl Into<ChannelKey>) -> Self {
        self.outside_inputs.push(key.into());
        self
    }

    /// The outside inputs this factory declared, in declaration order.
    pub(crate) fn outside_inputs(&self) -> &[ChannelKey] {
        &self.outside_inputs
    }

    /// Takes the built node out, for the assembler to add to the pipeline.
    pub(crate) fn into_node(self) -> Box<dyn PipelineNode> {
        self.node
    }
}

/// Why a factory failed. Every variant names the node and its kind, so the
/// message points at the config entry to fix.
#[derive(Debug)]
pub enum FactoryError {
    /// The node's config section did not parse into the kind's config type:
    /// an unknown key, a missing field, or a value of the wrong type. `key`
    /// is the path to the offending key inside the section; empty or `"."`
    /// when the error is about the section as a whole.
    InvalidConfig {
        node_name: String,
        kind: String,
        key: String,
        message: String,
    },
    /// The config parsed, but the kind's build function rejected it.
    BuildFailed {
        node_name: String,
        kind: String,
        reason: String,
    },
    /// The parsed config did not serialize back to a table for the resolved
    /// config dump. A bug in the kind's config type, not in the user's config.
    ResolveFailed {
        node_name: String,
        kind: String,
        message: String,
    },
}

impl Display for FactoryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfig {
                node_name,
                kind,
                key,
                message,
            } => {
                if key.is_empty() || key == "." {
                    write!(f, "nodes.{node_name} (kind '{kind}'): {message}")
                } else {
                    write!(f, "nodes.{node_name}.{key} (kind '{kind}'): {message}")
                }
            }
            Self::BuildFailed {
                node_name,
                kind,
                reason,
            } => write!(
                f,
                "nodes.{node_name} (kind '{kind}'): build failed: {reason}"
            ),
            Self::ResolveFailed {
                node_name,
                kind,
                message,
            } => write!(
                f,
                "nodes.{node_name} (kind '{kind}'): config type does not serialize back to a TOML table: {message}"
            ),
        }
    }
}

impl Error for FactoryError {}

/// What the registry gets back from an [`ErasedFactory`]: the factory's output
/// plus the node's resolved config section, defaults filled in, for the
/// resolved-config dump.
pub(crate) struct BuiltNode {
    pub(crate) output: FactoryOutput,
    pub(crate) resolved: toml::Table,
}

/// A node-kind factory with its config type hidden, so factories for every
/// kind fit in one map. Takes the node's raw config section (common keys
/// already removed) and its context. Built by [`erase`].
pub(crate) type ErasedFactory =
    Box<dyn Fn(toml::Table, &BuildContext<'_>) -> Result<BuiltNode, FactoryError> + Send + Sync>;

/// Wraps a kind's typed build function into an [`ErasedFactory`].
///
/// The returned closure parses the raw section strictly into `C` (the config
/// type must deny unknown fields, which a trait bound can't require),
/// serializes the parsed config back as the resolved section, then calls
/// `build`. Serializing comes before building because `build` takes the config
/// by value. `kind` is kept only to name the kind in errors.
pub(crate) fn erase<C, F>(kind: String, build: F) -> ErasedFactory
where
    C: DeserializeOwned + Serialize + 'static,
    F: Fn(C, &BuildContext<'_>) -> Result<FactoryOutput, String> + Send + Sync + 'static,
{
    Box::new(move |section: toml::Table, ctx: &BuildContext<'_>| {
        let config: C = serde_path_to_error::deserialize(section).map_err(|err| {
            FactoryError::InvalidConfig {
                node_name: ctx.node_name().to_string(),
                kind: kind.clone(),
                key: err.path().to_string(),
                message: err.inner().to_string().trim_end().to_string(),
            }
        })?;

        let resolved =
            toml::Table::try_from(&config).map_err(|err| FactoryError::ResolveFailed {
                node_name: ctx.node_name().to_string(),
                kind: kind.clone(),
                message: err.to_string().trim_end().to_string(),
            })?;

        let output = build(config, ctx).map_err(|err| FactoryError::BuildFailed {
            node_name: ctx.node_name().to_string(),
            kind: kind.clone(),
            reason: err,
        })?;

        Ok(BuiltNode { output, resolved })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::pipeline::node::TickContext;
    use crate::port::{AlgorithmNodePortDescriptor, InternalChannel, PortBus, PortDescriptor};

    use helios_core::prelude::TfProvider;

    use serde::Deserialize;

    const NODE: &str = "front_deproject";
    const KIND: &str = "Deproject";
    const DEFAULT_RATE: f64 = 10.0;
    const REJECTED_OUTPUT: &str = "reject_me";

    /// A node that does nothing; the adapter only passes it through.
    struct StubNode {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for StubNode {
        fn name(&self) -> &str {
            NODE
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    #[derive(Deserialize, Serialize)]
    #[serde(deny_unknown_fields)]
    struct StubConfig {
        output: String,
        #[serde(default = "default_rate")]
        rate: f64,
        #[serde(default)]
        noise: StubNoise,
    }

    #[derive(Default, Deserialize, Serialize)]
    #[serde(deny_unknown_fields)]
    struct StubNoise {
        sigma: f64,
    }

    fn default_rate() -> f64 {
        DEFAULT_RATE
    }

    /// Builds a [`StubNode`], or fails when `output` is [`REJECTED_OUTPUT`].
    fn build_stub(config: StubConfig, _ctx: &BuildContext<'_>) -> Result<FactoryOutput, String> {
        if config.output == REJECTED_OUTPUT {
            return Err(format!("output '{REJECTED_OUTPUT}' is not allowed"));
        }
        Ok(FactoryOutput::new(Box::new(StubNode {
            descriptor: AlgorithmNodePortDescriptor::new().build(),
        })))
    }

    /// Runs the erased stub factory on `section`, written as TOML.
    fn run(section: &str) -> Result<BuiltNode, FactoryError> {
        let factory = erase(KIND.to_string(), build_stub);
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("car"), NODE, &channels);
        factory(section, &ctx)
    }

    /// Unwraps the failure of [`run`]; `BuiltNode` has no `Debug`, so
    /// `unwrap_err` isn't available.
    fn run_err(section: &str) -> FactoryError {
        match run(section) {
            Ok(_) => panic!("expected the factory to fail"),
            Err(err) => err,
        }
    }

    /// A misspelled key is rejected, and the error says which key.
    #[test]
    fn unknown_key_is_rejected_with_its_path() {
        let err = run_err("output = \"cloud\"\noutptu = \"x\"");
        let FactoryError::InvalidConfig {
            node_name,
            kind,
            key,
            message,
        } = &err
        else {
            panic!("expected InvalidConfig, got {err}");
        };
        assert_eq!((node_name.as_str(), kind.as_str()), (NODE, KIND));
        assert_eq!(key, "outptu");
        assert!(message.contains("unknown field"), "{message}");
    }

    /// A misspelled key inside a nested table reports the full dotted path.
    #[test]
    fn unknown_nested_key_reports_the_dotted_path() {
        let err = run_err("output = \"cloud\"\n[noise]\nsigam = 0.1");
        let FactoryError::InvalidConfig { key, .. } = &err else {
            panic!("expected InvalidConfig, got {err}");
        };
        assert_eq!(key, "noise.sigam");
    }

    /// A missing field is reported at the section root, so the message names
    /// the node without a trailing key.
    #[test]
    fn missing_field_is_reported_at_the_section_root() {
        let err = run_err("rate = 5.0");
        assert_eq!(
            err.to_string(),
            format!("nodes.{NODE} (kind '{KIND}'): missing field `output`")
        );
    }

    /// A field the section leaves out shows up in the resolved section with
    /// its default, so the dump shows the value the node actually used.
    #[test]
    fn resolved_section_shows_defaulted_fields() {
        let Ok(built) = run("output = \"cloud\"") else {
            panic!("expected the factory to succeed");
        };
        assert_eq!(
            built.resolved.get("rate"),
            Some(&toml::Value::Float(DEFAULT_RATE))
        );
        assert_eq!(
            built.resolved.get("output"),
            Some(&toml::Value::String("cloud".into()))
        );
    }

    /// A build function's own error comes back naming the node and the kind.
    #[test]
    fn build_error_names_node_and_kind() {
        let err = run_err(&format!("output = \"{REJECTED_OUTPUT}\""));
        let FactoryError::BuildFailed {
            node_name,
            kind,
            reason,
        } = &err
        else {
            panic!("expected BuildFailed, got {err}");
        };
        assert_eq!((node_name.as_str(), kind.as_str()), (NODE, KIND));
        assert!(reason.contains(REJECTED_OUTPUT), "{reason}");
    }

    fn invalid_config(key: &str) -> FactoryError {
        FactoryError::InvalidConfig {
            node_name: "front_deproject".into(),
            kind: "Deproject".into(),
            key: key.into(),
            message: "bad value".into(),
        }
    }

    #[test]
    fn invalid_config_message_points_at_the_key() {
        assert_eq!(
            invalid_config("noise.sigma").to_string(),
            "nodes.front_deproject.noise.sigma (kind 'Deproject'): bad value"
        );
    }

    #[test]
    fn invalid_config_at_the_section_root_omits_the_key() {
        let expected = "nodes.front_deproject (kind 'Deproject'): bad value";
        assert_eq!(invalid_config(".").to_string(), expected);
        assert_eq!(invalid_config("").to_string(), expected);
    }

    #[test]
    fn build_failure_message_names_node_kind_and_reason() {
        let err = FactoryError::BuildFailed {
            node_name: "front_deproject".into(),
            kind: "Deproject".into(),
            reason: "unsupported direction model".into(),
        };
        assert_eq!(
            err.to_string(),
            "nodes.front_deproject (kind 'Deproject'): build failed: unsupported direction model"
        );
    }

    #[test]
    fn context_accessors_return_what_it_was_built_with() {
        let channels: HashSet<String> = ["front_lidar".to_string()].into();
        let ctx = BuildContext::new(AgentId::new("car"), "front_deproject", &channels);
        assert_eq!(ctx.agent(), &AgentId::new("car"));
        assert_eq!(ctx.node_name(), "front_deproject");
        assert!(ctx.sensor_channels().contains("front_lidar"));
    }

    /// An output declares no outside inputs until the factory adds them, and
    /// keeps them in the order they were added.
    #[test]
    fn outside_inputs_are_kept_in_declaration_order() {
        let stub = || -> Box<dyn PipelineNode> {
            Box::new(StubNode {
                descriptor: AlgorithmNodePortDescriptor::new().build(),
            })
        };
        assert!(FactoryOutput::new(stub()).outside_inputs().is_empty());

        let mission = InternalChannel::named::<f64>("mission");
        let waypoints = InternalChannel::named::<f64>("waypoints");
        let output = FactoryOutput::new(stub())
            .with_outside_input(mission.clone())
            .with_outside_input(waypoints.clone());
        assert_eq!(
            output.outside_inputs(),
            [ChannelKey::from(mission), ChannelKey::from(waypoints)]
        );
    }
}
