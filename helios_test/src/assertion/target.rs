use helios_core::prelude::AgentId;
use helios_runtime::{
    observe::path::{agent_path, channel_to_path_segment, AgentPathParts, PathPart},
    AutonomyPipeline, ChannelKey,
};

use serde::Deserialize;
use std::{collections::HashMap, fmt, sync::Arc};

/// _Shared (pre + per)._ The watcher path an assertion watches,
/// `agent.<agent>.<node>.<leaf>`, held whole and in its parts.
///
/// Only a path that splits into an agent, a node and a leaf becomes a
/// target, so a run file naming anything else is refused when it is read,
/// before a sim starts.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Deserialize)]
#[serde(try_from = "String")]
pub struct AssertionTarget {
    path: String,
    agent: AgentId,
    node: Arc<str>,
    leaf: Arc<str>,
}

impl AssertionTarget {
    /// The target `path` names, or why it names none.
    pub fn parse(path: impl Into<String>) -> Result<Self, MalformedTarget> {
        let path = path.into();
        let parts = match AgentPathParts::split(&path) {
            Ok(parts) => parts,
            Err(part) => return Err(MalformedTarget { path, part }),
        };
        let (agent, node, leaf) = (
            AgentId::new(parts.agent),
            parts.node.into(),
            parts.rest.into(),
        );

        Ok(Self {
            path,
            agent,
            node,
            leaf,
        })
    }

    pub fn as_str(&self) -> &str {
        &self.path
    }

    pub fn agent(&self) -> &AgentId {
        &self.agent
    }

    pub fn node(&self) -> &Arc<str> {
        &self.node
    }

    /// The leaf or channel segment, dots included.
    pub fn leaf(&self) -> &Arc<str> {
        &self.leaf
    }
}

impl TryFrom<String> for AssertionTarget {
    type Error = MalformedTarget;

    fn try_from(path: String) -> Result<Self, Self::Error> {
        Self::parse(path)
    }
}

impl fmt::Display for AssertionTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.path)
    }
}

/// _Pre-step._ A run-file target that doesn't split into an agent, a node
/// and a leaf, and the first part that is wrong.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error(
    "target '{path}' is not a watcher path agent.<agent>.<node>.<leaf>: {}",
    what_is_wrong(*.part)
)]
pub struct MalformedTarget {
    path: String,
    part: PathPart,
}

/// What is wrong with a target whose `part` is the first to fail.
fn what_is_wrong(part: PathPart) -> &'static str {
    match part {
        PathPart::Root => "it starts with something else",
        PathPart::Agent => "the agent is missing",
        PathPart::Node => "the node is missing",
        PathPart::Rest => "the leaf is missing",
    }
}

/// _Per-step._ Resolves an assertion target to a bus channel; built at
/// pipeline-built time, queried each tick.
// Leaf-lookup map: TOML target string → bus channel for one agent. Populated
// once per agent at `on_pipeline_built`; queried once per assertion per tick.
pub struct TargetRegistry {
    by_target: HashMap<(AgentId, AssertionTarget), ChannelKey>,
}

impl TargetRegistry {
    pub fn new() -> Self {
        TargetRegistry {
            by_target: HashMap::new(),
        }
    }

    // Last-writer-wins. Same justification as `ExtractorTable::register` —
    // lets tests stage fixture entries over a built-up registry without a
    // separate "override" entry point.
    pub fn register(&mut self, agent: AgentId, target: AssertionTarget, channel: ChannelKey) {
        self.by_target.insert((agent, target), channel);
    }

    // The `clone()` pair is the price of a tuple `HashMap` key — `get` needs
    // a `&(AgentId, AssertionTarget)`, and `(&AgentId, &AssertionTarget)`
    // doesn't satisfy `Borrow`. Two small string clones per assertion per
    // tick is well below anything worth restructuring the map for; revisit
    // if a profile ever flags it.
    pub fn resolve(&self, agent: &AgentId, target: &AssertionTarget) -> Option<&ChannelKey> {
        self.by_target.get(&(agent.clone(), target.clone()))
    }
}

impl Default for TargetRegistry {
    fn default() -> Self {
        Self::new()
    }
}

pub fn build_for_pipeline(
    agent: &AgentId,
    pipeline: &AutonomyPipeline,
) -> Vec<(AssertionTarget, ChannelKey)> {
    let mut pairs: Vec<(AssertionTarget, ChannelKey)> = pipeline
        .channels()
        // A channel whose path doesn't parse is left out: no target can
        // name it, since every target parses.
        .filter_map(|(node_name, key)| {
            let path = agent_path(agent.as_str(), node_name, &channel_to_path_segment(key));
            let target = AssertionTarget::parse(path).ok()?;
            Some((target, key.clone()))
        })
        .collect();

    // `channels()` already yields a deterministic topological order, but we
    // re-sort alphabetically by target path: these are human-referenced
    // strings, so alphabetical order makes registry dumps and any future
    // diagnostic output scannable and diffable, independent of pipeline build
    // order. Ordering doesn't affect `TargetRegistry` lookups (a `HashMap`);
    // it's purely for the listing.
    pairs.sort_by(|a, b| a.0.as_str().cmp(b.0.as_str()));
    pairs
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_runtime::port::InternalChannel;

    fn channel_unnamed<T: 'static>() -> ChannelKey {
        InternalChannel::of::<T>().into()
    }

    #[test]
    fn parse_keeps_the_path_and_its_parts() {
        let target =
            AssertionTarget::parse("agent.car.estimator.aiding.gps.nis").expect("valid target");

        assert_eq!(target.as_str(), "agent.car.estimator.aiding.gps.nis");
        assert_eq!(target.agent(), &AgentId::new("car"));
        assert_eq!(target.node().as_ref(), "estimator");
        assert_eq!(target.leaf().as_ref(), "aiding.gps.nis");
    }

    #[test]
    fn parse_refuses_a_path_with_no_leaf() {
        assert_eq!(
            AssertionTarget::parse("agent.car.estimator"),
            Err(MalformedTarget {
                path: "agent.car.estimator".to_owned(),
                part: PathPart::Rest,
            })
        );
    }

    #[test]
    fn a_malformed_target_says_which_part_is_wrong() {
        let error = AssertionTarget::parse("agent.car").expect_err("no node or leaf");

        assert!(
            error.to_string().ends_with("the node is missing"),
            "{error}"
        );
    }

    #[test]
    fn parse_refuses_a_path_outside_the_agent_root() {
        assert!(AssertionTarget::parse("car.estimator.speed.x").is_err());
    }

    #[test]
    fn register_and_resolve_roundtrip() {
        let mut reg = TargetRegistry::new();
        let agent = AgentId::new("car_1");
        let target =
            AssertionTarget::parse("agent.car_1.estimator.position").expect("valid target");
        let channel = channel_unnamed::<f64>();

        reg.register(agent.clone(), target.clone(), channel.clone());

        assert_eq!(reg.resolve(&agent, &target), Some(&channel));
    }

    #[test]
    fn resolve_returns_none_for_missing_pair() {
        let reg = TargetRegistry::new();
        let agent = AgentId::new("car_1");
        let target = AssertionTarget::parse("agent.car_1.estimator.unknown").expect("valid target");

        assert_eq!(reg.resolve(&agent, &target), None);
    }

    #[test]
    fn resolve_isolates_by_agent() {
        let mut reg = TargetRegistry::new();
        let agent_a = AgentId::new("car_a");
        let agent_b = AgentId::new("car_b");
        let target = AssertionTarget::parse("agent.shared.estimator.scalar").expect("valid target");
        let channel = channel_unnamed::<f64>();

        reg.register(agent_a.clone(), target.clone(), channel);

        assert!(reg.resolve(&agent_a, &target).is_some());
        assert!(reg.resolve(&agent_b, &target).is_none());
    }

    #[test]
    fn register_overrides_previous_entry() {
        let mut reg = TargetRegistry::new();
        let agent = AgentId::new("car_1");
        let target = AssertionTarget::parse("agent.car_1.estimator.scalar").expect("valid target");
        let first = channel_unnamed::<f64>();
        let second = channel_unnamed::<i64>();

        reg.register(agent.clone(), target.clone(), first);
        reg.register(agent.clone(), target.clone(), second.clone());

        assert_eq!(reg.resolve(&agent, &target), Some(&second));
    }

    #[test]
    fn default_is_empty() {
        let reg = TargetRegistry::default();
        let agent = AgentId::new("x");
        let target = AssertionTarget::parse("agent.x.estimator.y").expect("valid target");
        assert!(reg.resolve(&agent, &target).is_none());
    }
}
