use std::collections::HashMap;

use helios_runtime::{AutonomyPipeline, ChannelKey};
use serde::Deserialize;

/// _Shared (pre + per)._ A symbolic subsystem path naming what an assertion
/// watches. Parsed from the run file, then used as a per-step registry key.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Deserialize)]
pub struct AssertionTarget(String);

impl AssertionTarget {
    pub fn new(s: impl Into<String>) -> Self {
        Self(s.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// _Per-step._ Identifies one agent at runtime; supplied by the host, used as a
/// registry key. Not parsed from the run file.
// TODO: move to `helios_runtime` once multi-agent scheduling lands there.
// Lives in `helios_test` for now so we don't churn the runtime crate before
// it has a real consumer.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Deserialize)]
pub struct AgentId(String);

impl AgentId {
    pub fn new(s: impl Into<String>) -> Self {
        Self(s.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
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
        .map(|(node_name, key)| {
            let target_str = format!(
                "agent.{}.{}.{}",
                agent.as_str(),
                node_name,
                channel_to_path_segment(key)
            );
            (AssertionTarget::new(target_str), key.clone())
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

// Maps a `ChannelKey` to one dot-path segment used inside the target string.
// Prefers the channel's instance disambiguator (e.g. `oracle/pose`) because
// that's the name humans wrote in the config; falls back to the type name
// when the instance is empty (the unnamed/default channel case).
//
// `/` is rewritten to `_` so the segment stays a single dot-path token —
// `agent.car.mapper.oracle/pose` would otherwise split into two segments
// downstream.
fn channel_to_path_segment(key: &ChannelKey) -> String {
    let raw = key.instance();
    if !raw.trim().is_empty() {
        raw.replace('/', "_")
    } else {
        to_snake_case(last_segment(key.type_name()))
    }
}

fn last_segment(type_name: &'static str) -> &'static str {
    type_name.rsplit("::").next().unwrap_or(type_name)
}

// Lowercase + underscore-separated. `FrameAwareState` → `frame_aware_state`,
// `f64` → `f64`. Sufficient for Rust type names; not a general-purpose
// snake_case (no digit-boundary handling, no acronym preservation). If a
// type name with quirks shows up, switch to the `heck` crate.
fn to_snake_case(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 4);
    for (i, ch) in s.chars().enumerate() {
        if i > 0 && ch.is_uppercase() {
            out.push('_');
        }
        out.extend(ch.to_lowercase());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_runtime::port::InternalChannel;

    fn channel_named<T: 'static>(instance: &'static str) -> ChannelKey {
        InternalChannel::named::<T>(instance).into()
    }

    fn channel_unnamed<T: 'static>() -> ChannelKey {
        InternalChannel::of::<T>().into()
    }

    #[test]
    fn register_and_resolve_roundtrip() {
        let mut reg = TargetRegistry::new();
        let agent = AgentId::new("car_1");
        let target = AssertionTarget::new("agent.car_1.estimator.position");
        let channel = channel_unnamed::<f64>();

        reg.register(agent.clone(), target.clone(), channel.clone());

        assert_eq!(reg.resolve(&agent, &target), Some(&channel));
    }

    #[test]
    fn resolve_returns_none_for_missing_pair() {
        let reg = TargetRegistry::new();
        let agent = AgentId::new("car_1");
        let target = AssertionTarget::new("agent.car_1.unknown");

        assert_eq!(reg.resolve(&agent, &target), None);
    }

    #[test]
    fn resolve_isolates_by_agent() {
        let mut reg = TargetRegistry::new();
        let agent_a = AgentId::new("car_a");
        let agent_b = AgentId::new("car_b");
        let target = AssertionTarget::new("agent.shared.scalar");
        let channel = channel_unnamed::<f64>();

        reg.register(agent_a.clone(), target.clone(), channel);

        assert!(reg.resolve(&agent_a, &target).is_some());
        assert!(reg.resolve(&agent_b, &target).is_none());
    }

    #[test]
    fn register_overrides_previous_entry() {
        let mut reg = TargetRegistry::new();
        let agent = AgentId::new("car_1");
        let target = AssertionTarget::new("agent.car_1.scalar");
        let first = channel_unnamed::<f64>();
        let second = channel_unnamed::<i64>();

        reg.register(agent.clone(), target.clone(), first);
        reg.register(agent.clone(), target.clone(), second.clone());

        assert_eq!(reg.resolve(&agent, &target), Some(&second));
    }

    #[test]
    fn channel_to_path_segment_uses_instance() {
        let key = channel_named::<f64>("oracle/pose");
        assert_eq!(channel_to_path_segment(&key), "oracle_pose");
    }

    #[test]
    fn channel_to_path_segment_replaces_all_slashes() {
        let key = channel_named::<f64>("debug/inner/value");
        assert_eq!(channel_to_path_segment(&key), "debug_inner_value");
    }

    #[test]
    fn channel_to_path_segment_falls_back_to_type_name() {
        let key = channel_unnamed::<f64>();
        assert_eq!(channel_to_path_segment(&key), "f64");
    }

    #[test]
    fn last_segment_strips_module_path() {
        assert_eq!(last_segment("foo::bar::Baz"), "Baz");
        assert_eq!(last_segment("Baz"), "Baz");
        assert_eq!(last_segment("f64"), "f64");
    }

    #[test]
    fn to_snake_case_basic_cases() {
        assert_eq!(to_snake_case("FrameAwareState"), "frame_aware_state");
        assert_eq!(to_snake_case("f64"), "f64");
        assert_eq!(to_snake_case("A"), "a");
        assert_eq!(to_snake_case(""), "");
        assert_eq!(to_snake_case("MyType"), "my_type");
    }

    #[test]
    fn default_is_empty() {
        let reg = TargetRegistry::default();
        let agent = AgentId::new("x");
        let target = AssertionTarget::new("y");
        assert!(reg.resolve(&agent, &target).is_none());
    }
}
