//! Turning what sinks requested into what each pipeline watches.
//!
//! Sinks name full paths across every agent; a pipeline watches `(node,
//! leaf)` pairs and doesn't know its agent. A host resolves between the two
//! by formatting every path each pipeline can report and looking its
//! requests up among them. Paths are only ever formatted, never split apart:
//! a leaf may hold dots, so a path can't be split back into node and leaf.

use crate::{
    observe::{
        path::agent_path,
        sink::WatchRequest,
        watch::{WatchSet, WatchedLeaf},
    },
    port::Observable,
};

use helios_core::prelude::AgentId;

use std::collections::BTreeSet;

/// Resolves one request against every agent a host runs, one agent at a time.
///
/// A requested path is unresolved only once every agent has had its turn, so
/// the resolver carries what is still unmatched from one agent to the next.
/// Each path names one agent, one node and one leaf, so it matches at most
/// once, and matching removes it; what remains at the end matched nothing.
#[derive(Debug)]
pub struct WatchResolver {
    /// The merged request; for [`WatchRequest::Paths`], the paths no agent
    /// has matched yet.
    pending: WatchRequest,
}

impl WatchResolver {
    pub fn new(request: WatchRequest) -> Self {
        Self { pending: request }
    }

    /// The watch set for `agent`, whose pipeline reports `observables`, as
    /// [`observables`](crate::AutonomyPipeline::observables) lists them.
    pub fn watch_set_for<'a>(
        &mut self,
        agent: &AgentId,
        observables: impl IntoIterator<Item = (&'a str, Observable)>,
    ) -> WatchSet {
        let pending = match &mut self.pending {
            WatchRequest::Everything => return WatchSet::Everything,
            WatchRequest::Paths(pending) => pending,
        };

        let leaves: BTreeSet<WatchedLeaf> = observables
            .into_iter()
            .filter(|(node, observable)| {
                pending.remove(&agent_path(agent.as_str(), node, observable.leaf_name()))
            })
            .map(|(node, observable)| WatchedLeaf::new(node, observable.leaf_name().as_ref()))
            .collect();

        WatchSet::Leaves(leaves)
    }

    /// The requested paths no agent reports. The caller must report them
    /// loudly: an unresolved path watches nothing, so whatever reads it
    /// sees no samples rather than an error.
    pub fn into_unresolved(self) -> BTreeSet<String> {
        match self.pending {
            WatchRequest::Everything => BTreeSet::new(),
            WatchRequest::Paths(pending) => pending,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::Determinism;

    const CAR: &str = "car";
    const TRUCK: &str = "truck";
    const ESTIMATOR: &str = "estimator";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";

    /// What an agent's pipeline reports: one estimator with NIS and drops.
    fn observables() -> Vec<(&'static str, Observable)> {
        [NIS, DROPPED]
            .into_iter()
            .map(|leaf| (ESTIMATOR, Observable::new(leaf, Determinism::Reproducible)))
            .collect()
    }

    fn request(paths: &[String]) -> WatchRequest {
        WatchRequest::Paths(paths.iter().cloned().collect())
    }

    fn leaves(pairs: &[(&str, &str)]) -> WatchSet {
        WatchSet::Leaves(
            pairs
                .iter()
                .map(|(node, leaf)| WatchedLeaf::new(*node, *leaf))
                .collect(),
        )
    }

    fn unresolved(paths: &[&String]) -> BTreeSet<String> {
        paths.iter().map(|path| path.to_string()).collect()
    }

    #[test]
    fn requested_paths_resolve_to_their_pairs() {
        let mut resolver = WatchResolver::new(request(&[agent_path(CAR, ESTIMATOR, NIS)]));

        let set = resolver.watch_set_for(&AgentId::new(CAR), observables());

        assert_eq!(set, leaves(&[(ESTIMATOR, NIS)]));
        assert!(resolver.into_unresolved().is_empty());
    }

    #[test]
    fn a_path_for_another_agent_is_left_to_that_agent() {
        let mut resolver = WatchResolver::new(request(&[
            agent_path(CAR, ESTIMATOR, NIS),
            agent_path(TRUCK, ESTIMATOR, DROPPED),
        ]));

        let car = resolver.watch_set_for(&AgentId::new(CAR), observables());
        let truck = resolver.watch_set_for(&AgentId::new(TRUCK), observables());

        assert_eq!(car, leaves(&[(ESTIMATOR, NIS)]));
        assert_eq!(truck, leaves(&[(ESTIMATOR, DROPPED)]));
        assert!(resolver.into_unresolved().is_empty());
    }

    #[test]
    fn unknown_agent_node_and_leaf_are_returned_unresolved() {
        let unknown_agent = agent_path("bus", ESTIMATOR, NIS);
        let unknown_node = agent_path(CAR, "planner", NIS);
        let undeclared_leaf = agent_path(CAR, ESTIMATOR, "aiding.gps.nsi");
        let mut resolver = WatchResolver::new(request(&[
            agent_path(CAR, ESTIMATOR, NIS),
            unknown_agent.clone(),
            unknown_node.clone(),
            undeclared_leaf.clone(),
        ]));

        let set = resolver.watch_set_for(&AgentId::new(CAR), observables());

        assert_eq!(set, leaves(&[(ESTIMATOR, NIS)]));
        assert_eq!(
            resolver.into_unresolved(),
            unresolved(&[&unknown_agent, &unknown_node, &undeclared_leaf])
        );
    }

    #[test]
    fn everything_watches_everything_and_leaves_nothing_unresolved() {
        let mut resolver = WatchResolver::new(WatchRequest::Everything);

        let set = resolver.watch_set_for(&AgentId::new(CAR), observables());

        assert_eq!(set, WatchSet::Everything);
        assert!(resolver.into_unresolved().is_empty());
    }

    #[test]
    fn no_request_watches_nothing() {
        let mut resolver = WatchResolver::new(WatchRequest::default());

        let set = resolver.watch_set_for(&AgentId::new(CAR), observables());

        assert_eq!(set, WatchSet::default());
        assert!(resolver.into_unresolved().is_empty());
    }
}
