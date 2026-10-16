//! Turning what sinks requested into what each pipeline watches.
//!
//! Sinks name full paths across every agent; a pipeline watches typed
//! [`Subject`]s of its own agent. A host resolves between the two by spelling
//! every subject each pipeline can record and looking its requests up among
//! them, once per build.

use crate::{
    observe::{key::Subject, sink::WatchRequest, watch::WatchSet},
    port::Observable,
};

use helios_core::prelude::AgentId;

use std::collections::BTreeSet;

/// Resolves one request against every agent a host runs, one agent at a time.
///
/// A requested path is unresolved only once every agent has had its turn, so
/// the resolver carries what is still unmatched from one agent to the next.
/// Each path names one agent, so it matches only during that agent's turn,
/// and matching removes it; what remains at the end matched nothing. Within
/// the turn a path may match several subjects: a channel path matches every
/// instance of its channel.
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
    pub fn watch_set_for(
        &mut self,
        agent: &AgentId,
        observables: impl IntoIterator<Item = (Subject, Observable)>,
    ) -> WatchSet {
        let pending = match &mut self.pending {
            WatchRequest::Everything => return WatchSet::Everything,
            WatchRequest::Paths(pending) => pending,
        };

        let mut matched: BTreeSet<String> = BTreeSet::new();
        let subjects: BTreeSet<Subject> = observables
            .into_iter()
            .filter_map(|(subject, _)| {
                let path = subject.path(agent);
                if !pending.contains(&path) {
                    return None;
                }
                matched.insert(path);
                Some(subject)
            })
            .collect();
        pending.retain(|path| !matched.contains(path));

        WatchSet::Subjects(subjects)
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

    use crate::{
        observe::path::{channel_path, node_path},
        port::{ChannelKey, ChannelKind, Determinism, InstanceKey, Keyed},
    };

    const CAR: &str = "car";
    const TRUCK: &str = "truck";
    const ESTIMATOR: &str = "estimator";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    const TF: &str = "tf";

    struct Edge(&'static str);

    impl Keyed for Edge {
        type Instance = &'static str;

        fn instance(&self) -> &&'static str {
            &self.0
        }
    }

    fn tf() -> ChannelKey {
        ChannelKey::keyed::<Edge>(ChannelKind::Internal, TF)
    }

    fn edge(frame: &'static str) -> Subject {
        Subject::channel(tf(), Some(InstanceKey::of::<Edge>(&frame)))
    }

    /// What an agent's pipeline reports: one estimator with NIS and drops,
    /// and two instances of a keyed channel.
    fn observables() -> Vec<(Subject, Observable)> {
        let emissions = [NIS, DROPPED].into_iter().map(|leaf| {
            (
                Subject::emission(ESTIMATOR, leaf),
                Observable::scalar(leaf, Determinism::Deterministic),
            )
        });
        let edges = ["base_link", "imu"].into_iter().map(|frame| {
            (
                edge(frame),
                Observable::typed(TF, Determinism::Deterministic, &tf()),
            )
        });
        emissions.chain(edges).collect()
    }

    fn request(paths: &[String]) -> WatchRequest {
        WatchRequest::Paths(paths.iter().cloned().collect())
    }

    fn subjects(subjects: impl IntoIterator<Item = Subject>) -> WatchSet {
        WatchSet::Subjects(subjects.into_iter().collect())
    }

    fn unresolved(paths: &[&String]) -> BTreeSet<String> {
        paths.iter().map(|path| path.to_string()).collect()
    }

    #[test]
    fn requested_paths_resolve_to_their_subjects() {
        let mut resolver = WatchResolver::new(request(&[node_path(CAR, ESTIMATOR, NIS)]));

        let set = resolver.watch_set_for(&AgentId::new(CAR), observables());

        assert_eq!(set, subjects([Subject::emission(ESTIMATOR, NIS)]));
        assert!(resolver.into_unresolved().is_empty());
    }

    #[test]
    fn a_channel_path_resolves_to_every_instance_of_its_channel() {
        let mut resolver = WatchResolver::new(request(&[channel_path(CAR, &tf())]));

        let set = resolver.watch_set_for(&AgentId::new(CAR), observables());

        assert_eq!(set, subjects([edge("base_link"), edge("imu")]));
        assert!(resolver.into_unresolved().is_empty());
    }

    #[test]
    fn a_path_for_another_agent_is_left_to_that_agent() {
        let mut resolver = WatchResolver::new(request(&[
            node_path(CAR, ESTIMATOR, NIS),
            node_path(TRUCK, ESTIMATOR, DROPPED),
        ]));

        let car = resolver.watch_set_for(&AgentId::new(CAR), observables());
        let truck = resolver.watch_set_for(&AgentId::new(TRUCK), observables());

        assert_eq!(car, subjects([Subject::emission(ESTIMATOR, NIS)]));
        assert_eq!(truck, subjects([Subject::emission(ESTIMATOR, DROPPED)]));
        assert!(resolver.into_unresolved().is_empty());
    }

    #[test]
    fn unknown_agent_node_leaf_and_channel_are_returned_unresolved() {
        let unknown_agent = node_path("bus", ESTIMATOR, NIS);
        let unknown_node = node_path(CAR, "planner", NIS);
        let undeclared_leaf = node_path(CAR, ESTIMATOR, "aiding.gps.nsi");
        let unwritten_channel = channel_path(CAR, &ChannelKey::internal::<f64>("truth.tf"));
        let mut resolver = WatchResolver::new(request(&[
            node_path(CAR, ESTIMATOR, NIS),
            unknown_agent.clone(),
            unknown_node.clone(),
            undeclared_leaf.clone(),
            unwritten_channel.clone(),
        ]));

        let set = resolver.watch_set_for(&AgentId::new(CAR), observables());

        assert_eq!(set, subjects([Subject::emission(ESTIMATOR, NIS)]));
        assert_eq!(
            resolver.into_unresolved(),
            unresolved(&[
                &unknown_agent,
                &unknown_node,
                &undeclared_leaf,
                &unwritten_channel
            ])
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
