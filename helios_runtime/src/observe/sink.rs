//! The contract every consumer of observations codes against: tests,
//! recorders, visualizers, bridges.
//!
//! A sink says once what it wants, then receives each tick's batch. The
//! runtime holds no sink and calls none: a host asks each of its sinks for
//! its request before the first tick, and hands each the batch after every
//! drain. A sink that only consumes data needs nothing from the host, so the
//! same sink runs in simulation and on hardware.

use crate::observe::observation::AgentObservation;

use std::collections::BTreeSet;

/// A consumer of observations.
///
/// No `Send` or `Sync` bound: a host that keeps sinks across threads adds the
/// bounds where it names the sink type.
pub trait ObservationSink {
    /// What this sink wants recorded. The host asks once, before the first
    /// tick; a request that changes later is not seen.
    fn request(&self) -> WatchRequest;

    /// One tick's observations, from every agent the host runs, possibly
    /// none. Only what some sink requested is recorded, so the batch may hold
    /// more than this sink asked for when several sinks share a host.
    fn receive(&mut self, batch: &[AgentObservation]);
}

/// What a sink asks to have recorded, named by full watcher path across
/// every agent.
///
/// The default is an empty set of paths, which asks for nothing. Unlike a
/// [`WatchSet`](crate::observe::watch::WatchSet), which names `(node, leaf)`
/// pairs inside one pipeline, a request names the agent too, in the spelling
/// a person writes.
#[derive(Debug, Clone, PartialEq)]
pub enum WatchRequest {
    /// Every leaf of every node of every agent.
    Everything,
    /// Exactly these paths, each `agent.<agent>.<node>.<leaf>`. Held in
    /// order, so the same path named twice is kept once.
    Paths(BTreeSet<String>),
}

impl WatchRequest {
    /// Folds `other` into this request, so one request covers what both
    /// ask for: [`Everything`](Self::Everything) on either side wins,
    /// otherwise the paths are joined.
    pub fn add(&mut self, other: WatchRequest) {
        match other {
            WatchRequest::Everything => *self = WatchRequest::Everything,
            WatchRequest::Paths(theirs) => {
                if let WatchRequest::Paths(mine) = self {
                    mine.extend(theirs);
                }
            }
        }
    }
}

impl Default for WatchRequest {
    fn default() -> Self {
        Self::Paths(BTreeSet::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NIS: &str = "agent.car.estimator.aiding.gps.nis";
    const DROPPED: &str = "agent.car.estimator.aiding.gps.dropped";

    fn paths(paths: &[&str]) -> WatchRequest {
        WatchRequest::Paths(paths.iter().map(|path| path.to_string()).collect())
    }

    #[test]
    fn adding_paths_joins_them() {
        let mut request = paths(&[NIS]);
        request.add(paths(&[NIS, DROPPED]));

        assert_eq!(request, paths(&[NIS, DROPPED]));
    }

    #[test]
    fn adding_everything_to_paths_gives_everything() {
        let mut request = paths(&[NIS]);
        request.add(WatchRequest::Everything);

        assert_eq!(request, WatchRequest::Everything);
    }

    #[test]
    fn adding_paths_to_everything_keeps_everything() {
        let mut request = WatchRequest::Everything;
        request.add(paths(&[NIS]));

        assert_eq!(request, WatchRequest::Everything);
    }

    #[test]
    fn adding_to_the_default_gives_the_other_request() {
        let mut request = WatchRequest::default();
        request.add(paths(&[NIS]));

        assert_eq!(request, paths(&[NIS]));
    }
}
