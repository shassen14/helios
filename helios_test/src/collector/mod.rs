//! The sink a run's assertions read from: every value the brain reports for
//! a run-file target, kept as a series per target.
//!
//! The runner never reads a pipeline's bus. It asks the host, through the
//! sink's request, to record each key setup bound, and judges what arrives
//! here. So the same runner works wherever observations can be delivered: a
//! sim, a robot, a recording.

mod evaluations;
mod series;
mod unbound;

#[cfg(test)]
mod fixtures;

pub use evaluations::EvaluationId;
pub use series::{Rejection, Sample, Series, SeriesId};
pub use unbound::UnboundArrivals;

use crate::assertion::{
    extract::{Extracted, ExtractorTable},
    AssertionTarget,
};
use crate::evaluation::kind::Evaluator;

use helios_runtime::observe::{
    key::{RecordedKey, Subject},
    observation::Observation,
    sink::{ObservationSink, WatchRequest},
};

use std::{
    any::TypeId,
    collections::{HashMap, HashSet},
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};

/// The id the next collector built takes. Counts up for the life of the
/// process, so no two collectors share one.
static NEXT_COLLECTOR: AtomicU64 = AtomicU64::new(0);

/// _Per-step._ Collects, per recorded key, every value the brain reports
/// under it, turned into an [`AssertionValue`](crate::assertion::AssertionValue)
/// on arrival.
///
/// Built empty. Setup binds each series to one exact key, from what the
/// pipelines declare, with the selector it resolved the key from, so an
/// arrival is matched on its whole key (agent, channel, payload type and
/// instance). Every instance of a keyed channel has a series of its own.
/// The host is asked for the bound keys' paths, so it records exactly what
/// setup bound, and only once setup has bound it.
///
/// An arrival a bound selector covers but no bound key matches is
/// dropped and counted. Setup binds every key a judged assertion names, so
/// in a run that can judge, one means the pipeline recorded under a key its
/// catalog never declared, or delivered before setup bound it; the run fails
/// on it rather than judging a series that silently missed values. A
/// selector naming one instance covers only that one: a sibling instance on
/// the same path is no value its series missed.
///
/// A value is extracted the moment it arrives and the observation is then
/// dropped, so a channel carrying a large payload costs what a scalar does:
/// the series holds the extracted value, never the payload.
///
/// It also feeds each evaluation bound to it: a value under one of an
/// evaluation's input keys is handed to it bare, and the evaluation keeps
/// what it reads of it. One key may feed a series and evaluations both.
pub struct ObservationCollector {
    /// Stamped into every [`SeriesId`] this collector hands out, so an id is
    /// never read against another run's collector.
    id: CollectorId,
    /// The selectors setup resolved its bound keys from.
    covered: HashSet<AssertionTarget>,
    /// Each bound key's index into `series`.
    by_key: HashMap<RecordedKey, usize>,
    series: Vec<Series>,
    /// How many values arrived under each key no series is bound to that a
    /// bound selector covers.
    unbound: HashMap<RecordedKey, u64>,
    /// Keys no bound selector covers: what another sink asked for,
    /// which every sink is handed, or a sibling of a selected instance.
    /// Held so each key's path is spelled once, not on every arrival.
    unrequested: HashSet<RecordedKey>,
    extractors: ExtractorTable,
    evaluations: Vec<Box<dyn Evaluator>>,
    /// Each input key's evaluations, by index, with the role it fills.
    inputs: HashMap<RecordedKey, Vec<(usize, usize)>>,
    /// Whether the host has asked for its request. Setup refuses a
    /// collector already asked, which would record nothing it binds.
    /// Atomic, since the host asks through `&self` and a sink is `Sync`.
    asked: AtomicBool,
}

impl ObservationCollector {
    /// A collector reading channel values through `extractors`, requesting
    /// nothing until setup binds keys.
    pub fn new(extractors: ExtractorTable) -> Self {
        Self {
            id: CollectorId(NEXT_COLLECTOR.fetch_add(1, Ordering::Relaxed)),
            covered: HashSet::new(),
            by_key: HashMap::new(),
            series: Vec::new(),
            unbound: HashMap::new(),
            unrequested: HashSet::new(),
            extractors,
            evaluations: Vec::new(),
            inputs: HashMap::new(),
            asked: AtomicBool::new(false),
        }
    }

    /// Whether the host has asked for its request.
    pub fn was_asked(&self) -> bool {
        self.asked.load(Ordering::Relaxed)
    }

    /// What a channel value whose payload is of type `payload` turns into:
    /// the kind of [`AssertionValue`](crate::assertion::AssertionValue) and
    /// its unit, or `None` when it can't be turned into one.
    pub fn extracted(&self, payload: TypeId) -> Option<Extracted> {
        self.extractors.extracted(payload)
    }
}

impl ObservationSink for ObservationCollector {
    /// Each bound key's path, a series' or an evaluation input's, spelled
    /// as the host matches it. A channel path watches every instance; the
    /// bound keys decide which of them are kept.
    fn request(&self) -> WatchRequest {
        self.asked.store(true, Ordering::Relaxed);
        WatchRequest::Paths(
            self.by_key
                .keys()
                .chain(self.inputs.keys())
                .map(RecordedKey::path)
                .collect(),
        )
    }

    fn receive(&mut self, batch: &[Observation]) {
        for observation in batch {
            let recorded = self.record(observation);
            let fed = self.feed(observation);
            if !recorded && !fed {
                self.count_unbound(&observation.key);
            }
        }
    }
}

/// _Per-step._ Which collector a [`SeriesId`] or an [`EvaluationId`] came
/// from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct CollectorId(u64);

/// The spelling of the instance `subject` records: `None` for an emission or
/// a channel that isn't keyed.
pub fn instance_of(subject: &Subject) -> Option<&str> {
    match subject {
        Subject::Channel {
            instance: Some(instance),
            ..
        } => Some(instance.as_str()),
        Subject::Channel { instance: None, .. } | Subject::Emission { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        fixtures::{channel_target, collector, emission_key, key, speed, target, CAR, NIS},
        *,
    };

    #[test]
    fn the_request_is_each_bound_keys_path_once() {
        // Two selectors on NIS bind one key; the speed's key spells its
        // channel's path.
        let speed_key = key(CAR, Subject::channel(speed::<f64>(), None));
        let mut collector = collector();
        collector.bind(&target(CAR, NIS), emission_key(CAR, NIS));
        collector.bind(&target(CAR, NIS), emission_key(CAR, NIS));
        collector.bind(&channel_target(CAR, &speed::<f64>()), speed_key.clone());

        assert_eq!(
            collector.request(),
            WatchRequest::Paths(
                [emission_key(CAR, NIS).path(), speed_key.path()]
                    .into_iter()
                    .collect()
            )
        );
    }

    #[test]
    fn a_collector_nothing_is_bound_in_requests_nothing() {
        assert_eq!(
            collector().request(),
            WatchRequest::Paths(Default::default())
        );
    }

    #[test]
    fn asking_for_the_request_is_noted() {
        let collector = collector();
        assert!(!collector.was_asked());

        collector.request();

        assert!(collector.was_asked());
    }
}
