//! The sink a run's assertions read from: every value the brain reports for
//! a run-file target, kept as a series per target.
//!
//! The runner never reads a pipeline's bus. It asks the host, through the
//! sink's request, to record each target, and judges what arrives here. So
//! the same runner works wherever observations can be delivered: a sim, a
//! robot, a recording.

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
    sync::atomic::{AtomicU64, Ordering},
};

/// The id the next collector built takes. Counts up for the life of the
/// process, so no two collectors share one.
static NEXT_COLLECTOR: AtomicU64 = AtomicU64::new(0);

/// _Per-step._ Collects, per recorded key, every value the brain reports
/// under it, turned into an [`AssertionValue`](crate::assertion::AssertionValue)
/// on arrival.
///
/// Built requesting the targets' paths; each series is bound to one exact
/// key at setup, from what the pipelines declare, so an arrival is matched
/// on its whole key (agent, channel, payload type and instance). Every
/// instance of a keyed channel has a series of its own.
///
/// An arrival a requested selector covers but no bound key matches is
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
    /// The selectors it was built for. The host is asked for their paths.
    requested: HashSet<AssertionTarget>,
    /// Each bound key's index into `series`.
    by_key: HashMap<RecordedKey, usize>,
    series: Vec<Series>,
    /// How many values arrived under each key no series is bound to that a
    /// requested selector covers.
    unbound: HashMap<RecordedKey, u64>,
    /// Keys no requested selector covers: what another sink asked for,
    /// which every sink is handed, or a sibling of a selected instance.
    /// Held so each key's path is spelled once, not on every arrival.
    unrequested: HashSet<RecordedKey>,
    extractors: ExtractorTable,
    evaluations: Vec<Box<dyn Evaluator>>,
    /// Each input key's evaluations, by index, with the role it fills.
    inputs: HashMap<RecordedKey, Vec<(usize, usize)>>,
}

impl ObservationCollector {
    /// A collector requesting each of `targets`, with no series until keys
    /// are bound. A target named twice is requested once.
    pub fn new<'a>(
        targets: impl IntoIterator<Item = &'a AssertionTarget>,
        extractors: ExtractorTable,
    ) -> Self {
        Self {
            id: CollectorId(NEXT_COLLECTOR.fetch_add(1, Ordering::Relaxed)),
            requested: targets.into_iter().cloned().collect(),
            by_key: HashMap::new(),
            series: Vec::new(),
            unbound: HashMap::new(),
            unrequested: HashSet::new(),
            extractors,
            evaluations: Vec::new(),
            inputs: HashMap::new(),
        }
    }

    /// Whether the collector was built for `target`.
    pub fn requests(&self, target: &AssertionTarget) -> bool {
        self.requested.contains(target)
    }

    /// What a channel value whose payload is of type `payload` turns into:
    /// the kind of [`AssertionValue`](crate::assertion::AssertionValue) and
    /// its unit, or `None` when it can't be turned into one.
    pub fn extracted(&self, payload: TypeId) -> Option<Extracted> {
        self.extractors.extracted(payload)
    }
}

impl ObservationSink for ObservationCollector {
    /// Paths, so a channel path watches every instance; [`bind`]
    /// (Self::bind) decides which of them are kept.
    fn request(&self) -> WatchRequest {
        WatchRequest::Paths(
            self.requested
                .iter()
                .map(|target| target.path().to_owned())
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
        fixtures::{channel_target, collector, emission_key, speed, target, CAR, NIS, SPEED},
        *,
    };

    #[test]
    fn request_names_each_target_once() {
        let nis = target(CAR, NIS);
        let speed = channel_target(CAR, &speed::<f64>());
        let collector = collector(&[nis.clone(), speed.clone(), nis.clone()]);

        assert_eq!(
            collector.request(),
            WatchRequest::Paths(
                [nis.path().to_owned(), speed.path().to_owned()]
                    .into_iter()
                    .collect()
            )
        );
    }

    #[test]
    fn a_target_is_requested_but_has_no_series_until_its_key_is_bound() {
        let nis = target(CAR, NIS);
        let collector = collector(std::slice::from_ref(&nis));

        assert!(collector.requests(&nis));
        assert!(!collector.requests(&target(CAR, SPEED)));
        assert_eq!(collector.series_id(&emission_key(CAR, NIS)), None);
    }
}
