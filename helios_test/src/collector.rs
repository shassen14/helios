//! The sink a run's assertions read from: every value the brain reports for
//! a run-file target, kept as a series per target.
//!
//! The runner never reads a pipeline's bus. It asks the host, through the
//! sink's request, to record each target, and judges what arrives here. So
//! the same runner works wherever observations can be delivered: a sim, a
//! robot, a recording.

use crate::{
    assertion::{
        extract::{ExtractError, Extracted, ExtractorTable},
        AssertionTarget, AssertionValue,
    },
    evaluation::{
        derived::{Derived, Refused},
        kind::{Evaluator, InputRejection},
    },
};

use helios_core::prelude::MonotonicTime;
use helios_runtime::observe::{
    key::{RecordedKey, Subject},
    observation::{Observation, ObservedValue},
    sink::{ObservationSink, WatchRequest},
};

use std::{
    any::TypeId,
    collections::{HashMap, HashSet},
    fmt::{self, Display},
    sync::atomic::{AtomicU64, Ordering},
};

/// The id the next collector built takes. Counts up for the life of the
/// process, so no two collectors share one.
static NEXT_COLLECTOR: AtomicU64 = AtomicU64::new(0);

/// _Per-step._ Collects, per recorded key, every value the brain reports
/// under it, turned into an [`AssertionValue`] on arrival.
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

    /// The series bound to `key`. `None` when `key` was never bound.
    pub fn series_id(&self, key: &RecordedKey) -> Option<SeriesId> {
        let index = self.by_key.get(key)?;
        Some(self.id_at(*index))
    }

    /// What arrived so far in the series `id` names. `None` when `id` came
    /// from another collector. A series nothing has reported to yet is
    /// empty.
    pub fn series(&self, id: SeriesId) -> Option<&Series> {
        if id.collector != self.id {
            return None;
        }
        self.series.get(id.index)
    }

    /// Each key a value arrived under that a requested selector covers but
    /// no series is bound to, with how many arrived, in path order. Empty when every
    /// requested value landed in a series.
    pub fn unbound_arrivals(&self) -> Vec<UnboundArrivals> {
        let mut unbound: Vec<UnboundArrivals> = self
            .unbound
            .iter()
            .map(|(key, count)| UnboundArrivals {
                key: key.clone(),
                count: *count,
            })
            .collect();
        unbound.sort_by_cached_key(|arrivals| (arrivals.key.path(), arrivals.key.subject.clone()));
        unbound
    }

    /// What a channel value whose payload is of type `payload` turns into:
    /// the kind of [`AssertionValue`] and its unit, or `None` when it can't
    /// be turned into one.
    pub fn extracted(&self, payload: TypeId) -> Option<Extracted> {
        self.extractors.extracted(payload)
    }

    /// The series values recorded under `key` arrive in, given an empty one
    /// unless `key` has one already, so every reader of one key shares one
    /// series. Bound before the first tick: a value arriving under a key not
    /// yet bound is dropped.
    pub fn bind(&mut self, key: RecordedKey) -> SeriesId {
        if let Some(index) = self.by_key.get(&key) {
            return self.id_at(*index);
        }
        let index = self.series.len();
        self.series.push(Series::default());
        self.by_key.insert(key, index);
        self.id_at(index)
    }

    /// Counts a value that arrived under `key`, bound to no series, when a
    /// requested selector covers it.
    fn count_unbound(&mut self, key: &RecordedKey) {
        if let Some(count) = self.unbound.get_mut(key) {
            *count += 1;
            return;
        }
        if self.unrequested.contains(key) {
            return;
        }
        let path = key.path();
        let instance = instance_of(&key.subject);
        let covered = self
            .requested
            .iter()
            .any(|target| target.path() == path && target.covers(instance));
        if covered {
            self.unbound.insert(key.clone(), 1);
        } else {
            self.unrequested.insert(key.clone());
        }
    }

    /// Feeds `evaluation` the values recorded under each of `inputs`, one
    /// key per role in role order. Bound before the first tick, as a series
    /// is.
    pub fn bind_evaluation(
        &mut self,
        evaluation: Box<dyn Evaluator>,
        inputs: Vec<RecordedKey>,
    ) -> EvaluationId {
        let index = self.evaluations.len();
        self.evaluations.push(evaluation);
        for (role, key) in inputs.into_iter().enumerate() {
            self.inputs.entry(key).or_default().push((index, role));
        }
        EvaluationId {
            collector: self.id,
            index,
        }
    }

    /// Each role's refusals so far in the evaluation `id` names. `None` when
    /// `id` came from another collector.
    pub fn refused(&self, id: EvaluationId) -> Option<&[Refused]> {
        self.evaluation(id).map(|evaluation| evaluation.refused())
    }

    /// What the evaluation `id` names derived of its `wanted` metrics from
    /// the values stamped at or before `until`. `None` when `id` came from
    /// another collector.
    pub fn derive(
        &self,
        id: EvaluationId,
        until: MonotonicTime,
        wanted: &[usize],
    ) -> Option<Derived> {
        self.evaluation(id)
            .map(|evaluation| evaluation.derive(until, wanted))
    }

    fn evaluation(&self, id: EvaluationId) -> Option<&dyn Evaluator> {
        if id.collector != self.id {
            return None;
        }
        self.evaluations.get(id.index).map(Box::as_ref)
    }

    fn id_at(&self, index: usize) -> SeriesId {
        SeriesId {
            collector: self.id,
            index,
        }
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
            let series = self
                .by_key
                .get(&observation.key)
                .and_then(|index| self.series.get_mut(*index));
            let inputs = self.inputs.get(&observation.key);
            if series.is_none() && inputs.is_none() {
                self.count_unbound(&observation.key);
                continue;
            }
            if let Some(series) = series {
                match extract(&self.extractors, &observation.value) {
                    Ok(value) => series.samples.push(Sample {
                        timestamp: observation.timestamp,
                        value,
                    }),
                    Err(rejection) => {
                        series.rejection.get_or_insert(rejection);
                    }
                }
            }
            // Setup binds an input only to a channel of the role's payload
            // type, so a value there is always typed.
            if let (Some(inputs), ObservedValue::Typed(stamped)) = (inputs, &observation.value) {
                for (evaluation, role) in inputs {
                    if let Some(evaluation) = self.evaluations.get_mut(*evaluation) {
                        evaluation.receive(*role, observation.timestamp, stamped.payload());
                    }
                }
            }
        }
    }
}

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

/// The comparable value inside `value`: a scalar is a float, a channel value
/// goes through its payload type's extractor.
fn extract(
    extractors: &ExtractorTable,
    value: &ObservedValue,
) -> Result<AssertionValue, Rejection> {
    match value {
        ObservedValue::Scalar(scalar) => Ok(AssertionValue::Float(*scalar)),
        ObservedValue::Typed(stamped) => extractors
            .extract(stamped.as_ref())
            .map_err(Rejection::Extract),
        _ => Err(Rejection::UnsupportedKind),
    }
}

/// _Post-step._ Values that arrived under one key whose path a target
/// requests but to which no series is bound.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnboundArrivals {
    pub key: RecordedKey,
    pub count: u64,
}

/// The path, then the whole key where the path alone doesn't name it: a
/// channel's kind and payload type, and its instance.
impl Display for UnboundArrivals {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.key.path())?;
        if let Subject::Channel { channel, instance } = &self.key.subject {
            write!(f, " ({channel}")?;
            if let Some(instance) = instance {
                write!(f, ", instance \"{instance}\"")?;
            }
            write!(f, ")")?;
        }
        write!(f, ": {} value(s) arrived under no bound series", self.count)
    }
}

/// _Per-step._ Which collector a [`SeriesId`] came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct CollectorId(u64);

/// _Per-step._ Names one series in one collector, handed out by
/// [`ObservationCollector::series_id`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SeriesId {
    collector: CollectorId,
    index: usize,
}

/// _Per-step._ Names one evaluation in one collector, handed out by
/// [`ObservationCollector::bind_evaluation`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EvaluationId {
    collector: CollectorId,
    index: usize,
}

/// _Per-step._ Everything that arrived under one key.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Series {
    /// In arrival order, which is not always time order: a measurement
    /// update is stamped with its reading's time and can arrive after a
    /// later-stamped value.
    samples: Vec<Sample>,
    /// Why the first value that arrived but couldn't be read was turned
    /// away. Kept so an assertion over it fails for that reason rather than
    /// for having no samples. One key carries one kind of value, so the
    /// first reason stands for every one.
    rejection: Option<Rejection>,
}

impl Series {
    /// A series an evaluation derived, whole, with the first value its
    /// inputs refused.
    pub(crate) fn derived(samples: Vec<Sample>, rejection: Option<Rejection>) -> Self {
        Self { samples, rejection }
    }

    pub fn samples(&self) -> &[Sample] {
        &self.samples
    }

    pub fn rejection(&self) -> Option<Rejection> {
        self.rejection
    }
}

/// _Per-step._ One value under a key, with the time it describes.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    pub timestamp: MonotonicTime,
    pub value: AssertionValue,
}

/// _Per-step._ Why an arrived value isn't in its series.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum Rejection {
    /// A channel value with no comparable value.
    #[error(transparent)]
    Extract(ExtractError),
    /// A kind of observation assertions can't read.
    #[error("a kind of observation assertions can't read")]
    UnsupportedKind,
    /// A value an evaluation's input refused, kept on each series it
    /// derives.
    #[error("an input of its evaluation refused {0}")]
    Evaluation(InputRejection),
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::{extract::standard_extractors, InstanceSelector};

    use helios_core::prelude::AgentId;
    use helios_runtime::{
        observe::{
            key::Subject,
            path::{channel_path, node_path},
        },
        port::{ChannelKey, ChannelKind, InstanceKey, Keyed},
        prelude::{Health, Stamped},
        ErasedStamped,
    };

    use std::sync::Arc;

    const CAR: &str = "car";
    const TRUCK: &str = "truck";
    const ESTIMATOR: &str = "estimator";
    const NIS: &str = "aiding.gps.nis";
    const SPEED: &str = "speed";
    const WHEELS: &str = "wheels";

    /// A value about one wheel, keyed by the wheel. Only its key is used:
    /// the values sent under it are scalars.
    struct WheelSpeed(&'static str);

    impl Keyed for WheelSpeed {
        type Instance = &'static str;

        fn instance(&self) -> &&'static str {
            &self.0
        }
    }

    fn target(agent: &str, leaf: &str) -> AssertionTarget {
        AssertionTarget::parse(node_path(agent, ESTIMATOR, leaf)).expect("valid target")
    }

    /// The target on `agent`'s channel `channel`.
    fn channel_target(agent: &str, channel: &ChannelKey) -> AssertionTarget {
        AssertionTarget::parse(channel_path(agent, channel)).expect("valid target")
    }

    /// The [`SPEED`] channel carrying `T`.
    fn speed<T: Send + Sync + 'static>() -> ChannelKey {
        ChannelKey::internal::<T>(SPEED)
    }

    fn wheels() -> ChannelKey {
        ChannelKey::keyed::<WheelSpeed>(ChannelKind::Internal, WHEELS)
    }

    fn wheel(name: &'static str) -> Subject {
        Subject::channel(wheels(), Some(InstanceKey::of::<WheelSpeed>(&name)))
    }

    fn emission_key(agent: &str, leaf: &str) -> RecordedKey {
        key(agent, Subject::emission(ESTIMATOR, leaf))
    }

    fn key(agent: &str, subject: Subject) -> RecordedKey {
        RecordedKey {
            agent: AgentId::new(agent),
            subject,
        }
    }

    fn collector(targets: &[AssertionTarget]) -> ObservationCollector {
        ObservationCollector::new(targets, standard_extractors())
    }

    /// A collector bound to `key` alone, with the series it binds.
    fn bound(key: &RecordedKey) -> (ObservationCollector, SeriesId) {
        let mut collector = collector(&[]);
        let id = collector.bind(key.clone());
        (collector, id)
    }

    fn arrival(key: &RecordedKey, at: f64, value: ObservedValue) -> Observation {
        Observation {
            key: key.clone(),
            timestamp: MonotonicTime(at),
            value,
        }
    }

    /// A channel value as a tap carries it: the bus's own stamped `Arc`.
    fn tapped<T: Send + Sync + 'static>(value: T, at: f64) -> ObservedValue {
        let stamped: Arc<dyn ErasedStamped> = Arc::new(Stamped {
            value,
            timestamp: MonotonicTime(at),
            health: Health::Ok,
            producer: 0,
        });
        ObservedValue::Typed(stamped)
    }

    fn sample(at: f64, value: AssertionValue) -> Sample {
        Sample {
            timestamp: MonotonicTime(at),
            value,
        }
    }

    fn samples(collector: &ObservationCollector, id: SeriesId) -> Vec<Sample> {
        collector
            .series(id)
            .expect("the id is this collector's")
            .samples()
            .to_vec()
    }

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

    #[test]
    fn a_bound_key_nothing_reported_has_an_empty_series() {
        let (collector, id) = bound(&emission_key(CAR, NIS));

        assert_eq!(collector.series(id), Some(&Series::default()));
    }

    #[test]
    fn binding_one_key_twice_shares_one_series() {
        let mut collector = collector(&[]);

        let first = collector.bind(emission_key(CAR, NIS));
        let second = collector.bind(emission_key(CAR, NIS));

        assert_eq!(first, second);
    }

    #[test]
    fn a_series_id_reads_nothing_from_another_collector() {
        // Two runs' collectors bound the same way: an id from the first must
        // not read the second's series, which looks just the same.
        let nis = emission_key(CAR, NIS);
        let (first, id) = bound(&nis);
        let (second, _) = bound(&nis);

        assert!(first.series(id).is_some());
        assert_eq!(second.series(id), None);
    }

    #[test]
    fn a_scalar_arrives_as_a_float() {
        let nis = emission_key(CAR, NIS);
        let (mut collector, id) = bound(&nis);

        collector.receive(&[arrival(&nis, 1.0, ObservedValue::Scalar(2.5))]);

        assert_eq!(
            samples(&collector, id),
            vec![sample(1.0, AssertionValue::Float(2.5))]
        );
    }

    #[test]
    fn a_channel_value_arrives_through_its_payload_extractor() {
        let speed = key(CAR, Subject::channel(speed::<bool>(), None));
        let (mut collector, id) = bound(&speed);

        collector.receive(&[arrival(&speed, 1.0, tapped(true, 1.0))]);

        assert_eq!(
            samples(&collector, id),
            vec![sample(1.0, AssertionValue::Bool(true))]
        );
    }

    #[test]
    fn a_value_under_a_bound_key_with_no_extractor_is_rejected_with_its_reason() {
        // Setup refuses such a target; a collector bound by hand still says
        // why it kept nothing.
        struct Unregistered;
        let speed = key(CAR, Subject::channel(speed::<Unregistered>(), None));
        let (mut collector, id) = bound(&speed);

        collector.receive(&[arrival(&speed, 1.0, tapped(Unregistered, 1.0))]);

        let series = collector.series(id).expect("the id is this collector's");
        assert!(series.samples().is_empty());
        assert_eq!(
            series.rejection(),
            Some(Rejection::Extract(ExtractError::NoExtractor {
                payload: std::any::type_name::<Unregistered>()
            }))
        );
    }

    #[test]
    fn a_value_under_the_right_name_but_another_payload_type_lands_nowhere() {
        // The path spells only the name; the bound key carries the type.
        let as_float = key(CAR, Subject::channel(speed::<f64>(), None));
        let as_bool = key(CAR, Subject::channel(speed::<bool>(), None));
        let (mut collector, id) = bound(&as_float);

        collector.receive(&[arrival(&as_bool, 1.0, tapped(true, 1.0))]);

        assert_eq!(collector.series(id), Some(&Series::default()));
    }

    #[test]
    fn each_instance_of_a_keyed_channel_has_its_own_series() {
        let front = key(CAR, wheel("front"));
        let rear = key(CAR, wheel("rear"));
        let mut collector = collector(&[channel_target(CAR, &wheels())]);
        let front_id = collector.bind(front.clone());
        let rear_id = collector.bind(rear.clone());

        collector.receive(&[
            arrival(&front, 1.0, ObservedValue::Scalar(1.0)),
            arrival(&rear, 1.0, ObservedValue::Scalar(2.0)),
            arrival(&key(CAR, wheel("spare")), 1.0, ObservedValue::Scalar(3.0)),
        ]);

        assert_ne!(front_id, rear_id);
        assert_eq!(
            samples(&collector, front_id),
            vec![sample(1.0, AssertionValue::Float(1.0))]
        );
        assert_eq!(
            samples(&collector, rear_id),
            vec![sample(1.0, AssertionValue::Float(2.0))]
        );
    }

    #[test]
    fn a_value_under_a_requested_path_but_no_bound_key_is_counted() {
        // `wheels` is requested and only the front wheel bound: the rear's
        // values are counted, each arrival once, the front's are not.
        let front = key(CAR, wheel("front"));
        let rear = key(CAR, wheel("rear"));
        let mut collector = collector(&[channel_target(CAR, &wheels())]);
        collector.bind(front.clone());

        collector.receive(&[
            arrival(&front, 1.0, ObservedValue::Scalar(1.0)),
            arrival(&rear, 1.0, ObservedValue::Scalar(2.0)),
        ]);
        collector.receive(&[arrival(&rear, 2.0, ObservedValue::Scalar(2.0))]);

        assert_eq!(
            collector.unbound_arrivals(),
            vec![UnboundArrivals {
                key: rear,
                count: 2
            }]
        );
    }

    #[test]
    fn a_selector_naming_one_instance_does_not_count_its_siblings() {
        // Front is selected and bound; rear is a sibling no selector covers.
        // Another selector with no instance would cover it.
        let front = key(CAR, wheel("front"));
        let rear = key(CAR, wheel("rear"));
        let front_only =
            channel_target(CAR, &wheels()).with_instance(InstanceSelector::One("front".into()));
        let mut selected = collector(std::slice::from_ref(&front_only));
        selected.bind(front.clone());
        selected.receive(&[arrival(&rear, 1.0, ObservedValue::Scalar(2.0))]);
        assert_eq!(selected.unbound_arrivals(), vec![]);

        let mut with_whole = collector(&[front_only, channel_target(CAR, &wheels())]);
        with_whole.bind(front);
        with_whole.receive(&[arrival(&rear, 1.0, ObservedValue::Scalar(2.0))]);
        assert_eq!(
            with_whole.unbound_arrivals(),
            vec![UnboundArrivals {
                key: rear,
                count: 1
            }]
        );
    }

    #[test]
    fn a_value_under_a_path_no_target_requests_is_not_counted() {
        // Another sink's request: every sink is handed the whole batch.
        let mut collector = collector(&[target(CAR, NIS)]);
        collector.bind(emission_key(CAR, NIS));

        collector.receive(&[
            arrival(&emission_key(CAR, SPEED), 1.0, ObservedValue::Scalar(1.0)),
            arrival(&emission_key(CAR, SPEED), 2.0, ObservedValue::Scalar(1.0)),
        ]);

        assert_eq!(collector.unbound_arrivals(), vec![]);
    }

    #[test]
    fn a_value_arriving_before_its_key_is_bound_is_counted() {
        let nis = emission_key(CAR, NIS);
        let mut collector = collector(&[target(CAR, NIS)]);

        collector.receive(&[arrival(&nis, 1.0, ObservedValue::Scalar(1.0))]);
        let id = collector.bind(nis.clone());

        assert_eq!(collector.series(id), Some(&Series::default()));
        assert_eq!(
            collector.unbound_arrivals(),
            vec![UnboundArrivals { key: nis, count: 1 }]
        );
    }

    #[test]
    fn unbound_arrivals_name_the_whole_key_where_the_path_does_not() {
        let rear = UnboundArrivals {
            key: key(CAR, wheel("rear")),
            count: 2,
        };
        let nis = UnboundArrivals {
            key: emission_key(CAR, NIS),
            count: 1,
        };

        assert_eq!(
            rear.to_string(),
            format!(
                "agent.car.channel.wheels ({}, instance \"rear\"): 2 value(s) arrived \
                 under no bound series",
                wheels()
            )
        );
        assert_eq!(
            nis.to_string(),
            "agent.car.node.estimator.aiding.gps.nis: 1 value(s) arrived under no bound series"
        );
    }

    #[test]
    fn only_bound_keys_are_kept_and_agents_and_subjects_are_told_apart() {
        let car_nis = emission_key(CAR, NIS);
        let (mut collector, id) = bound(&car_nis);

        collector.receive(&[
            arrival(&emission_key(TRUCK, NIS), 1.0, ObservedValue::Scalar(9.0)),
            arrival(&emission_key(CAR, SPEED), 1.0, ObservedValue::Scalar(9.0)),
            arrival(
                &key(
                    CAR,
                    Subject::channel(ChannelKey::internal::<f64>(NIS), None),
                ),
                1.0,
                ObservedValue::Scalar(9.0),
            ),
            arrival(&car_nis, 1.0, ObservedValue::Scalar(2.0)),
        ]);

        assert_eq!(
            samples(&collector, id),
            vec![sample(1.0, AssertionValue::Float(2.0))]
        );
    }

    #[test]
    fn samples_keep_arrival_order_across_batches() {
        let nis = emission_key(CAR, NIS);
        let (mut collector, id) = bound(&nis);

        collector.receive(&[arrival(&nis, 2.0, ObservedValue::Scalar(1.0))]);
        collector.receive(&[
            arrival(&nis, 1.5, ObservedValue::Scalar(2.0)),
            arrival(&nis, 3.0, ObservedValue::Scalar(3.0)),
        ]);

        assert_eq!(
            samples(&collector, id),
            vec![
                sample(2.0, AssertionValue::Float(1.0)),
                sample(1.5, AssertionValue::Float(2.0)),
                sample(3.0, AssertionValue::Float(3.0)),
            ]
        );
    }
}
