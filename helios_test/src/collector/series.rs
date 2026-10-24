//! The collector's series: binding a key to one, and keeping each value
//! that arrives under it.

use super::{CollectorId, ObservationCollector};
use crate::assertion::{
    extract::{ExtractError, ExtractorTable},
    AssertionTarget, AssertionValue,
};
use crate::evaluation::kind::InputRejection;

use helios_core::prelude::MonotonicTime;
use helios_runtime::observe::{
    key::RecordedKey,
    observation::{Observation, ObservedValue},
};

impl ObservationCollector {
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

    /// The series values recorded under `key` arrive in, given an empty one
    /// unless `key` has one already, so every reader of one key shares one
    /// series. `selector` is what setup resolved `key` from, and covers it.
    /// Bound before the host asks what to record: a value arriving under a
    /// key not yet bound is dropped.
    pub fn bind(&mut self, selector: &AssertionTarget, key: RecordedKey) -> SeriesId {
        self.cover(selector.clone());
        if let Some(index) = self.by_key.get(&key) {
            return self.id_at(*index);
        }
        let index = self.series.len();
        self.series.push(Series::default());
        self.by_key.insert(key, index);
        self.id_at(index)
    }

    /// Keeps `observation` in the series bound to its key, or the reason it
    /// couldn't be read. Whether a series is bound to the key.
    pub(super) fn record(&mut self, observation: &Observation) -> bool {
        let Some(series) = self
            .by_key
            .get(&observation.key)
            .and_then(|index| self.series.get_mut(*index))
        else {
            return false;
        };
        match extract(&self.extractors, &observation.value) {
            Ok(value) => series.samples.push(Sample {
                timestamp: observation.timestamp,
                value,
            }),
            Err(rejection) => {
                series.rejection.get_or_insert(rejection);
            }
        }
        true
    }

    fn id_at(&self, index: usize) -> SeriesId {
        SeriesId {
            collector: self.id,
            index,
        }
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

/// _Per-step._ Names one series in one collector, handed out by
/// [`ObservationCollector::series_id`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SeriesId {
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
    use super::{super::fixtures::*, *};

    use helios_runtime::{
        observe::{key::Subject, sink::ObservationSink},
        port::ChannelKey,
    };

    #[test]
    fn a_bound_key_nothing_reported_has_an_empty_series() {
        let (collector, id) = bound(&emission_key(CAR, NIS));

        assert_eq!(collector.series(id), Some(&Series::default()));
    }

    #[test]
    fn binding_one_key_twice_shares_one_series() {
        let mut collector = collector();

        let first = collector.bind(&target(CAR, NIS), emission_key(CAR, NIS));
        let second = collector.bind(&target(CAR, NIS), emission_key(CAR, NIS));

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
        let every = channel_target(CAR, &wheels());
        let mut collector = collector();
        let front_id = collector.bind(&every, front.clone());
        let rear_id = collector.bind(&every, rear.clone());

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
