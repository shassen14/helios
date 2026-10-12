//! The sink a run's assertions read from: every value the brain reports for
//! a run-file target, kept as a series per target.
//!
//! The runner never reads a pipeline's bus. It asks the host, through the
//! sink's request, to record each target, and judges what arrives here. So
//! the same runner works wherever observations can be delivered: a sim, a
//! robot, a recording.

use crate::assertion::{
    extract::{ExtractError, ExtractorTable},
    AssertionTarget, AssertionValue,
};

use helios_core::prelude::{AgentId, MonotonicTime};
use helios_runtime::observe::{
    observation::{AgentObservation, ObservedValue},
    sink::{ObservationSink, WatchRequest},
};

use std::{any::TypeId, collections::HashMap, sync::Arc};

/// An observation's agent, node and leaf: the parts of its path, held the
/// way the observation carries them.
type LeafKey = (AgentId, Arc<str>, Arc<str>);

/// _Per-step._ Collects, per run-file target, every value the brain reports
/// for it, turned into an [`AssertionValue`] on arrival.
///
/// A value is extracted the moment it arrives and the observation is then
/// dropped, so a channel carrying a large payload costs what a scalar does:
/// the series holds the extracted value, never the payload.
pub struct ObservationCollector {
    /// Each target's index into `series`.
    by_target: HashMap<AssertionTarget, usize>,
    /// The same indices by the parts an observation carries, so an arrival
    /// is matched without building its path.
    by_leaf: HashMap<LeafKey, usize>,
    series: Vec<Series>,
    extractors: ExtractorTable,
}

impl ObservationCollector {
    /// A collector requesting each of `targets`, with an empty series for
    /// each. A target named twice gets one series, shared by every
    /// assertion on it.
    pub fn new<'a>(
        targets: impl IntoIterator<Item = &'a AssertionTarget>,
        extractors: ExtractorTable,
    ) -> Self {
        let mut collector = Self {
            by_target: HashMap::new(),
            by_leaf: HashMap::new(),
            series: Vec::new(),
            extractors,
        };
        for target in targets {
            collector.add(target);
        }

        collector
    }

    /// What arrived for `target` so far. `None` when `target` was never
    /// requested; a requested target that nothing has reported yet has an
    /// empty series.
    pub fn series(&self, target: &AssertionTarget) -> Option<&Series> {
        let index = self.by_target.get(target)?;
        self.series.get(*index)
    }

    /// Whether a channel value whose payload is of type `payload` can be
    /// turned into an [`AssertionValue`].
    pub fn can_extract(&self, payload: TypeId) -> bool {
        self.extractors.handles(payload)
    }

    /// Gives `target` a series, unless it has one already.
    fn add(&mut self, target: &AssertionTarget) {
        if self.by_target.contains_key(target) {
            return;
        }
        let index = self.series.len();
        self.series.push(Series::default());
        self.by_target.insert(target.clone(), index);
        self.by_leaf.insert(
            (
                target.agent().clone(),
                target.node().clone(),
                target.leaf().clone(),
            ),
            index,
        );
    }
}

impl ObservationSink for ObservationCollector {
    fn request(&self) -> WatchRequest {
        WatchRequest::Paths(
            self.by_target
                .keys()
                .map(|target| target.as_str().to_owned())
                .collect(),
        )
    }

    fn receive(&mut self, batch: &[AgentObservation]) {
        for tagged in batch {
            // Clones three `Arc`s: a reference count each, no allocation.
            let key = (
                tagged.agent.clone(),
                tagged.observation.node.clone(),
                tagged.observation.leaf.clone(),
            );
            let Some(series) = self
                .by_leaf
                .get(&key)
                .and_then(|index| self.series.get_mut(*index))
            else {
                continue;
            };
            match extract(&self.extractors, &tagged.observation.value) {
                Ok(value) => series.samples.push(Sample {
                    timestamp: tagged.observation.timestamp,
                    value,
                }),
                Err(rejection) => {
                    series.rejection.get_or_insert(rejection);
                }
            }
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

/// _Per-step._ Everything that arrived for one target.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Series {
    /// In arrival order, which is not always time order: a measurement
    /// update is stamped with its reading's time and can arrive after a
    /// later-stamped value.
    samples: Vec<Sample>,
    /// Why the first value that arrived but couldn't be read was turned
    /// away. Kept so an assertion over it fails for that reason rather than
    /// for having no samples. One path carries one kind of value, so the
    /// first reason stands for every one.
    rejection: Option<Rejection>,
}

impl Series {
    pub fn samples(&self) -> &[Sample] {
        &self.samples
    }

    pub fn rejection(&self) -> Option<Rejection> {
        self.rejection
    }
}

/// _Per-step._ One value for a target, with the time it describes.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    pub timestamp: MonotonicTime,
    pub value: AssertionValue,
}

/// _Per-step._ Why an arrived value isn't in its series.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rejection {
    /// A channel value with no comparable value.
    Extract(ExtractError),
    /// A kind of observation assertions can't read.
    UnsupportedKind,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::extract::standard_extractors;

    use helios_runtime::{
        observe::{observation::Observation, path::agent_path},
        prelude::{Health, Stamped},
        ErasedStamped,
    };

    const CAR: &str = "car";
    const TRUCK: &str = "truck";
    const ESTIMATOR: &str = "estimator";
    const NIS: &str = "aiding.gps.nis";
    const SPEED: &str = "speed";

    fn target(agent: &str, leaf: &str) -> AssertionTarget {
        AssertionTarget::parse(agent_path(agent, ESTIMATOR, leaf)).expect("valid target")
    }

    fn collector(targets: &[AssertionTarget]) -> ObservationCollector {
        ObservationCollector::new(targets, standard_extractors())
    }

    fn observed(agent: &str, leaf: &str, at: f64, value: ObservedValue) -> AgentObservation {
        AgentObservation {
            agent: AgentId::new(agent),
            observation: Observation {
                node: ESTIMATOR.into(),
                leaf: leaf.into(),
                timestamp: MonotonicTime(at),
                value,
            },
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

    fn samples_of(collector: &ObservationCollector, target: &AssertionTarget) -> Vec<Sample> {
        collector
            .series(target)
            .expect("target is requested")
            .samples()
            .to_vec()
    }

    #[test]
    fn request_names_each_target_once() {
        let nis = target(CAR, NIS);
        let speed = target(CAR, SPEED);
        let collector = collector(&[nis.clone(), speed.clone(), nis.clone()]);

        assert_eq!(
            collector.request(),
            WatchRequest::Paths(
                [nis.as_str().to_owned(), speed.as_str().to_owned()]
                    .into_iter()
                    .collect()
            )
        );
    }

    #[test]
    fn a_requested_target_nothing_reported_has_an_empty_series() {
        let nis = target(CAR, NIS);
        let collector = collector(std::slice::from_ref(&nis));

        assert_eq!(collector.series(&nis), Some(&Series::default()));
    }

    #[test]
    fn an_unrequested_target_has_no_series() {
        let collector = collector(&[target(CAR, NIS)]);

        assert_eq!(collector.series(&target(CAR, SPEED)), None);
    }

    #[test]
    fn a_scalar_arrives_as_a_float() {
        let nis = target(CAR, NIS);
        let mut collector = collector(std::slice::from_ref(&nis));

        collector.receive(&[observed(CAR, NIS, 1.0, ObservedValue::Scalar(2.5))]);

        assert_eq!(
            samples_of(&collector, &nis),
            vec![sample(1.0, AssertionValue::Float(2.5))]
        );
    }

    #[test]
    fn a_channel_value_arrives_through_its_payload_extractor() {
        let speed = target(CAR, SPEED);
        let mut collector = collector(std::slice::from_ref(&speed));

        collector.receive(&[observed(CAR, SPEED, 1.0, tapped(true, 1.0))]);

        assert_eq!(
            samples_of(&collector, &speed),
            vec![sample(1.0, AssertionValue::Bool(true))]
        );
    }

    #[test]
    fn a_channel_value_with_no_extractor_is_rejected_with_its_reason() {
        struct Unregistered;
        let speed = target(CAR, SPEED);
        let mut collector = collector(std::slice::from_ref(&speed));

        collector.receive(&[observed(CAR, SPEED, 1.0, tapped(Unregistered, 1.0))]);

        let series = collector.series(&speed).expect("target is requested");
        assert!(series.samples().is_empty());
        assert_eq!(
            series.rejection(),
            Some(Rejection::Extract(ExtractError::NoExtractor {
                payload: std::any::type_name::<Unregistered>()
            }))
        );
    }

    #[test]
    fn only_requested_targets_are_kept_and_agents_are_told_apart() {
        let car_nis = target(CAR, NIS);
        let mut collector = collector(std::slice::from_ref(&car_nis));

        collector.receive(&[
            observed(TRUCK, NIS, 1.0, ObservedValue::Scalar(9.0)),
            observed(CAR, SPEED, 1.0, ObservedValue::Scalar(9.0)),
            observed(CAR, NIS, 1.0, ObservedValue::Scalar(2.0)),
        ]);

        assert_eq!(
            samples_of(&collector, &car_nis),
            vec![sample(1.0, AssertionValue::Float(2.0))]
        );
    }

    #[test]
    fn samples_keep_arrival_order_across_batches() {
        let nis = target(CAR, NIS);
        let mut collector = collector(std::slice::from_ref(&nis));

        collector.receive(&[observed(CAR, NIS, 2.0, ObservedValue::Scalar(1.0))]);
        collector.receive(&[
            observed(CAR, NIS, 1.5, ObservedValue::Scalar(2.0)),
            observed(CAR, NIS, 3.0, ObservedValue::Scalar(3.0)),
        ]);

        assert_eq!(
            samples_of(&collector, &nis),
            vec![
                sample(2.0, AssertionValue::Float(1.0)),
                sample(1.5, AssertionValue::Float(2.0)),
                sample(3.0, AssertionValue::Float(3.0)),
            ]
        );
    }
}
