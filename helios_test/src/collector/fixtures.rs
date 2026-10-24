//! What the collector's tests build from: targets on one agent's estimator
//! and channels, keys to bind, and arrivals to hand it.

use super::{ObservationCollector, Sample, SeriesId};
use crate::assertion::{extract::standard_extractors, AssertionTarget, AssertionValue};

use helios_core::prelude::{AgentId, FrameId, MonotonicTime};
use helios_runtime::{
    observe::{
        key::{RecordedKey, Subject},
        observation::{Observation, ObservedValue},
        path::{channel_path, node_path},
    },
    port::{ChannelKey, ChannelKind, InstanceKey, Keyed},
    prelude::{Health, Stamped},
    ErasedStamped,
};

use std::sync::Arc;

pub(super) const CAR: &str = "car";
pub(super) const TRUCK: &str = "truck";
const ESTIMATOR: &str = "estimator";
pub(super) const NIS: &str = "aiding.gps.nis";
pub(super) const SPEED: &str = "speed";
const WHEELS: &str = "wheels";

/// A value about one wheel, keyed by the wheel. Only its key is used: the
/// values sent under it are scalars.
struct WheelSpeed(&'static str);

impl Keyed for WheelSpeed {
    type Instance = &'static str;
    type Declared = &'static str;

    fn instance(&self) -> &&'static str {
        &self.0
    }

    fn declared_frame(&self) -> Option<&FrameId> {
        None
    }
}

pub(super) fn target(agent: &str, leaf: &str) -> AssertionTarget {
    AssertionTarget::parse(node_path(agent, ESTIMATOR, leaf)).expect("valid target")
}

/// The target on `agent`'s channel `channel`.
pub(super) fn channel_target(agent: &str, channel: &ChannelKey) -> AssertionTarget {
    AssertionTarget::parse(channel_path(agent, channel)).expect("valid target")
}

/// The [`SPEED`] channel carrying `T`.
pub(super) fn speed<T: Send + Sync + 'static>() -> ChannelKey {
    ChannelKey::internal::<T>(SPEED)
}

pub(super) fn wheels() -> ChannelKey {
    ChannelKey::keyed::<WheelSpeed>(ChannelKind::Internal, WHEELS)
}

pub(super) fn wheel(name: &'static str) -> Subject {
    Subject::channel(wheels(), Some(InstanceKey::of::<WheelSpeed>(&name)))
}

pub(super) fn emission_key(agent: &str, leaf: &str) -> RecordedKey {
    key(agent, Subject::emission(ESTIMATOR, leaf))
}

pub(super) fn key(agent: &str, subject: Subject) -> RecordedKey {
    RecordedKey {
        agent: AgentId::new(agent),
        subject,
    }
}

pub(super) fn collector() -> ObservationCollector {
    ObservationCollector::new(standard_extractors())
}

/// A collector bound to `key` alone, with the series it binds, from the
/// selector spelling `key`'s path.
pub(super) fn bound(key: &RecordedKey) -> (ObservationCollector, SeriesId) {
    let mut collector = collector();
    let selector = AssertionTarget::parse(key.path()).expect("a key spells a valid path");
    let id = collector.bind(&selector, key.clone());
    (collector, id)
}

pub(super) fn arrival(key: &RecordedKey, at: f64, value: ObservedValue) -> Observation {
    Observation {
        key: key.clone(),
        timestamp: MonotonicTime(at),
        value,
    }
}

/// A channel value as a tap carries it: the bus's own stamped `Arc`.
pub(super) fn tapped<T: Send + Sync + 'static>(value: T, at: f64) -> ObservedValue {
    let stamped: Arc<dyn ErasedStamped> = Arc::new(Stamped {
        value,
        timestamp: MonotonicTime(at),
        health: Health::Ok,
        producer: 0,
    });
    ObservedValue::Typed(stamped)
}

pub(super) fn sample(at: f64, value: AssertionValue) -> Sample {
    Sample {
        timestamp: MonotonicTime(at),
        value,
    }
}

pub(super) fn samples(collector: &ObservationCollector, id: SeriesId) -> Vec<Sample> {
    collector
        .series(id)
        .expect("the id is this collector's")
        .samples()
        .to_vec()
}
