//! [`PayloadReader`]: takes each reading on a typed sensor channel once, in
//! time order.

use crate::port::{ChannelKey, PortBus, SensorChannel};

use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::interchange::measurement::sensor::SensorPayload;
use helios_core::prelude::MonotonicTime;

use atomic_float::AtomicF64;
use nalgebra::DVector;
use std::marker::PhantomData;
use std::sync::atomic::Ordering;

/// One reading, as a measurement vector and the time it was measured.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Measurement {
    pub(crate) z: DVector<f64>,
    pub(crate) at: MonotonicTime,
}

/// Reads `Vec<SensorReading<P>>` from one channel and hands back only the
/// readings it has not handed back before, oldest first.
///
/// Bus slots are last-known-good, so the same batch can sit on the channel for
/// several ticks. Taking it again would apply one measurement as several
/// independent ones and overstate the filter's confidence; the reader skips any
/// reading stamped no later than the newest it has already taken.
pub(crate) struct PayloadReader<P: SensorPayload> {
    channel: ChannelKey,
    /// Newest reading time taken so far; `NEG_INFINITY` until the first.
    last_taken: AtomicF64,
    // Output-only, non-owned, so the reader is Send + Sync for any payload.
    _payload: PhantomData<fn() -> P>,
}

impl<P: SensorPayload> PayloadReader<P> {
    pub(crate) fn new(channel: SensorChannel) -> Self {
        Self {
            channel: channel.into(),
            last_taken: AtomicF64::new(f64::NEG_INFINITY),
            _payload: PhantomData,
        }
    }

    /// The channel this reader reads.
    pub(crate) fn channel(&self) -> &ChannelKey {
        &self.channel
    }

    /// Every reading newer than the last one taken, sorted by measurement time
    /// so a filter sees them in causal order even when the producer batched
    /// them out of order. Empty when the channel is empty or holds nothing new.
    pub(crate) fn take_new(&self, bus: &PortBus) -> Vec<Measurement> {
        let Some(stamped) = bus.read::<Vec<SensorReading<P>>>(self.channel.clone()) else {
            return Vec::new();
        };

        let mut readings: Vec<&SensorReading<P>> = stamped.value.iter().collect();
        readings.sort_by(|a, b| a.timestamp.0.total_cmp(&b.timestamp.0));

        let last_taken = self.last_taken.load(Ordering::Relaxed);
        let fresh: Vec<Measurement> = readings
            .into_iter()
            .filter(|reading| reading.timestamp.0 > last_taken)
            .map(|reading| Measurement {
                z: reading.data.to_measurement_vector(),
                at: reading.timestamp,
            })
            .collect();

        if let Some(newest) = fresh.last() {
            self.last_taken.store(newest.at.0, Ordering::Relaxed);
        }
        fresh
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::PortDescriptor;
    use crate::stamped::{Health, Stamped};

    use helios_core::interchange::measurement::sensor::GpsPosition;
    use helios_core::prelude::AgentId;
    use helios_core::spatial::FrameId;

    use nalgebra::Vector3;

    const CHANNEL: &str = "gps";

    fn channel() -> SensorChannel {
        SensorChannel::named::<Vec<SensorReading<GpsPosition>>>(CHANNEL)
    }

    fn bus() -> PortBus {
        let key: ChannelKey = channel().into();
        PortBus::new(&[PortDescriptor::new(vec![], vec![], vec![key], None)])
    }

    /// A reading at `at` whose east coordinate is `at`, so order is visible.
    fn reading(at: f64) -> SensorReading<GpsPosition> {
        SensorReading {
            sensor: FrameId::sensor(AgentId::new("car"), CHANNEL),
            timestamp: MonotonicTime(at),
            data: GpsPosition {
                position: Vector3::new(at, 0.0, 0.0),
            },
        }
    }

    fn publish(bus: &PortBus, times: &[f64]) {
        bus.write(
            channel().into(),
            Stamped {
                value: times.iter().map(|&t| reading(t)).collect::<Vec<_>>(),
                timestamp: MonotonicTime(times.iter().copied().fold(f64::MIN, f64::max)),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("channel is on the bus");
    }

    fn times(taken: &[Measurement]) -> Vec<f64> {
        taken.iter().map(|m| m.at.0).collect()
    }

    #[test]
    fn an_empty_channel_yields_nothing() {
        let reader = PayloadReader::<GpsPosition>::new(channel());
        assert!(reader.take_new(&bus()).is_empty());
    }

    /// An out-of-order batch comes back sorted, each vector built from its
    /// own reading.
    #[test]
    fn readings_come_back_oldest_first() {
        let bus = bus();
        publish(&bus, &[2.0, 1.0, 3.0]);
        let taken = PayloadReader::<GpsPosition>::new(channel()).take_new(&bus);
        assert_eq!(times(&taken), [1.0, 2.0, 3.0]);
        assert_eq!(taken[0].z[0], 1.0);
    }

    /// A batch still on the channel next tick is not taken twice; only
    /// readings newer than the last taken come through.
    #[test]
    fn a_reading_is_taken_once() {
        let bus = bus();
        let reader = PayloadReader::<GpsPosition>::new(channel());

        publish(&bus, &[1.0, 2.0]);
        assert_eq!(times(&reader.take_new(&bus)), [1.0, 2.0]);
        assert!(reader.take_new(&bus).is_empty(), "same batch, next tick");

        publish(&bus, &[2.0, 3.0]);
        assert_eq!(times(&reader.take_new(&bus)), [3.0]);
    }
}
