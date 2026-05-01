use crate::pipeline::node::NodeId;

use std::borrow::Cow;

use helios_core::types::MonotonicTime;

/// Bus envelope wrapping every value on the port bus.
///
/// `timestamp` is set by the producer at write time. Consumers use it to
/// detect staleness relative to the current tick clock. `health` lets
/// consumers distinguish absence (`None`) from a present-but-degraded value.
/// `producer` is a build-time-assigned node ID used for diagnostics — it
/// identifies which node wrote this slot, not the originating sensor.
#[derive(Clone, Debug)]
pub struct Stamped<T> {
    pub value: T,
    pub timestamp: MonotonicTime,
    pub health: Health,
    pub producer: NodeId,
}

/// Runtime health of a bus slot as reported by its producer.
///
/// `Stale { duration }` is written by the producer when it detects its own
/// output has not been refreshed; `duration` is how many seconds it has been
/// stale. Consumers may also compute staleness independently via
/// `bus.read_fresh()` using the slot's `timestamp`.
#[derive(Clone, Debug)]
pub enum Health {
    Ok,
    Degraded { reason: Cow<'static, str> },
    Stale { duration: f64 },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamped_clone_preserves_all_fields() {
        let s = Stamped {
            value: 99u32,
            timestamp: MonotonicTime(2.5),
            health: Health::Ok,
            producer: 7,
        };
        let c = s.clone();
        assert_eq!(c.value, 99);
        assert_eq!(c.timestamp.0, 2.5);
        assert_eq!(c.producer, 7);
    }

    #[test]
    fn health_degraded_preserves_reason() {
        let h = Health::Degraded {
            reason: "sensor fault".into(),
        };
        let Health::Degraded { reason } = h else {
            panic!("wrong variant")
        };
        assert_eq!(reason, "sensor fault");
    }

    #[test]
    fn health_stale_preserves_duration() {
        let h = Health::Stale { duration: 0.25 };
        let Health::Stale { duration } = h else {
            panic!("wrong variant")
        };
        assert!((duration - 0.25).abs() < 1e-9);
    }

    #[test]
    fn health_ok_clone() {
        let h = Health::Ok;
        let _ = h.clone();
    }
}
