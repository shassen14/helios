use crate::pipeline::node::NodeId;

use std::borrow::Cow;

use helios_core::data::primitives::MonotonicTime;

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

impl Health {
    /// The graver of two healths — for collapsing several inputs' health into one
    /// output's when a node combines them.
    ///
    /// `Ok` is the identity (the least grave), so folding a set of contributors
    /// from an `Ok` seed yields `Ok` only when every one is `Ok`; any `Degraded`
    /// or `Stale` dominates it. The winner is returned whole, keeping its own
    /// `reason` / `duration`, so the consumer sees *why* the output is degraded,
    /// not merely that it is. A severity tie returns `other`.
    pub fn worse_of(self, other: Health) -> Health {
        if self.severity() > other.severity() {
            self
        } else {
            other
        }
    }

    /// Severity rank, higher is graver: `Ok` (0) < `Degraded` (1) < `Stale` (2).
    /// Staleness outranks a reported degradation because folding *old* information
    /// into a fresh output is the more insidious failure. Ranks the variant only;
    /// the `reason` / `duration` payload does not affect the order.
    fn severity(&self) -> u8 {
        match self {
            Health::Ok => 0,
            Health::Degraded { .. } => 1,
            Health::Stale { .. } => 2,
        }
    }
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

    #[test]
    fn worse_of_prefers_the_graver_variant() {
        // Ok < Degraded < Stale, regardless of argument order.
        assert!(matches!(
            Health::Ok.worse_of(Health::Degraded { reason: "x".into() }),
            Health::Degraded { .. }
        ));
        assert!(matches!(
            Health::Degraded { reason: "x".into() }.worse_of(Health::Ok),
            Health::Degraded { .. }
        ));
        assert!(matches!(
            Health::Degraded { reason: "x".into() }.worse_of(Health::Stale { duration: 1.0 }),
            Health::Stale { .. }
        ));
        assert!(matches!(Health::Ok.worse_of(Health::Ok), Health::Ok));
    }

    #[test]
    fn worse_of_severity_is_order_independent() {
        // Swapping the arguments cannot change which severity wins — only which
        // payload survives a tie (that is `other`, covered separately).
        let stale = Health::Stale { duration: 2.0 };
        let degraded = Health::Degraded { reason: "x".into() };
        assert!(matches!(
            stale.clone().worse_of(degraded.clone()),
            Health::Stale { .. }
        ));
        assert!(matches!(degraded.worse_of(stale), Health::Stale { .. }));
    }

    #[test]
    fn worse_of_keeps_the_winners_payload() {
        // The graver variant is returned whole, so its diagnostic survives.
        let winner = Health::Ok.worse_of(Health::Degraded {
            reason: "sensor fault".into(),
        });
        let Health::Degraded { reason } = winner else {
            panic!("expected Degraded")
        };
        assert_eq!(reason, "sensor fault");
    }
}
