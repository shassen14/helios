//! [`RoleBuffer`]: one role's kept values, and how they line up at the
//! base's stamps. The one place a payload is downcast, and the one place a
//! role's values are lined up.

use crate::evaluation::{
    config::Gap,
    kind::{BoundInput, Cap, InputRejection},
};

use helios_core::prelude::MonotonicTime;

use std::any::Any;

/// A value of another payload type than its role reads. Setup binds each
/// role to a series of its type, so this is never reached.
pub(super) const NOT_ITS_PAYLOAD: InputRejection =
    InputRejection("a value of another type than its role reads");

/// _Per-step._ One role's kept values, each with the stamp of its payload,
/// kept in stamp order as they arrive; values of one stamp keep the order
/// they arrived in.
pub struct RoleBuffer<K> {
    values: Vec<(MonotonicTime, K)>,
}

impl<K> RoleBuffer<K> {
    /// Its values stamped at or before `until`, in stamp order.
    pub(super) fn up_to(&self, until: MonotonicTime) -> &[(MonotonicTime, K)] {
        &self.values[..self.values.partition_point(|(at, _)| at.0 <= until.0)]
    }

    /// Its value at the base stamp `at` as `rule` says, from its values up
    /// to `until`, or `None` where it has none: before its first value,
    /// past its cap, or, interpolated, after its last or between two it
    /// can't blend. Never extrapolated.
    pub(super) fn at(
        &self,
        at: MonotonicTime,
        until: MonotonicTime,
        rule: &GapRule<K>,
    ) -> Option<Lined<'_, K>> {
        let values = self.up_to(until);
        match rule {
            GapRule::Hold(cap) => {
                let after = values.partition_point(|(stamp, _)| stamp.0 <= at.0);
                let (held_at, held) = values.get(after.checked_sub(1)?)?;
                if cap.is_some_and(|cap| !cap.spans(at.0 - held_at.0)) {
                    return None;
                }
                Some(Lined::Kept(held))
            }
            GapRule::Interpolate(blend, cap) => {
                let after = values.partition_point(|(stamp, _)| stamp.0 < at.0);
                let (after_at, after_value) = values.get(after)?;
                if after_at.0 == at.0 {
                    return Some(Lined::Kept(after_value));
                }
                let (before_at, before_value) = values.get(after.checked_sub(1)?)?;
                if !cap.spans(after_at.0 - before_at.0) {
                    return None;
                }
                let fraction = (at.0 - before_at.0) / (after_at.0 - before_at.0);
                blend(before_value, after_value, fraction).map(Lined::Blended)
            }
        }
    }

    /// What `project` keeps of `payload`, a `P`, stamped `at`, or why it is
    /// refused. A value that arrives late is placed at its stamp.
    pub(super) fn keep<P: 'static>(
        &mut self,
        at: MonotonicTime,
        payload: &dyn Any,
        project: impl FnOnce(&P) -> Result<K, InputRejection>,
    ) -> Result<(), InputRejection> {
        let payload = payload.downcast_ref::<P>().ok_or(NOT_ITS_PAYLOAD)?;
        let kept = project(payload)?;
        let after = self.values.partition_point(|(stamp, _)| stamp.0 <= at.0);
        self.values.insert(after, (at, kept));
        Ok(())
    }
}

impl<K> Default for RoleBuffer<K> {
    fn default() -> Self {
        Self { values: Vec::new() }
    }
}

/// _Pre-step._ How a role other than the base lines up at each base stamp,
/// and how wide a gap it spans. A cap's width is the run's, not the kind's.
pub enum Lining<K> {
    /// The value in force however old: a goal published once.
    HoldIndefinitely,
    /// The value in force while no older than the role's cap: a command
    /// that expires.
    HoldWithinCap,
    /// Blended from the two values around the stamp by the function given,
    /// when they are no further apart than the role's cap: the value
    /// `fraction` of the way from the first to the second, `None` when the
    /// two can't be blended.
    InterpolateWithinCap(fn(&K, &K, f64) -> Option<K>),
}

impl<K> Lining<K> {
    /// The rule `role`, reading `input`, lines up by, with the cap `gap`
    /// gives a capped lining, or why a capped one has none.
    pub(super) fn with_cap(
        self,
        role: &str,
        input: &BoundInput,
        gap: Option<Gap>,
    ) -> Result<GapRule<K>, String> {
        Ok(match self {
            Lining::HoldIndefinitely => GapRule::Hold(None),
            Lining::HoldWithinCap => GapRule::Hold(Some(Cap::of(role, input, gap)?)),
            Lining::InterpolateWithinCap(blend) => {
                GapRule::Interpolate(blend, Cap::of(role, input, gap)?)
            }
        })
    }
}

/// _Pre-step._ A [`Lining`] with its cap: what a role lines up by once
/// bound, so a capped lining can't be used without one.
pub(super) enum GapRule<K> {
    /// Held, up to the cap when there is one.
    Hold(Option<Cap>),
    /// Interpolated across gaps up to the cap.
    Interpolate(fn(&K, &K, f64) -> Option<K>, Cap),
}

impl<K> GapRule<K> {
    pub(super) fn cap(&self) -> Option<Cap> {
        match self {
            GapRule::Hold(cap) => *cap,
            GapRule::Interpolate(_, cap) => Some(*cap),
        }
    }
}

/// _Per-step._ One role's value lined up at a base sample: one it kept, or
/// one blended from the two it kept around the sample.
pub enum Lined<'a, K> {
    Kept(&'a K),
    Blended(K),
}

impl<K> Lined<'_, K> {
    pub(super) fn value(&self) -> &K {
        match self {
            Lined::Kept(value) => value,
            Lined::Blended(value) => value,
        }
    }
}

/// _Per-step._ Every base value within a run, and each covered one's
/// values, lined up in role order.
pub struct LinedUp<L> {
    /// Base values stamped within the run.
    pub base_samples: usize,
    /// Base values some role had no value for.
    pub uncovered: usize,
    /// Each covered base value's stamp and its row, in stamp order.
    pub rows: Vec<(MonotonicTime, L)>,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::config::GapWidth;

    fn at(t: f64) -> MonotonicTime {
        MonotonicTime(t)
    }

    fn blend(before: &f64, after: &f64, fraction: f64) -> Option<f64> {
        Some(before + (after - before) * fraction)
    }

    /// A buffer of `values` kept as they are, in the order given.
    fn buffer(values: &[(f64, f64)]) -> RoleBuffer<f64> {
        let mut buffer = RoleBuffer::default();
        for (t, value) in values {
            buffer
                .keep(at(*t), value, |value: &f64| Ok(*value))
                .expect("kept");
        }
        buffer
    }

    fn lined(lined: Option<Lined<'_, f64>>) -> Option<f64> {
        lined.map(|lined| *lined.value())
    }

    fn width(value: f64) -> GapWidth {
        GapWidth::try_from(value).expect("positive and finite")
    }

    fn cap(seconds: f64) -> Cap {
        Cap::in_seconds(width(seconds))
    }

    #[test]
    fn a_payload_of_another_type_or_one_its_projection_refuses_is_not_kept() {
        let mut buffer = RoleBuffer::<f64>::default();

        let other_type = buffer.keep(at(1.0), &"one".to_owned(), |value: &f64| Ok(*value));
        let refused = buffer.keep(at(1.0), &1.0, |_: &f64| Err(InputRejection("refused")));

        assert_eq!(other_type, Err(NOT_ITS_PAYLOAD));
        assert_eq!(refused, Err(InputRejection("refused")));
        assert!(buffer.up_to(at(10.0)).is_empty());
    }

    #[test]
    fn values_are_taken_up_to_the_end_in_stamp_order_whatever_order_they_arrived_in() {
        let buffer = buffer(&[(3.0, 30.0), (1.0, 10.0), (5.0, 50.0), (2.0, 20.0)]);

        assert_eq!(
            buffer.up_to(at(3.0)),
            vec![(at(1.0), 10.0), (at(2.0), 20.0), (at(3.0), 30.0)]
        );
    }

    #[test]
    fn a_held_value_is_the_last_at_or_before_the_stamp_and_none_before_the_first() {
        let buffer = buffer(&[(1.0, 10.0), (3.0, 30.0)]);

        let at_stamps: Vec<_> = [0.5, 1.0, 2.0, 3.0, 9.0]
            .map(|t| lined(buffer.at(at(t), at(10.0), &GapRule::Hold(None))))
            .into();

        assert_eq!(
            at_stamps,
            vec![None, Some(10.0), Some(10.0), Some(30.0), Some(30.0)]
        );
    }

    #[test]
    fn an_interpolated_value_is_blended_between_its_neighbors_and_never_extrapolated() {
        let buffer = buffer(&[(1.0, 10.0), (3.0, 30.0)]);
        let unblendable = GapRule::Interpolate(|_, _, _| None, cap(10.0));

        let at_stamps: Vec<_> = [0.5, 1.0, 2.5, 3.0, 4.0]
            .map(|t| lined(buffer.at(at(t), at(10.0), &GapRule::Interpolate(blend, cap(10.0)))))
            .into();

        assert_eq!(
            at_stamps,
            vec![None, Some(10.0), Some(25.0), Some(30.0), None]
        );
        assert_eq!(lined(buffer.at(at(2.0), at(10.0), &unblendable)), None);
        assert_eq!(
            lined(buffer.at(at(3.0), at(10.0), &unblendable)),
            Some(30.0)
        );
    }

    #[test]
    fn a_held_value_older_than_its_cap_is_none() {
        let buffer = buffer(&[(1.0, 10.0)]);
        let capped = GapRule::Hold(Some(cap(0.5)));

        let at_stamps: Vec<_> = [1.0, 1.5, 1.6]
            .map(|t| lined(buffer.at(at(t), at(10.0), &capped)))
            .into();

        assert_eq!(at_stamps, vec![Some(10.0), Some(10.0), None]);
    }

    #[test]
    fn values_further_apart_than_the_cap_are_not_blended_but_each_is_kept_at_its_stamp() {
        let buffer = buffer(&[(1.0, 10.0), (2.0, 20.0), (4.0, 40.0)]);
        let capped = GapRule::Interpolate(blend, cap(1.0));

        let at_stamps: Vec<_> = [1.5, 2.0, 3.0, 4.0]
            .map(|t| lined(buffer.at(at(t), at(10.0), &capped)))
            .into();

        assert_eq!(at_stamps, vec![Some(15.0), Some(20.0), None, Some(40.0)]);
    }

    #[test]
    fn a_gap_of_exactly_the_cap_is_spanned_whichever_way_its_stamps_rounded() {
        // Stamps of a 400 Hz tick, three periods apart: their difference
        // rounds above 3/400 for most ticks, and a cap of 3 periods spans
        // it all the same.
        const RATE_HZ: f64 = 400.0;
        let three_periods = Cap::in_periods(width(3.0), RATE_HZ);
        let stamp = |tick: u32| f64::from(tick) / RATE_HZ;

        for tick in [400, 7_001, 23_997] {
            let (first, last) = (stamp(tick), stamp(tick + 3));
            let held = buffer(&[(first, 1.0)]);
            let blended = buffer(&[(first, 1.0), (last, 2.0)]);
            let middle = at(stamp(tick + 1));

            assert!(
                lined(held.at(at(last), at(last), &GapRule::Hold(Some(three_periods)))).is_some(),
                "held at tick {tick}"
            );
            assert!(
                lined(blended.at(
                    middle,
                    at(last),
                    &GapRule::Interpolate(blend, three_periods)
                ))
                .is_some(),
                "blended at tick {tick}"
            );
        }
    }
}
