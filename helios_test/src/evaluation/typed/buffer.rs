//! [`RoleBuffer`]: one role's kept values, and how they line up at the
//! base's stamps. The one place a payload is downcast, and the one place a
//! role's values are lined up.

use crate::evaluation::kind::InputRejection;

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

    /// Its value at the base stamp `at` as `lining` says, from its values up
    /// to `until`, or `None` where it has none: before its first value, or,
    /// interpolated, after its last or between two it can't blend. Never
    /// extrapolated.
    pub(super) fn at(
        &self,
        at: MonotonicTime,
        until: MonotonicTime,
        lining: &Lining<K>,
    ) -> Option<Lined<'_, K>> {
        let values = self.up_to(until);
        match lining {
            Lining::Hold => {
                let after = values.partition_point(|(stamp, _)| stamp.0 <= at.0);
                let (_, held) = values.get(after.checked_sub(1)?)?;
                Some(Lined::Kept(held))
            }
            Lining::Interpolate(blend) => {
                let after = values.partition_point(|(stamp, _)| stamp.0 < at.0);
                let (after_at, after_value) = values.get(after)?;
                if after_at.0 == at.0 {
                    return Some(Lined::Kept(after_value));
                }
                let (before_at, before_value) = values.get(after.checked_sub(1)?)?;
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

/// _Pre-step._ How a role other than the base lines up at each base stamp.
pub enum Lining<K> {
    /// The value in force: a goal, a command.
    Hold,
    /// Blended from the two values around the stamp by the function given:
    /// the value `fraction` of the way from the first to the second, `None`
    /// when the two can't be blended.
    Interpolate(fn(&K, &K, f64) -> Option<K>),
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
            .map(|t| lined(buffer.at(at(t), at(10.0), &Lining::Hold)))
            .into();

        assert_eq!(
            at_stamps,
            vec![None, Some(10.0), Some(10.0), Some(30.0), Some(30.0)]
        );
    }

    #[test]
    fn an_interpolated_value_is_blended_between_its_neighbors_and_never_extrapolated() {
        let buffer = buffer(&[(1.0, 10.0), (3.0, 30.0)]);
        let unblendable = Lining::Interpolate(|_, _, _| None);

        let at_stamps: Vec<_> = [0.5, 1.0, 2.5, 3.0, 4.0]
            .map(|t| lined(buffer.at(at(t), at(10.0), &Lining::Interpolate(blend))))
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
}
