use crate::{
    assertion::{AssertionValue, FailureKind},
    collector::{Rejection, Sample},
};

use helios_core::prelude::MonotonicTime;

/// _Per-step._ The running verdict of an assertion that judges every sample
/// as it arrives — one that must hold throughout its window, not just at the
/// end. It's a latch: once `FailedAt`, it stays there, because a single
/// violation condemns the whole run even if the value later recovers.
///
/// It decides only whether `fail_fast` stops the run. The report's verdict
/// is judged again at finalize from the window's samples, which places a
/// failure at the earliest failing stamp rather than the first failure to
/// arrive.
#[derive(Debug, Clone, PartialEq)]
pub enum ContinuousStatus {
    /// No observation yet — nothing to judge.
    Pending,
    /// Held on every tick observed so far.
    Holding,
    /// Violated at time `t` by `value` (`None` when the failure had no
    /// comparable value, a rejected one), for the reason `kind`. Terminal,
    /// never left once entered.
    FailedAt {
        t: MonotonicTime,
        value: Option<AssertionValue>,
        kind: FailureKind,
    },
}

/// _Per-step._ Per-assertion runtime state the runner carries across ticks.
/// `continuous` is the latch above; `reached` counts the samples of the
/// entry's series the run has reached, and `rejection` is the series'
/// rejection once the run has reached it; `carried_in` is whether the value
/// a held series carries into the window has been judged. Assertions judged
/// only at finalize, and measures, reuse this type but leave `continuous` as
/// `Pending` forever, so the runner can hold one uniform collection.
#[derive(Debug, Clone, PartialEq)]
pub struct AssertionState {
    continuous: ContinuousStatus,
    reached: usize,
    rejection: Option<Rejection>,
    carried_in: bool,
}

impl Default for AssertionState {
    fn default() -> Self {
        Self::new()
    }
}

impl AssertionState {
    /// Starting state for every assertion: nothing observed, status `Pending`.
    pub fn new() -> Self {
        Self {
            continuous: ContinuousStatus::Pending,
            reached: 0,
            rejection: None,
            carried_in: false,
        }
    }

    /// Fold one observation into the continuous status. `record` does **not**
    /// judge whether the value satisfies the condition — the evaluator already
    /// did that and handed the verdict in via `outcome` (`Ok` = held this tick,
    /// `Err(kind)` = failed, for that reason). This method only applies the
    /// latch, so the condition logic stays in one place. `value` is the
    /// failing value, `None` for a failure that produced no comparable value
    /// (a rejected value). The runner skips `record` entirely on a `Pending`
    /// observation, leaving the status alone.
    pub fn record(
        &mut self,
        value: Option<AssertionValue>,
        outcome: Result<(), FailureKind>,
        t: MonotonicTime,
    ) {
        match outcome {
            Ok(()) => {
                // Held this tick: advance Pending -> Holding. The guard makes
                // the latch one-way — once FailedAt, a later passing tick must
                // NOT pardon the run, so we leave it failed.
                if !matches!(self.continuous, ContinuousStatus::FailedAt { .. }) {
                    self.continuous = ContinuousStatus::Holding;
                }
            }
            Err(kind) => {
                // Only the FIRST failure latches. The same guard means a later
                // failure is ignored, so `FailedAt` keeps the original `t` and
                // `kind` — the report wants when it first broke, not the most
                // recent stumble.
                if !matches!(self.continuous, ContinuousStatus::FailedAt { .. }) {
                    self.continuous = ContinuousStatus::FailedAt { t, value, kind };
                }
            }
        }
    }

    /// Whether the value carried into the window is still to be judged, and
    /// from now on that it has been: true once, on the first call.
    pub fn carry_in_once(&mut self) -> bool {
        !std::mem::replace(&mut self.carried_in, true)
    }

    /// The samples of `samples` that arrived since the last call, which the
    /// run has now reached. `samples` is the assertion's whole series, which
    /// only grows. The series' `rejection`, if it has one, is reached with
    /// them: a value turned away after the run's last tick is past its end,
    /// like a sample.
    pub fn reach_new<'s>(
        &mut self,
        samples: &'s [Sample],
        rejection: Option<Rejection>,
    ) -> &'s [Sample] {
        let new = samples.get(self.reached..).unwrap_or_default();
        self.reached = samples.len();
        self.rejection = self.rejection.or(rejection);
        new
    }

    /// The current latch state.
    pub fn status(&self) -> &ContinuousStatus {
        &self.continuous
    }

    /// Why a value of the series was turned away, if the run reached one.
    pub fn rejection(&self) -> Option<Rejection> {
        self.rejection
    }

    /// The samples of `samples` the run has reached. Samples that arrive
    /// after the run's last tick are past its end and never judged.
    pub fn reached<'s>(&self, samples: &'s [Sample]) -> &'s [Sample] {
        &samples[..self.reached.min(samples.len())]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(secs: f64) -> MonotonicTime {
        MonotonicTime(secs)
    }

    fn val(x: f64) -> AssertionValue {
        AssertionValue::Float(x)
    }

    fn sample(at_secs: f64, x: f64) -> Sample {
        Sample {
            timestamp: at(at_secs),
            value: val(x),
        }
    }

    #[test]
    fn each_sample_is_new_once() {
        let mut s = AssertionState::new();
        let series = [sample(1.0, 1.0), sample(2.0, 2.0), sample(3.0, 3.0)];

        assert_eq!(s.reach_new(&series[..2], None), &series[..2]);
        assert_eq!(s.reach_new(&series[..2], None), &[]);
        assert_eq!(s.reach_new(&series, None), &series[2..]);
    }

    #[test]
    fn samples_past_the_last_reach_are_not_reached() {
        // A sample arriving after the run's last tick is past its end.
        let mut s = AssertionState::new();
        let series = [sample(1.0, 1.0), sample(2.0, 2.0)];

        s.reach_new(&series[..1], None);

        assert_eq!(s.reached(&series), &series[..1]);
    }

    #[test]
    fn starts_pending() {
        let s = AssertionState::new();
        assert_eq!(s.status(), &ContinuousStatus::Pending);
    }

    #[test]
    fn the_carried_in_value_is_judged_once() {
        let mut s = AssertionState::new();
        assert!(s.carry_in_once());
        assert!(!s.carry_in_once());
    }

    #[test]
    fn first_passing_tick_advances_to_holding() {
        let mut s = AssertionState::new();
        s.record(Some(val(1.0)), Ok(()), at(0.0));
        assert_eq!(s.status(), &ContinuousStatus::Holding);
    }

    #[test]
    fn stays_holding_while_passing() {
        let mut s = AssertionState::new();
        s.record(Some(val(1.0)), Ok(()), at(0.0));
        s.record(Some(val(2.0)), Ok(()), at(1.0));
        assert_eq!(s.status(), &ContinuousStatus::Holding);
    }

    #[test]
    fn failure_latches_with_time_and_reason() {
        let mut s = AssertionState::new();
        s.record(Some(val(1.0)), Ok(()), at(0.0));
        s.record(Some(val(9.0)), Err(FailureKind::ConditionFailed), at(3.0));
        assert_eq!(
            s.status(),
            &ContinuousStatus::FailedAt {
                t: at(3.0),
                value: Some(val(9.0)),
                kind: FailureKind::ConditionFailed,
            }
        );
    }

    #[test]
    fn recovery_does_not_pardon_a_failure() {
        // The point of the latch: a passing tick after a failure must NOT
        // overwrite FailedAt back to Holding.
        let mut s = AssertionState::new();
        s.record(Some(val(9.0)), Err(FailureKind::ConditionFailed), at(3.0));
        s.record(Some(val(1.0)), Ok(()), at(4.0));
        assert_eq!(
            s.status(),
            &ContinuousStatus::FailedAt {
                t: at(3.0),
                value: Some(val(9.0)),
                kind: FailureKind::ConditionFailed,
            }
        );
    }

    #[test]
    fn second_failure_keeps_the_first_time_and_reason() {
        // Only the first violation latches; a later failure is ignored so the
        // report shows when it first broke.
        let mut s = AssertionState::new();
        s.record(Some(val(9.0)), Err(FailureKind::ConditionFailed), at(3.0));
        s.record(Some(val(8.0)), Err(FailureKind::TypeMismatch), at(5.0));
        assert_eq!(
            s.status(),
            &ContinuousStatus::FailedAt {
                t: at(3.0),
                value: Some(val(9.0)),
                kind: FailureKind::ConditionFailed,
            }
        );
    }

    #[test]
    fn valueless_failure_latches() {
        // A failure with no comparable value (a rejected one) still condemns
        // the run.
        let mut s = AssertionState::new();
        s.record(Some(val(2.0)), Ok(()), at(1.0));
        s.record(
            None,
            Err(FailureKind::Rejected(Rejection::UnsupportedKind)),
            at(2.0),
        );
        assert_eq!(
            s.status(),
            &ContinuousStatus::FailedAt {
                t: at(2.0),
                value: None,
                kind: FailureKind::Rejected(Rejection::UnsupportedKind),
            }
        );
    }
}
