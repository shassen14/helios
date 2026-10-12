use crate::{assertion::AssertionValue, collector::Sample};

use helios_core::prelude::MonotonicTime;

/// _Per-step._ The running verdict of a *continuous* assertion — one that must hold every
/// tick, not just at the end. It's a latch: once `FailedAt`, it stays there,
/// because a single violation condemns the whole run even if the value later
/// recovers.
#[derive(Debug, Clone, PartialEq)]
pub enum ContinuousStatus {
    /// No observation yet — nothing to judge.
    Pending,
    /// Held on every tick observed so far.
    Holding,
    /// Violated at time `t`; `reason` is captured for the report. Terminal,
    /// never left once entered.
    FailedAt { t: MonotonicTime, reason: String },
}

/// _Per-step._ Per-assertion runtime state the runner carries across ticks. `last_value`
/// caches the most recent observation for the report; `continuous` is the latch
/// above; `reached` counts the samples of the assertion's series the run has
/// reached. Terminal assertions reuse this type but leave `continuous` as
/// `Pending` forever — they're judged once at finalize, not per tick — so the
/// runner can hold one uniform collection instead of two.
#[derive(Debug, Clone, PartialEq)]
pub struct AssertionState {
    last_value: Option<AssertionValue>,
    continuous: ContinuousStatus,
    reached: usize,
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
            last_value: None,
            continuous: ContinuousStatus::Pending,
            reached: 0,
        }
    }

    /// Fold one observation into the continuous status. `record` does **not**
    /// judge whether the value satisfies the condition — the evaluator already
    /// did that and handed the verdict in via `outcome` (`Ok` = held this tick,
    /// `Err(reason)` = failed, with a human-readable reason). This method only
    /// caches the value and applies the latch, so the condition logic stays in
    /// one place. `value` is `Some` whenever an observation was extracted (every
    /// pass, and most failures); it's `None` for a failure that produced no
    /// comparable value (a rejected value), which still latches but leaves
    /// `last_value` untouched. The runner skips `record` entirely on a `Pending`
    /// observation, leaving the status alone.
    pub fn record(
        &mut self,
        value: Option<AssertionValue>,
        outcome: Result<(), String>,
        t: MonotonicTime,
    ) {
        // Always cache the latest reading so the report can show it, regardless
        // of pass/fail or whether the status is already latched.
        if value.is_some() {
            self.last_value = value;
        }

        match outcome {
            Ok(()) => {
                // Held this tick: advance Pending -> Holding. The guard makes
                // the latch one-way — once FailedAt, a later passing tick must
                // NOT pardon the run, so we leave it failed.
                if !matches!(self.continuous, ContinuousStatus::FailedAt { .. }) {
                    self.continuous = ContinuousStatus::Holding;
                }
            }
            Err(reason) => {
                // Only the FIRST failure latches. The same guard means a later
                // failure is ignored, so `FailedAt` keeps the original `t` and
                // `reason` — the report wants when it first broke, not the most
                // recent stumble.
                if !matches!(self.continuous, ContinuousStatus::FailedAt { .. }) {
                    self.continuous = ContinuousStatus::FailedAt { t, reason };
                }
            }
        }
    }

    /// The samples of `samples` that arrived since the last call, which the
    /// run has now reached. `samples` is the assertion's whole series, which
    /// only grows.
    pub fn reach_new<'s>(&mut self, samples: &'s [Sample]) -> &'s [Sample] {
        let new = samples.get(self.reached..).unwrap_or_default();
        self.reached = samples.len();
        new
    }

    /// The most recent observed value, if any has been recorded yet.
    pub fn last_value(&self) -> &Option<AssertionValue> {
        &self.last_value
    }

    /// The current latch state.
    pub fn status(&self) -> &ContinuousStatus {
        &self.continuous
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

        assert_eq!(s.reach_new(&series[..2]), &series[..2]);
        assert_eq!(s.reach_new(&series[..2]), &[]);
        assert_eq!(s.reach_new(&series), &series[2..]);
    }

    #[test]
    fn samples_past_the_last_reach_are_not_reached() {
        // A sample arriving after the run's last tick is past its end.
        let mut s = AssertionState::new();
        let series = [sample(1.0, 1.0), sample(2.0, 2.0)];

        s.reach_new(&series[..1]);

        assert_eq!(s.reached(&series), &series[..1]);
    }

    #[test]
    fn starts_pending_with_no_value() {
        let s = AssertionState::new();
        assert_eq!(s.status(), &ContinuousStatus::Pending);
        assert_eq!(s.last_value(), &None);
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
        s.record(Some(val(9.0)), Err("over limit".into()), at(3.0));
        assert_eq!(
            s.status(),
            &ContinuousStatus::FailedAt {
                t: at(3.0),
                reason: "over limit".into(),
            }
        );
    }

    #[test]
    fn recovery_does_not_pardon_a_failure() {
        // The point of the latch: a passing tick after a failure must NOT
        // overwrite FailedAt back to Holding.
        let mut s = AssertionState::new();
        s.record(Some(val(9.0)), Err("over limit".into()), at(3.0));
        s.record(Some(val(1.0)), Ok(()), at(4.0));
        assert_eq!(
            s.status(),
            &ContinuousStatus::FailedAt {
                t: at(3.0),
                reason: "over limit".into(),
            }
        );
    }

    #[test]
    fn second_failure_keeps_the_first_time_and_reason() {
        // Only the first violation latches; a later failure is ignored so the
        // report shows when it first broke.
        let mut s = AssertionState::new();
        s.record(Some(val(9.0)), Err("first".into()), at(3.0));
        s.record(Some(val(8.0)), Err("second".into()), at(5.0));
        assert_eq!(
            s.status(),
            &ContinuousStatus::FailedAt {
                t: at(3.0),
                reason: "first".into(),
            }
        );
    }

    #[test]
    fn last_value_updates_even_when_latched_failed() {
        // record caches the latest observed reading, regardless of status.
        let mut s = AssertionState::new();
        s.record(Some(val(9.0)), Err("over limit".into()), at(3.0));
        s.record(Some(val(1.0)), Ok(()), at(4.0));
        assert_eq!(s.last_value(), &Some(val(1.0)));
    }

    #[test]
    fn valueless_failure_latches_but_leaves_last_value() {
        // A failure with no comparable value (a rejected one) still condemns
        // the run, but there's nothing to cache — last_value stays as it was.
        let mut s = AssertionState::new();
        s.record(Some(val(2.0)), Ok(()), at(1.0));
        s.record(None, Err("no extractor".into()), at(2.0));
        assert_eq!(
            s.status(),
            &ContinuousStatus::FailedAt {
                t: at(2.0),
                reason: "no extractor".into(),
            }
        );
        assert_eq!(s.last_value(), &Some(val(2.0)));
    }
}
