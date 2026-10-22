//! The evaluations the collector feeds: each is handed the values recorded
//! under its input keys, and derives its series from them at finalize.

use super::{CollectorId, ObservationCollector};
use crate::evaluation::{
    derived::{Derived, Refused},
    kind::Evaluator,
};

use helios_core::prelude::MonotonicTime;
use helios_runtime::observe::{
    key::RecordedKey,
    observation::{Observation, ObservedValue},
};

impl ObservationCollector {
    /// Feeds `evaluation` the values recorded under each of `inputs`, one
    /// key per role in role order. Bound before the first tick, as a series
    /// is.
    pub fn bind_evaluation(
        &mut self,
        evaluation: Box<dyn Evaluator>,
        inputs: Vec<RecordedKey>,
    ) -> EvaluationId {
        let index = self.evaluations.len();
        self.evaluations.push(evaluation);
        for (role, key) in inputs.into_iter().enumerate() {
            self.inputs.entry(key).or_default().push((index, role));
        }
        EvaluationId {
            collector: self.id,
            index,
        }
    }

    /// Each role's refusals so far in the evaluation `id` names. `None` when
    /// `id` came from another collector.
    pub fn refused(&self, id: EvaluationId) -> Option<&[Refused]> {
        self.evaluation(id).map(|evaluation| evaluation.refused())
    }

    /// What the evaluation `id` names derived of its `wanted` metrics from
    /// the values stamped at or before `until`. `None` when `id` came from
    /// another collector.
    pub fn derive(
        &self,
        id: EvaluationId,
        until: MonotonicTime,
        wanted: &[usize],
    ) -> Option<Derived> {
        self.evaluation(id)
            .map(|evaluation| evaluation.derive(until, wanted))
    }

    /// Hands `observation`'s payload to each evaluation reading its key.
    /// Whether any does.
    pub(super) fn feed(&mut self, observation: &Observation) -> bool {
        let Some(inputs) = self.inputs.get(&observation.key) else {
            return false;
        };
        // Setup binds an input only to a channel of the role's payload
        // type, so a value there is always typed.
        if let ObservedValue::Typed(stamped) = &observation.value {
            for (evaluation, role) in inputs {
                if let Some(evaluation) = self.evaluations.get_mut(*evaluation) {
                    evaluation.receive(*role, observation.timestamp, stamped.payload());
                }
            }
        }
        true
    }

    fn evaluation(&self, id: EvaluationId) -> Option<&dyn Evaluator> {
        if id.collector != self.id {
            return None;
        }
        self.evaluations.get(id.index).map(Box::as_ref)
    }
}

/// _Per-step._ Names one evaluation in one collector, handed out by
/// [`ObservationCollector::bind_evaluation`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EvaluationId {
    collector: CollectorId,
    index: usize,
}
