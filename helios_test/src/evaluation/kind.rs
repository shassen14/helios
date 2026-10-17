//! What an evaluator kind declares and builds. A kind is code someone adds
//! to the registry: it names its input roles, how each lines up in time with
//! the others, the metrics it derives and their units, and builds one
//! [`Evaluator`] per run-file entry. Storing what arrives, lining it up and
//! cutting it into series is the machinery's, the same for every kind.

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::AgentId;

use std::{
    any::{Any, TypeId},
    fmt,
};

/// _Pre-step._ An evaluator kind, registered under its name. Checked against
/// a run file at load, and asked at setup for the evaluator of one entry.
pub trait EvaluatorKind: Send + Sync {
    /// The name a run file's `kind` gives.
    fn name(&self) -> &'static str;

    /// Its input roles, exactly one of them [`Alignment::Base`]. Each entry
    /// names one series for every role.
    fn roles(&self) -> Vec<Role>;

    /// The metrics it derives, in the order [`Evaluator::compute`] returns
    /// them: each base sample gives one value of each.
    fn metrics(&self) -> &'static [Metric];

    /// Whether `params`, an entry's `[evaluate.params]`, are ones this kind
    /// reads, and what is wrong with them if not. Checked at load, so a
    /// misspelled parameter stops the run before a sim starts.
    fn check_params(&self, params: &toml::Table) -> Result<(), String>;

    /// The evaluator of one entry, from its `params` and the series each
    /// role was bound to (in [`roles`](Self::roles) order), or every reason
    /// those series can't be compared: inputs whose frames the kind can't
    /// relate are refused here, before the first tick.
    fn build(
        &self,
        params: &toml::Table,
        inputs: &[BoundInput],
    ) -> Result<Box<dyn Evaluator>, Vec<String>>;
}

/// _Per-step._ One entry's evaluation: what it keeps of each value that
/// arrives, and its metrics at one base sample.
pub trait Evaluator: Send + Sync {
    /// What a value of `role` keeps, read off its bare `payload` as it
    /// arrives: a small value, so the payload itself is dropped. Refused
    /// when it isn't a value the evaluation can compare, such as a velocity
    /// expressed in another frame.
    fn project(&self, role: usize, payload: &dyn Any) -> Result<Projection, InputRejection>;

    /// Each metric's value, in [`EvaluatorKind::metrics`] order, from every
    /// role's projection lined up at one base sample, in role order.
    fn compute(&self, aligned: &[&Projection]) -> Vec<f64>;
}

/// _Per-step._ What an [`Evaluator`] keeps of one value. Its type is the
/// evaluator's own, which alone reads it back.
pub type Projection = Box<dyn Any + Send + Sync>;

/// _Pre-step._ One input a kind reads: its name in `[evaluate.inputs]`, the
/// payload type its series must carry, and how it lines up in time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Role {
    pub name: &'static str,
    pub payload: TypeId,
    pub payload_name: &'static str,
    pub alignment: Alignment,
}

impl Role {
    /// The role `name`, reading values of type `T`.
    pub fn of<T: 'static>(name: &'static str, alignment: Alignment) -> Self {
        Self {
            name,
            payload: TypeId::of::<T>(),
            payload_name: std::any::type_name::<T>(),
            alignment,
        }
    }
}

/// _Pre-step._ How a role's values line up with the base's. A fact about
/// the input, so the kind declares it and a run file never does: a goal or
/// a command holds until the next one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Alignment {
    /// Its stamps are the metrics' stamps: each of its values is one sample
    /// of every metric.
    Base,
    /// The latest value stamped at or before the base sample. A base sample
    /// with none yet is uncovered.
    Hold,
}

/// _Pre-step._ One metric a kind derives, and its unit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Metric {
    pub name: &'static str,
    pub si_unit: SiUnit,
}

/// _Pre-step._ The series setup bound one role to: whose it is, and which
/// instance of a keyed channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoundInput {
    pub agent: AgentId,
    pub instance: Option<String>,
}

/// _Per-step._ Why an evaluator refused a value that arrived. A kind's own
/// words, fixed when it is written, so a refusal is copied freely.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputRejection(pub &'static str);

impl fmt::Display for InputRejection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}
