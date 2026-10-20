//! What an evaluator kind declares and builds, erased: the form the
//! machinery stores and feeds. A kind is code someone adds to the registry:
//! it names its input roles, how each lines up in time with the others, and
//! builds one [`Evaluator`] per run-file entry, which names the metrics it
//! offers. An evaluation is untyped only here, where the collector hands it
//! each payload of a stream that carries every type. A kind is usually
//! written typed ([`TypedKind`](super::typed::TypedKind)) and erased once by
//! [`Typed`](super::typed::Typed), whose evaluation keeps each value typed and
//! lines the roles up in time the same way for every kind; these traits stay
//! open to one that pairs its values some other way, such as by association.

use crate::evaluation::derived::{Derived, Refused};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::{AgentId, FrameId, MonotonicTime};

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

    /// The metrics every evaluator of this kind offers, when they are fixed,
    /// so load checks a derived target's metric before a sim starts. `None`
    /// when they are known only once an entry is bound, such as one per
    /// block of an estimator's state, so setup checks them.
    fn menu(&self) -> Option<Vec<Metric>>;

    /// Whether `params`, an entry's `[evaluate.params]`, are ones this kind
    /// reads, and what is wrong with them if not. Checked at load, so a
    /// misspelled parameter stops the run before a sim starts.
    fn check_params(&self, params: &toml::Table) -> Result<(), String>;

    /// The evaluation of one entry, which the collector feeds, from its
    /// `params` (ones [`check_params`](Self::check_params) accepted) and the
    /// series each role was bound to (in [`roles`](Self::roles) order), or
    /// every reason those series can't be compared: inputs whose frames the
    /// kind can't relate are refused here, before the first tick.
    fn build(
        &self,
        params: &toml::Table,
        inputs: &[BoundInput],
    ) -> Result<Box<dyn Evaluator>, Vec<String>>;
}

/// _Per-step._ One entry's evaluation, as the collector feeds it and finalize
/// reads it: the metrics it offers, each value of its inputs as it arrives,
/// and its metrics over the run. The one place an evaluation is untyped; a
/// [`Typed`](super::typed::Typed) kind's keeps every value its own type
/// behind it.
pub trait Evaluator: Send + Sync {
    /// The metrics it offers, each a derived series a run file can name.
    /// Indices into this list are what [`derive`](Self::derive) is asked for.
    fn metrics(&self) -> Vec<Metric>;

    /// A value of `role` stamped `at` arrived, its payload bare: kept as far
    /// as the evaluation needs it, or counted as refused when it isn't a
    /// value the evaluation can compare, such as a velocity expressed in
    /// another frame.
    fn receive(&mut self, role: usize, at: MonotonicTime, payload: &dyn Any);

    /// Each role's refusals so far, in role order.
    fn refused(&self) -> &[Refused];

    /// The derived series of the `wanted` metrics, indices into
    /// [`metrics`](Self::metrics), from every value stamped at or before
    /// `until`: the run's last tick, where a metric of the whole run is
    /// stamped. A metric not wanted is not derived.
    fn derive(&self, until: MonotonicTime, wanted: &[usize]) -> Derived;
}

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
    /// The value at the base sample's stamp, between the two values that
    /// bracket it, as the role's type blends them. Never
    /// extrapolated: a base sample before the first value or after the
    /// last is uncovered.
    Interpolate,
}

/// _Pre-step._ One metric an evaluator offers, and its unit.
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

impl BoundInput {
    /// The instance as a frame, for a kind whose instances are frames (a
    /// transform edge's child, a velocity's moving body). `None` when no
    /// instance was selected. A spelling that isn't a frame's is refused,
    /// naming `role` and the spelling.
    pub fn frame(&self, role: &str) -> Result<Option<FrameId>, String> {
        self.instance
            .as_deref()
            .map(|spelling| {
                spelling.parse().map_err(|why| {
                    format!("the {role}'s instance '{spelling}' is not a frame: {why}")
                })
            })
            .transpose()
    }
}

/// _Per-step._ Why an evaluator refused a value that arrived, or a base
/// sample it couldn't compute. A kind's own words, fixed when it is written,
/// so a refusal is copied freely.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputRejection(pub &'static str);

impl fmt::Display for InputRejection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}
