//! What an evaluator kind declares and builds, erased: the form the
//! machinery stores and feeds. A kind is code someone adds to the registry:
//! it names its input roles, how each lines up in time with the others, and
//! builds one [`Evaluator`] per run-file entry, which names the metrics it
//! offers. Storing what arrives, lining it up and cutting it into series is
//! the machinery's, the same for every kind. A kind is usually written typed
//! ([`TypedKind`](super::typed::TypedKind)) and erased once by
//! [`Typed`](super::typed::Typed); these traits stay open to one that needs
//! something else.

use crate::evaluation::derived::Refused;

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

    /// The evaluator of one entry, from its `params` (ones
    /// [`check_params`](Self::check_params) accepted) and the series each
    /// role was bound to (in [`roles`](Self::roles) order), or every reason
    /// those series can't be compared: inputs whose frames the kind can't
    /// relate are refused here, before the first tick.
    fn build(
        &self,
        params: &toml::Table,
        inputs: &[BoundInput],
    ) -> Result<Box<dyn Evaluator>, Vec<String>>;
}

/// _Per-step._ One entry's evaluation: the metrics it offers, what it keeps
/// of each value that arrives, how a kept value lies between two others,
/// and its metrics over the run once the roles are lined up.
pub trait Evaluator: Send + Sync {
    /// The metrics it offers, each a derived series a run file can name.
    /// Indices into this list are what [`grade`](Self::grade) is asked for.
    fn metrics(&self) -> Vec<Metric>;

    /// What a value of `role` keeps, read off its bare `payload` as it
    /// arrives: a small value, so the payload itself is dropped. Refused
    /// when it isn't a value the evaluation can compare, such as a velocity
    /// expressed in another frame.
    fn project(&self, role: usize, payload: &dyn Any) -> Result<Projection, InputRejection>;

    /// The value of `role` `fraction` of the way from `before` to `after`,
    /// two of its projections: how its type interpolates, so a kind with an
    /// [`Alignment::Interpolate`] role says. `None` when the two can't be
    /// blended (a pose whose root changed between them), which leaves the
    /// base sample uncovered. A kind with no such role never gets here.
    fn interpolate(
        &self,
        _role: usize,
        _before: &Projection,
        _after: &Projection,
        _fraction: f64,
    ) -> Option<Projection> {
        None
    }

    /// The `wanted` metrics, indices into [`metrics`](Self::metrics), over
    /// the `run`: every covered base sample's projections, in stamp order,
    /// each lined up in role order. `until` is the run's last tick, where a
    /// metric of the whole run is stamped. Only what is wanted is derived.
    fn grade(&self, run: &[Row<'_>], wanted: &[usize], until: MonotonicTime) -> Graded;
}

/// _Per-step._ One covered base sample: its stamp, and every role's
/// projection lined up there, in role order.
pub struct Row<'a> {
    pub at: MonotonicTime,
    pub aligned: Vec<&'a Projection>,
}

/// _Per-step._ What an [`Evaluator`] graded over a run: each metric's
/// values, `None` for one not wanted, and the samples it couldn't compute.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Graded {
    /// One per metric, in [`Evaluator::metrics`] order.
    pub metrics: Vec<Option<Vec<(MonotonicTime, f64)>>>,
    pub refused: Refused,
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
    /// The value at the base sample's stamp, between the two values that
    /// bracket it, as [`Evaluator::interpolate`] blends them. Never
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
