//! What an evaluator kind declares and builds, erased: the form the
//! machinery stores and feeds. A kind is code someone adds to the registry:
//! it names its input roles, the first of them the base, and builds one
//! [`Evaluator`] per run-file entry, which names the metrics it offers. An
//! evaluation is untyped only here, where the collector hands it each
//! payload of a stream that carries every type. A kind is usually written
//! typed, one trait per number of inputs
//! ([`TwoInputs`](super::typed::TwoInputs)), and erased once by that
//! shape's adapter, whose evaluation keeps each value typed and lines the
//! inputs up in time the same way for every kind; these traits stay open to
//! one that pairs its values some other way, such as by association.

use crate::evaluation::{
    agent_series::AgentSeries,
    config::{Gap, GapWidth},
    derived::{Derived, Refused},
};

use helios_core::estimation::schema::StateSchema;
use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::{AgentId, FrameId, MonotonicTime};
use helios_core::spatial::transforms::Convention;
use helios_runtime::{
    pipeline::FramePlacement,
    port::{ChannelKey, Writer},
};

use serde::Serialize;
use std::{
    any::{Any, TypeId},
    fmt,
    sync::Arc,
};

/// _Pre-step._ An evaluator kind, registered under its name. Checked against
/// a run file at load, and asked at setup for the evaluator of one entry.
pub trait EvaluatorKind: Send + Sync {
    /// The name a run file's `kind` gives.
    fn name(&self) -> &'static str;

    /// Its input roles, the first of them the base: each of its values is one
    /// sample of every metric. Each entry names one series for every role.
    fn roles(&self) -> Vec<Role>;

    /// The channel the role at `role` in [`roles`](Self::roles) reads when
    /// a suite binds it to `series`, or why it can't: the series carries
    /// another type, or the kind has no such role. Answered by the kind's
    /// typed form, where each role's payload type is known.
    fn series_key(&self, role: usize, series: AgentSeries) -> Result<ChannelKey, String>;

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
    /// `params` (ones [`check_params`](Self::check_params) accepted), the
    /// series each role was bound to and the gap each role's cap is made
    /// from (both in [`roles`](Self::roles) order), or every reason those
    /// series can't be compared: inputs whose frames the kind can't relate,
    /// and a role that needs a cap given none, are refused here, before the
    /// first tick.
    fn build(
        &self,
        params: &toml::Table,
        inputs: &[BoundInput],
        gaps: &[Option<Gap>],
    ) -> Result<Box<dyn Evaluator>, Vec<String>>;
}

/// _Per-step._ One entry's evaluation, as the collector feeds it and finalize
/// reads it: the metrics it offers, each value of its inputs as it arrives,
/// and its metrics over the run. The one place an evaluation is untyped; a
/// typed kind's, such as a [`TwoInputs`](super::typed::TwoInputs) kind's,
/// keeps every value its own type behind it.
pub trait Evaluator: Send + Sync {
    /// The metrics it offers, each a derived series a run file can name.
    /// Indices into this list are what [`derive`](Self::derive) is asked for.
    fn metrics(&self) -> Vec<Metric>;

    /// A value of `role` stamped `at` arrived, its payload bare: kept as far
    /// as the evaluation needs it, or counted as refused when it isn't a
    /// value the evaluation can compare, such as a velocity expressed in
    /// another frame.
    fn receive(&mut self, role: usize, at: MonotonicTime, payload: &dyn Any);

    /// Each role's cap, in role order: the base's finds its gaps, another
    /// role's is the widest gap it spans; `None` for a role held however
    /// old.
    fn caps(&self) -> Vec<Option<Cap>>;

    /// What binding fixed, beyond the entry's parameters, that shapes every
    /// value, as the report states it beside them: how two roots relate,
    /// say. Empty when nothing does.
    fn binding(&self) -> toml::Table;

    /// Each role's refusals so far, in role order.
    fn refused(&self) -> &[Refused];

    /// The derived series of the `wanted` metrics, indices into
    /// [`metrics`](Self::metrics), from every value stamped at or before
    /// `until`: the run's last tick, where a metric of the whole run is
    /// stamped. A metric not wanted is not derived.
    fn derive(&self, until: MonotonicTime, wanted: &[usize]) -> Derived;
}

/// _Pre-step._ One input a kind reads: its name in `[evaluate.inputs]`, the
/// payload type its series must carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Role {
    pub name: &'static str,
    pub payload: TypeId,
    pub payload_name: &'static str,
}

impl Role {
    /// The role `name`, reading values of type `T`.
    pub fn of<T: 'static>(name: &'static str) -> Self {
        Self {
            name,
            payload: TypeId::of::<T>(),
            payload_name: std::any::type_name::<T>(),
        }
    }
}

/// _Pre-step._ One metric an evaluator offers, and its unit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Metric {
    pub name: &'static str,
    pub si_unit: SiUnit,
}

/// _Pre-step._ The series setup bound one role to: whose it is, which
/// instance of a keyed channel, and the facts its writer declared before the
/// first value: the frame its values are expressed in, that frame's axis
/// convention and where it sits in the world, its nominal rate, and who
/// writes it.
#[derive(Debug, Clone)]
pub struct BoundInput {
    pub agent: AgentId,
    pub instance: Option<String>,
    /// The frame the series' values are expressed in, when its writer
    /// declared one apart from the values: a transform edge's parent, the
    /// root of the tree it belongs to, or the root a state is held in.
    pub expressed_in: Option<FrameId>,
    /// `expressed_in`'s axis convention, as the frame's owner gives it in the
    /// agent's pipeline. `None` when no frame is declared or no owner gives
    /// it one.
    pub convention: Option<Convention>,
    /// Where `expressed_in` sits in the world, when a node of the agent's
    /// pipeline defines it: anchored at a world frame, or free and why.
    /// `None` for a world or body frame, or when no frame is declared.
    pub placement: Option<FramePlacement>,
    /// The schema every state the series carries is built on, when its
    /// writer describes them. `None` for any other payload.
    pub schema: Option<Arc<StateSchema>>,
    /// How many times a second its writer writes it, nominally, as declared.
    pub rate_hz: Option<f64>,
    /// Who writes it, with a body writer's provenance. `None` only when no
    /// description names the series' writer.
    pub writer: Option<Writer>,
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

/// How far, as a fraction of a cap, a gap may exceed it by rounding and
/// still be spanned. A gap of exactly N periods is the difference of two
/// stamps, which rounds either side of N periods; a millionth of the cap
/// absorbs that for stamps up to about 10^7 s, and is far below any width
/// a run means.
const CAP_ROUNDING: f64 = 1e-6;

/// _Pre-step._ The widest gap a role's values span: a held value older
/// than it, or two values further apart than it to interpolate between,
/// leave the base value uncovered. The base's finds where its own values
/// dropped out, and never leaves one uncovered. Made only from a [`Gap`],
/// so it is positive.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Cap {
    /// The multiple of its input's period it came from, unless given in
    /// seconds.
    #[serde(skip_serializing_if = "Option::is_none")]
    multiple: Option<f64>,
    seconds: f64,
}

impl Cap {
    /// The cap `gap` gives `role`, reading `input`, for `used`, or why it
    /// gives none: no gap given, or a multiple of the period of an input
    /// that declares no rate.
    pub fn of(
        role: &str,
        input: &BoundInput,
        gap: Option<Gap>,
        used: CapUse,
    ) -> Result<Self, String> {
        let used = used.phrase();
        match (gap, input.rate_hz) {
            (Some(Gap::Seconds(seconds)), _) => Ok(Self::in_seconds(seconds)),
            (Some(Gap::Multiple(multiple)), Some(rate_hz)) => {
                Ok(Self::in_periods(multiple, rate_hz))
            }
            (Some(Gap::Multiple(_)), None) => Err(format!(
                "the {role} {used}, and its series declares no rate to take a \
                 multiple of: give [evaluate.gap.{role}] seconds, or read a series whose \
                 writer declares a rate"
            )),
            (None, _) => Err(format!(
                "the {role} {used}, and none is given: give [evaluation] gap_multiple in \
                 the entry's file, or [evaluate.gap.{role}] multiple or seconds"
            )),
        }
    }

    /// A cap of `seconds`.
    pub fn in_seconds(seconds: GapWidth) -> Self {
        Self {
            multiple: None,
            seconds: seconds.get(),
        }
    }

    /// A cap of `multiple` periods of an input at `rate_hz`.
    pub fn in_periods(multiple: GapWidth, rate_hz: f64) -> Self {
        Self {
            multiple: Some(multiple.get()),
            seconds: multiple.get() / rate_hz,
        }
    }

    pub fn multiple(&self) -> Option<f64> {
        self.multiple
    }

    pub fn seconds(&self) -> f64 {
        self.seconds
    }

    /// Whether a gap of `seconds` is within it. A gap of exactly the cap
    /// is, whichever way its stamps rounded.
    pub fn spans(&self, seconds: f64) -> bool {
        seconds <= self.seconds * (1.0 + CAP_ROUNDING)
    }
}

/// _Pre-step._ What a role's [`Cap`] is for, which a refusal to make one
/// names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapUse {
    /// A role other than the base spans gaps up to it.
    Spans,
    /// The base's gaps are found against it.
    FindsGaps,
}

impl CapUse {
    fn phrase(self) -> &'static str {
        match self {
            CapUse::Spans => "spans gaps up to a cap",
            CapUse::FindsGaps => "is the base, whose gaps are found against a cap",
        }
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
