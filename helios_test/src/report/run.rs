//! What [`Report::new`](super::Report::new) assembles a report from, one
//! group per report section: who the run was, how it ended, what its wiring
//! and recording dropped, and its entries.

use super::{
    AssertionReportEntry, AssertionStatus, EvaluationReport, HostFault, MeasureReportEntry,
};
use crate::run::termination::TerminationReason;

/// _Post-step._ Which run a report is of.
#[derive(Debug, Clone)]
pub struct RunIdentity {
    pub run_name: String,
    pub scenario: String,
    /// `None` for an unseeded run, whose real seed is chosen inside the sim.
    pub master_seed: Option<u64>,
}

/// _Post-step._ How long the run took and why it stopped.
#[derive(Debug, Clone)]
pub struct RunEnd {
    pub simulated_duration_secs: f64,
    pub wall_duration_secs: f64,
    pub stop: RunStop,
}

/// _Post-step._ Why the run stopped, as the runner knows it. Whether entries
/// were refused is read from the entries, so no caller states it.
#[derive(Debug, Clone)]
pub enum RunStop {
    /// The run was tested and stopped for `0`.
    Ended(TerminationReason),
    /// The runner stopped the run untested: each pipeline not built and each
    /// way the host broke its contract, in the order found. Both are empty
    /// only when some entry is unusable, the abort's one other cause.
    Aborted {
        unbuilt: Vec<UnbuiltAgent>,
        faults: Vec<HostFault>,
    },
}

/// _Post-step._ An agent whose pipeline the host couldn't build, for
/// `errors`, as the host gave them.
#[derive(Debug, Clone)]
pub struct UnbuiltAgent {
    pub agent: String,
    pub errors: Vec<String>,
}

/// _Post-step._ What the run's wiring and recording dropped, one line each.
#[derive(Debug, Clone, Default)]
pub struct DroppedValues {
    /// The writes an agent's bus refused, one line per channel.
    pub refused_writes: Vec<String>,
    /// What the recording dropped.
    pub recording: Vec<String>,
}

impl DroppedValues {
    /// Whether nothing was dropped.
    pub fn is_empty(&self) -> bool {
        self.refused_writes.is_empty() && self.recording.is_empty()
    }
}

/// _Post-step._ The run's entries, each in run-file order.
#[derive(Debug, Clone, Default)]
pub struct RunEntries {
    pub assertions: Vec<AssertionReportEntry>,
    pub measures: Vec<MeasureReportEntry>,
    pub evaluations: Vec<EvaluationReport>,
}

impl RunEntries {
    /// Whether the run refused any entry or evaluation.
    pub fn any_unusable(&self) -> bool {
        self.assertions
            .iter()
            .any(|entry| entry.status() == AssertionStatus::Unusable)
            || self.measures.iter().any(MeasureReportEntry::is_unusable)
            || self.evaluations.iter().any(EvaluationReport::is_unusable)
    }

    /// Whether every assertion passed.
    pub fn all_passed(&self) -> bool {
        self.assertions
            .iter()
            .all(|entry| entry.status() == AssertionStatus::Passed)
    }
}
