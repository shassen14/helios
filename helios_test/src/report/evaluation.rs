//! _Post-step._ [`EvaluationReport`]: what one evaluation derived over a
//! run, and from what; what it was bound to, in a run aborted before it
//! derived; or why the run refused it.

use crate::{
    assertion::AssertionTarget,
    evaluation::{derived::Coverage, kind::Cap},
    report::ReportedUnit,
};

use helios_runtime::port::{Provenance, Writer};

use serde::Serialize;

/// _Post-step._ One evaluation's line in the report: which it is, and what
/// became of it under `status`.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct EvaluationReport {
    /// The evaluation's name: an inline entry's own, or a suite's entry as
    /// `agent.<agent>.<application>.<entry>`, so it is unique in the report.
    pub name: String,
    /// The name the run file applied the entry's suite under, as `name`
    /// spells it; `None` for an inline entry.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub application: Option<String>,
    /// The agent a suite's entry was bound to; `None` for an inline entry.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub agent: Option<String>,
    /// The kind its entry names.
    pub kind: String,
    #[serde(flatten)]
    pub outcome: EvaluationOutcome,
}

impl EvaluationReport {
    /// What it derived; `None` for one the run didn't derive.
    pub fn derived(&self) -> Option<&DerivedEvaluation> {
        match &self.outcome {
            EvaluationOutcome::Derived(derived) => Some(derived),
            EvaluationOutcome::Underived(_) | EvaluationOutcome::Unusable { .. } => None,
        }
    }

    /// Whether the run refused it.
    pub fn is_unusable(&self) -> bool {
        matches!(self.outcome, EvaluationOutcome::Unusable { .. })
    }
}

/// _Post-step._ What became of an evaluation: written as its `status`
/// beside what it holds.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case", tag = "status")]
pub enum EvaluationOutcome {
    /// Derived over the run.
    Derived(DerivedEvaluation),
    /// Bound at setup, in a run aborted before it derived anything: what it
    /// would have read and offered, and no count, since none was taken.
    Underived(BoundEvaluation),
    /// Refused at setup, for every reason why.
    Unusable { problems: Vec<String> },
}

/// _Post-step._ What an evaluation derived: how many base values the run
/// gave, how many some input didn't cover and how many it couldn't compute,
/// beside what it read, with each metric's series' samples.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DerivedEvaluation {
    /// Base values stamped within the run, one sample of every metric each
    /// unless uncovered.
    pub samples: usize,
    /// Base values the evaluation couldn't compute, such as poses in roots
    /// nothing relates, or between two truth poses it can't blend. Any one
    /// fails the run, under `[recording]`.
    #[serde(skip_serializing_if = "is_zero")]
    pub refused: u64,
    /// Base values some input had no value for, each input's by where they
    /// fell under [`InputReport`]. Reported, never failed.
    pub uncovered: usize,
    #[serde(flatten)]
    pub bound: BoundEvaluation,
}

/// _Post-step._ What setup bound an evaluation to: the parameters it was
/// given, the series each role reads and who writes it, and every metric it
/// offers.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BoundEvaluation {
    /// The entry's `[evaluate.params]`, as the run file gave them: what
    /// shaped the numbers, such as a frame policy.
    #[serde(skip_serializing_if = "toml::Table::is_empty")]
    pub params: toml::Table,
    /// What binding fixed beside the parameters that shapes the numbers,
    /// in its own table, so no key of it can shadow one of the entry's:
    /// under `align = "declared"`, `roots`, how the two roots relate.
    #[serde(skip_serializing_if = "toml::Table::is_empty")]
    pub binding: toml::Table,
    /// In role order, the base first.
    pub inputs: Vec<InputReport>,
    pub metrics: Vec<MetricReport>,
}

/// _Post-step._ One role's input: the selector the run file gave, or a
/// suite's agent series and the selector it was bound to, the instance it
/// named, who wrote it, its cap and how it covered the base:
/// another role's cap is the widest gap it spanned, and the base values it
/// had none for are counted by where they fell (before its first value, in
/// a gap wider than its cap, after its last); the base's cap is the gap its
/// own values were found to drop out at, and its gaps are counted. A body's
/// channel says how its values were produced, so a grade against an exact
/// oracle reads apart from one against a modeled sensor.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct InputReport {
    pub role: String,
    /// The agent series a suite's entry read, which `target` is the
    /// selector of on its agent; `None` for an inline entry.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub series: Option<String>,
    pub target: AssertionTarget,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instance: Option<String>,
    pub writer: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provenance: Option<&'static str>,
    /// `None` for a role held however old.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cap: Option<Cap>,
    /// Values the evaluation refused. Any one fails the run, under
    /// `[recording]`.
    #[serde(skip_serializing_if = "is_zero")]
    pub refused: u64,
    /// `None` when the evaluation derived nothing for its roles.
    #[serde(flatten, skip_serializing_if = "Option::is_none")]
    pub coverage: Option<Coverage>,
}

impl InputReport {
    /// `writer` as the report spells it, with a body channel's provenance.
    pub fn writer_of(writer: Option<&Writer>) -> (String, Option<&'static str>) {
        match writer {
            Some(Writer::Body { provenance, .. }) => (
                writer_spelled(writer),
                Some(provenance_spelled(*provenance)),
            ),
            _ => (writer_spelled(writer), None),
        }
    }
}

/// _Post-step._ One metric the evaluation offers: its path, so a reader
/// finds the entries judging it or names it in the next run, its unit, and
/// how many samples its series holds.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MetricReport {
    pub path: String,
    pub unit: ReportedUnit,
    /// `None` for a metric not derived: one nothing read, or any in a run
    /// that derived nothing.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub samples: Option<usize>,
}

fn writer_spelled(writer: Option<&Writer>) -> String {
    match writer {
        Some(Writer::Body { unit, .. }) => format!("body unit {unit}"),
        Some(Writer::Node { node }) => format!("node {node}"),
        Some(Writer::Outside) => "outside".to_owned(),
        None => "unknown".to_owned(),
    }
}

fn provenance_spelled(provenance: Provenance) -> &'static str {
    match provenance {
        Provenance::Exact => "exact",
        Provenance::Simulated => "simulated",
    }
}

fn is_zero(count: &u64) -> bool {
    *count == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_body_writer_carries_its_provenance_and_a_node_none() {
        let body = Writer::Body {
            unit: "truth".to_owned(),
            provenance: Provenance::Exact,
        };
        let node = Writer::Node {
            node: "reference_arbiter".to_owned(),
        };

        assert_eq!(
            InputReport::writer_of(Some(&body)),
            ("body unit truth".to_owned(), Some("exact"))
        );
        assert_eq!(
            InputReport::writer_of(Some(&node)),
            ("node reference_arbiter".to_owned(), None)
        );
    }
}
