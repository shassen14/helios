//! _Post-step._ [`EvaluationReportEntry`]: what one evaluation derived over
//! a run, and from what.

use crate::{assertion::AssertionTarget, evaluation::Align, report::ReportedUnit};

use helios_runtime::port::{Provenance, Writer};

use serde::Serialize;

/// _Post-step._ One evaluation's report: its kind and frame policy, the
/// series each role read and who wrote it, how many base values the run
/// gave and how many no held input covered yet, and each metric's series.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct EvaluationReportEntry {
    pub name: String,
    pub kind: String,
    pub align: Align,
    /// Base values stamped within the run, one sample of every metric each
    /// unless uncovered.
    pub samples: usize,
    /// Base values before any held input had one: start-up, until the
    /// first command or goal. Reported, never failed.
    pub uncovered: usize,
    pub inputs: Vec<InputReport>,
    pub metrics: Vec<MetricReport>,
}

/// _Post-step._ One role's input: the selector the run file gave, the
/// instance it named, and who wrote it. A body's channel says how its
/// values were produced, so a grade against an exact oracle reads apart
/// from one against a modeled sensor.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct InputReport {
    pub role: String,
    pub target: AssertionTarget,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instance: Option<String>,
    pub writer: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provenance: Option<&'static str>,
    /// Values the evaluation refused. Any one fails the run, under
    /// `[recording]`.
    #[serde(skip_serializing_if = "is_zero")]
    pub refused: u64,
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

/// _Post-step._ One metric's derived series: its path, so a reader finds the
/// entries judging it, its unit and how many samples it holds.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MetricReport {
    pub path: String,
    pub unit: ReportedUnit,
    pub samples: usize,
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
