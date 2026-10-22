//! An evaluation's line: what it read, how well its inputs covered the
//! base, and which metrics it computed.

use super::super::{EvaluationReportEntry, MetricReport};
use crate::evaluation::derived::Coverage;

use helios_core::kernel::identifier::LEVEL_SEPARATOR;

/// "[evaluate] <name>: <kind>[ (<param> = <value>, …)] of <role> (<target>,
/// <writer>[, <provenance>][, gaps up to <seconds> s[ = <multiple> periods]]
/// [, <n> before its first value][, <n> in its gaps][, <n> after its last
/// value]), … (the base's cap reads "<n> gaps past <seconds> s")
/// … — <samples> samples[, <n> uncovered][, <refused> not computed]", then
/// each metric's path and unit. The parameters and each count but the
/// base's gaps show only when nonzero.
pub(super) fn evaluation_line(evaluation: &EvaluationReportEntry) -> String {
    let inputs: Vec<String> = evaluation
        .inputs
        .iter()
        .enumerate()
        .map(|(role, input)| {
            let instance = input
                .instance
                .as_ref()
                .map(|instance| format!("[{instance}]"))
                .unwrap_or_default();
            let provenance = input
                .provenance
                .map(|provenance| format!(", {provenance}"))
                .unwrap_or_default();
            let cap = input
                .cap
                .map(|cap| {
                    let multiple = cap
                        .multiple()
                        .map(|multiple| format!(" = {multiple} periods"))
                        .unwrap_or_default();
                    let reads = match input.coverage {
                        Some(Coverage::Base { gaps: 1 }) => "1 gap past".to_owned(),
                        Some(Coverage::Base { gaps: 0 }) => "no gaps past".to_owned(),
                        Some(Coverage::Base { gaps }) => format!("{gaps} gaps past"),
                        _ if role == 0 => "its gaps found past".to_owned(),
                        _ => "gaps up to".to_owned(),
                    };
                    format!(", {reads} {} s{multiple}", cap.seconds())
                })
                .unwrap_or_default();
            let mut uncovered = String::new();
            if let Some(Coverage::Lined { uncovered: counts }) = input.coverage {
                for (count, place) in [
                    (counts.before_first, "before its first value"),
                    (counts.in_gap, "in its gaps"),
                    (counts.after_last, "after its last value"),
                ] {
                    if count > 0 {
                        uncovered.push_str(&format!(", {count} {place}"));
                    }
                }
            }
            format!(
                "{} ({}{instance}, {}{provenance}{cap}{uncovered})",
                input.role,
                input.target.path(),
                input.writer
            )
        })
        .collect();
    let (read, unread): (Vec<&MetricReport>, Vec<&MetricReport>) = evaluation
        .metrics
        .iter()
        .partition(|metric| metric.samples.is_some());
    let metrics: Vec<String> = read
        .iter()
        .map(|metric| format!("{} in {}", metric.path, metric.unit))
        .collect();
    let offered: Vec<&str> = unread
        .iter()
        .filter_map(|metric| metric.path.rsplit(LEVEL_SEPARATOR).next())
        .collect();
    let params = if evaluation.params.is_empty() {
        String::new()
    } else {
        let params: Vec<String> = evaluation
            .params
            .iter()
            .map(|(key, value)| format!("{key} = {value}"))
            .collect();
        format!(" ({})", params.join(", "))
    };
    let mut line = format!(
        "[evaluate] {}: {}{params} of {} - {} samples",
        evaluation.name,
        evaluation.kind,
        inputs.join(", "),
        evaluation.samples,
    );
    if evaluation.uncovered > 0 {
        line.push_str(&format!(", {} uncovered", evaluation.uncovered));
    }
    if evaluation.refused > 0 {
        line.push_str(&format!(", {} not computed", evaluation.refused));
    }
    line.push_str(&format!(" -> {}", metrics.join(", ")));
    if !offered.is_empty() {
        line.push_str(&format!("; also offers {}", offered.join(", ")));
    }
    line
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        assertion::AssertionTarget,
        evaluation::{derived::UncoveredCounts, kind::Cap, GapWidth},
        report::{InputReport, ReportedUnit},
    };

    use helios_core::kernel::si_unit::SiUnit;

    #[test]
    fn an_evaluation_shows_its_inputs_coverage_and_the_metrics_read_then_offered() {
        let twist = EvaluationReportEntry {
            name: "twist".to_string(),
            kind: "twist_tracking".to_string(),
            samples: 100,
            uncovered: 3,
            refused: 0,
            params: toml::Table::new(),
            inputs: vec![
                InputReport {
                    role: "truth".to_string(),
                    target: AssertionTarget::parse("agent.car.channel.truth.twist")
                        .expect("valid target"),
                    instance: Some("agent.car.base_link".to_string()),
                    writer: "body unit truth".to_string(),
                    provenance: Some("exact"),
                    cap: Some(Cap::in_seconds(GapWidth::try_from(0.5).expect("positive"))),
                    refused: 0,
                    coverage: Some(Coverage::Base { gaps: 2 }),
                },
                InputReport {
                    role: "reference".to_string(),
                    target: AssertionTarget::parse("agent.car.channel.reference")
                        .expect("valid target"),
                    instance: None,
                    writer: "node reference_arbiter".to_string(),
                    provenance: None,
                    cap: Some(Cap::in_periods(
                        GapWidth::try_from(3.0).expect("positive"),
                        400.0,
                    )),
                    refused: 0,
                    coverage: Some(Coverage::Lined {
                        uncovered: UncoveredCounts {
                            before_first: 1,
                            in_gap: 0,
                            after_last: 2,
                        },
                    }),
                },
            ],
            metrics: vec![
                MetricReport {
                    path: "eval.inline.twist.surge".to_string(),
                    unit: ReportedUnit::Declared(SiUnit::MetersPerSecond),
                    samples: Some(99),
                },
                MetricReport {
                    path: "eval.inline.twist.sway".to_string(),
                    unit: ReportedUnit::Declared(SiUnit::MetersPerSecond),
                    samples: None,
                },
            ],
        };

        assert_eq!(
            evaluation_line(&twist),
            "[evaluate] twist: twist_tracking of truth (agent.car.channel.truth.twist\
             [agent.car.base_link], body unit truth, exact, 2 gaps past 0.5 s), \
             reference (agent.car.channel.reference, node reference_arbiter, gaps up to \
             0.0075 s = 3 periods, 1 before its first value, 2 after its last value) - 100 \
             samples, 3 uncovered -> eval.inline.twist.surge in m/s; also offers sway"
        );
    }

    #[test]
    fn an_evaluation_shows_its_params_and_what_it_didnt_compute() {
        let pose = EvaluationReportEntry {
            name: "pose".to_string(),
            kind: "pose_error".to_string(),
            samples: 10,
            uncovered: 0,
            refused: 2,
            params: "align = \"first_pose\"".parse().expect("valid TOML"),
            inputs: vec![],
            metrics: vec![],
        };

        assert_eq!(
            evaluation_line(&pose),
            "[evaluate] pose: pose_error (align = \"first_pose\") of  - 10 samples, 2 \
             not computed -> "
        );
    }
}
