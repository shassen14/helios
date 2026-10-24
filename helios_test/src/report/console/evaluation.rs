//! An evaluation's block: what it read, how well its inputs covered the
//! base, and which metrics it computed, one fact per line.

use super::super::{EvaluationReportEntry, MetricReport, ReportedUnit};
use crate::evaluation::derived::Coverage;

use helios_core::kernel::{identifier::LEVEL_SEPARATOR, si_unit::SiUnit};

/// How far a fact sits under its evaluation's head line, and an entry of a
/// table under its key.
const INDENT: &str = "    ";

/// "[evaluate] <name>: <kind> over <samples> samples[, <n> uncovered][, <n>
/// not computed]", then one indented line per parameter and per fact
/// binding fixed (a table's entries each on its own line beneath its key),
/// one per role ("<role>: [<series> at ]<target>, <writer>[, <provenance>]
/// [, gaps up to <seconds> s[ = <multiple> periods]][, <n> before its first
/// value][, <n> in its gaps][, <n> after its last value]", the base's cap
/// reading "<n> gaps past <seconds> s"), each metric read with its unit,
/// and the metrics offered but unread. Each count but the base's gaps
/// shows only when nonzero.
pub(super) fn evaluation_lines(evaluation: &EvaluationReportEntry) -> String {
    let mut head = format!(
        "[evaluate] {}: {} over {} samples",
        evaluation.name, evaluation.kind, evaluation.samples,
    );
    if evaluation.uncovered > 0 {
        head.push_str(&format!(", {} uncovered", evaluation.uncovered));
    }
    if evaluation.refused > 0 {
        head.push_str(&format!(", {} not computed", evaluation.refused));
    }
    let mut lines = vec![head];
    for (key, value) in evaluation.params.iter().chain(&evaluation.binding) {
        lines.extend(field_lines(key, value));
    }
    for (role, input) in evaluation.inputs.iter().enumerate() {
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
        let series = input
            .series
            .as_ref()
            .map(|series| format!("{series} at "))
            .unwrap_or_default();
        lines.push(format!(
            "{INDENT}{}: {series}{}{instance}, {}{provenance}{cap}{uncovered}",
            input.role,
            input.target.path(),
            input.writer
        ));
    }
    let (read, unread): (Vec<&MetricReport>, Vec<&MetricReport>) = evaluation
        .metrics
        .iter()
        .partition(|metric| metric.samples.is_some());
    if !read.is_empty() {
        lines.push(format!("{INDENT}metrics:"));
        lines.extend(
            read.iter()
                .map(|metric| format!("{INDENT}{INDENT}{}", metric_phrase(metric))),
        );
    }
    let offered: Vec<&str> = unread
        .iter()
        .filter_map(|metric| metric.path.rsplit(LEVEL_SEPARATOR).next())
        .collect();
    if !offered.is_empty() {
        lines.push(format!("{INDENT}also offers: {}", offered.join(", ")));
    }
    lines.join("\n")
}

/// A parameter's or a binding fact's lines: "<key>: <value>", or a table's
/// key alone with each entry on its own line beneath it, `none` for an
/// empty one. Strings print unquoted.
fn field_lines(key: &str, value: &toml::Value) -> Vec<String> {
    match value {
        toml::Value::Table(table) if table.is_empty() => vec![format!("{INDENT}{key}: none")],
        toml::Value::Table(table) => std::iter::once(format!("{INDENT}{key}:"))
            .chain(
                table
                    .iter()
                    .map(|(key, value)| format!("{INDENT}{INDENT}{key}: {}", unquoted(value))),
            )
            .collect(),
        value => vec![format!("{INDENT}{key}: {}", unquoted(value))],
    }
}

/// A string as its text, any other value as TOML writes it.
fn unquoted(value: &toml::Value) -> String {
    match value {
        toml::Value::String(text) => text.clone(),
        value => value.to_string(),
    }
}

/// A metric's path and the unit its samples are in; a dimensionless one's
/// path alone.
fn metric_phrase(metric: &MetricReport) -> String {
    match metric.unit {
        ReportedUnit::Declared(SiUnit::Dimensionless) => metric.path.clone(),
        unit => format!("{} in {unit}", metric.path),
    }
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
            application: None,
            agent: None,
            kind: "twist_tracking".to_string(),
            samples: 100,
            uncovered: 3,
            refused: 0,
            params: toml::Table::new(),
            binding: toml::Table::new(),
            inputs: vec![
                InputReport {
                    role: "truth".to_string(),
                    series: None,
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
                    series: None,
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
            evaluation_lines(&twist),
            "[evaluate] twist: twist_tracking over 100 samples, 3 uncovered\n    \
             truth: agent.car.channel.truth.twist[agent.car.base_link], body unit truth, \
             exact, 2 gaps past 0.5 s\n    \
             reference: agent.car.channel.reference, node reference_arbiter, gaps up to \
             0.0075 s = 3 periods, 1 before its first value, 2 after its last value\n    \
             metrics:\n        \
             eval.inline.twist.surge in m/s\n    \
             also offers: sway"
        );
    }

    #[test]
    fn an_evaluation_lists_its_params_and_each_table_binding_fixed_entry_by_entry() {
        let pose = EvaluationReportEntry {
            name: "pose".to_string(),
            application: None,
            agent: None,
            kind: "pose_error".to_string(),
            samples: 10,
            uncovered: 0,
            refused: 2,
            params: "align = \"declared\"".parse().expect("valid TOML"),
            binding: toml::Table::from_iter([
                (
                    "roots".to_owned(),
                    "agent.car.odom anchored at global.datum".into(),
                ),
                (
                    "graded".to_owned(),
                    toml::Value::Table(toml::Table::from_iter([(
                        "position in agent.car.odom".to_owned(),
                        3.into(),
                    )])),
                ),
                (
                    "ungraded".to_owned(),
                    toml::Value::Table(toml::Table::new()),
                ),
            ]),
            inputs: vec![],
            metrics: vec![MetricReport {
                path: "eval.inline.pose.nees".to_string(),
                unit: ReportedUnit::Declared(SiUnit::Dimensionless),
                samples: Some(8),
            }],
        };

        assert_eq!(
            evaluation_lines(&pose),
            "[evaluate] pose: pose_error over 10 samples, 2 not computed\n    \
             align: declared\n    \
             graded:\n        \
             position in agent.car.odom: 3\n    \
             roots: agent.car.odom anchored at global.datum\n    \
             ungraded: none\n    \
             metrics:\n        \
             eval.inline.pose.nees"
        );
    }
}
