//! _Post-step._ The report's machine surface: serialize a finished [`Report`] to
//! TOML and write it to disk, so CI can parse the verdict alongside the process
//! exit code. Deliberately thin — serialize and write, nothing else. Human-facing
//! formatting lives in `console.rs`; keeping the two apart is what stops
//! readability concerns from leaking into the machine schema.

use super::Report;

use std::fs;
use std::path::Path;
use thiserror::Error;

/// _Post-step._ Why a report couldn't be written. The two variants stay distinct
/// so a failure is diagnosable: `Io` is a disk/path problem; `Serialize` is a
/// report whose shape the TOML serializer rejected (e.g. a scalar field ordered
/// after a sub-table). Each carries its source via `#[from]` so `?` converts
/// cleanly and the message keeps the underlying detail.
///
/// Not `Clone`: neither `std::io::Error` nor `toml::ser::Error` is `Clone`, and a
/// write error is consumed once at the call site.
#[derive(Debug, Error)]
pub enum ReportWriteError {
    #[error("failed to output toml: {0}")]
    Io(#[from] std::io::Error),
    #[error("failed to serialize toml: {0}")]
    Serialize(#[from] toml::ser::Error),
}

/// Serialize `report` to pretty TOML and write it to `path`, overwriting any
/// existing file. The `?` on each step converts the underlying error into the
/// matching [`ReportWriteError`] variant via `#[from]`. The serializer puts
/// each table's scalars ahead of its sub-tables itself, so `Report`'s field
/// order is free.
pub fn write(report: &Report, path: &Path) -> Result<(), ReportWriteError> {
    let text = toml::to_string_pretty(report)?;

    fs::write(path, text)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::{
        AssertionTarget, AssertionValue, Bound, Condition, InstanceSelector, RunInstant, Window,
    };
    use crate::evaluation::{
        derived::{Coverage, UncoveredCounts},
        kind::Cap,
        GapWidth,
    };
    use crate::report::{
        AbortCause, AssertionReportEntry, BoundEvaluation, DerivedEvaluation, EvaluationOutcome,
        EvaluationReport, FailureReason, FaultStep, HostFault, InputReport, MeasureReportEntry,
        MetricReport, ReportedUnit, SeriesSummary, TerminatedBy, Verdict,
    };
    use crate::statistics::within_run::Reducer;

    use helios_core::kernel::si_unit::SiUnit;

    use std::sync::atomic::{AtomicU64, Ordering};

    fn series(samples: usize) -> SeriesSummary {
        SeriesSummary {
            samples,
            max_gap_secs: Some(1.5),
            final_age_secs: None,
            window_secs: None,
        }
    }

    /// As [`series`], over a window placed at 7.5 s to 12.5 s.
    fn windowed_series(samples: usize) -> SeriesSummary {
        SeriesSummary {
            window_secs: Some([7.5, 12.5]),
            ..series(samples)
        }
    }

    /// The last 5 s of the run.
    fn window() -> Window {
        Window {
            start: Some(Bound::At {
                at: RunInstant::End,
                plus: -5.0,
            }),
            end: None,
        }
    }

    /// A failing `time_above` assertion on the speed channel over the last
    /// 5 s.
    fn time_above_failing(verdict: Verdict) -> AssertionReportEntry {
        AssertionReportEntry::new(
            "rarely_fast".to_string(),
            AssertionTarget::parse("agent.car.channel.speed").expect("valid target"),
            None,
            None,
            window(),
            Some(Reducer::TimeAbove(5.0)),
            Some(ReportedUnit::Declared(SiUnit::Seconds)),
            Condition::LessThan(2.0),
            windowed_series(40),
            verdict,
        )
    }

    // A representative finished report: a failing run that stopped on the time
    // budget, with a passing `within_range` line, a failing `time_above` line
    // and one measure. Exercises every serialized shape the writer touches —
    // scalars, the `terminated_by` table, the condition and reducer tables
    // inside the `assertions` array-of-tables, and the `measures` one.
    fn sample_report() -> Report {
        Report::new(
            "smoke".to_string(),
            "sim.scenarios.parking_lot".to_string(),
            Some(42),
            Vec::new(),
            Vec::new(),
            12.5,
            0.3,
            TerminatedBy::TimeBudget,
            vec![
                AssertionReportEntry::new(
                    "parked".to_string(),
                    AssertionTarget::parse("agent.car.node.estimator.position")
                        .expect("valid target"),
                    None,
                    None,
                    Window::default(),
                    Some(Reducer::Final),
                    Some(ReportedUnit::Declared(SiUnit::Meters)),
                    Condition::WithinRange([-0.5, 0.5]),
                    SeriesSummary {
                        final_age_secs: Some(0.25),
                        ..series(3)
                    },
                    Verdict::Passed {
                        actual: AssertionValue::Float(0.0),
                    },
                ),
                time_above_failing(Verdict::Failed {
                    actual: Some(AssertionValue::Float(7.0)),
                    reason: FailureReason::ConditionFailed,
                    failed_at_secs: None,
                }),
            ],
            vec![MeasureReportEntry::new(
                "goal_distance".to_string(),
                AssertionTarget::parse("agent.car.node.eval.goal_distance").expect("valid target"),
                None,
                None,
                Window::default(),
                Reducer::Final,
                true,
                Some(Ok(AssertionValue::Float(0.5))),
                Some(ReportedUnit::Declared(SiUnit::Meters)),
                series(1),
            )],
            vec![twist_evaluation()],
        )
    }

    /// A `twist_tracking` evaluation of the car against its exact truth.
    fn twist_evaluation() -> EvaluationReport {
        EvaluationReport {
            name: "twist".to_string(),
            application: None,
            agent: None,
            kind: "twist_tracking".to_string(),
            outcome: EvaluationOutcome::Derived(DerivedEvaluation {
                samples: 100,
                uncovered: 1,
                refused: 0,
                bound: twist_bound(),
            }),
        }
    }

    /// What the twist evaluation was bound to, and each input's coverage.
    fn twist_bound() -> BoundEvaluation {
        BoundEvaluation {
            params: toml::Table::new(),
            binding: toml::Table::new(),
            inputs: vec![
                InputReport {
                    role: "truth".to_string(),
                    series: None,
                    target: AssertionTarget::parse("agent.car.channel.truth.twist")
                        .expect("valid target")
                        .with_instance(InstanceSelector::One("agent.car.base_link".into())),
                    instance: Some("agent.car.base_link".to_string()),
                    writer: "body unit truth".to_string(),
                    provenance: Some("exact"),
                    cap: Some(Cap::in_seconds(GapWidth::try_from(0.5).expect("positive"))),
                    refused: 0,
                    coverage: Some(Coverage::Base { gaps: 0 }),
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
                    refused: 2,
                    coverage: Some(Coverage::Lined {
                        uncovered: UncoveredCounts {
                            before_first: 0,
                            in_gap: 1,
                            after_last: 0,
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
        }
    }

    /// What the first evaluation of `report`, which derived, was bound to.
    fn bound_mut(report: &mut Report) -> &mut BoundEvaluation {
        match &mut report.evaluations[0].outcome {
            EvaluationOutcome::Derived(derived) => &mut derived.bound,
            outcome => panic!("the first evaluation didn't derive: {outcome:?}"),
        }
    }

    /// `report` written and read back as a plain TOML document.
    fn written(report: &Report) -> toml::Value {
        let path = write_to_temp(report);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        toml::from_str(&text).unwrap()
    }

    // Write `report` to a unique temp file, returning the path. The process id
    // and the atomic counter keep parallel tests, as threads or as processes,
    // from colliding on a filename — same std-only pattern as `run::mod`'s
    // tests, to avoid a temp-file dev-dependency.
    fn write_to_temp(report: &Report) -> std::path::PathBuf {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "helios_report_test_{}_{n}.toml",
            std::process::id()
        ));
        write(report, &path).unwrap();
        path
    }

    #[test]
    fn round_trips_through_toml() {
        let value = written(&sample_report());

        // Scalars, the derived status and the nested `terminated_by` table.
        assert_eq!(value["schema_version"].as_str(), Some("3.0"));
        assert_eq!(value["run_name"].as_str(), Some("smoke"));
        assert_eq!(value["status"].as_str(), Some("failed"));
        assert_eq!(value["master_seed"].as_integer(), Some(42));
        assert_eq!(value["terminated_by"]["kind"].as_str(), Some("time_budget"));

        let assertions = value["assertions"].as_array().unwrap();
        assert_eq!(assertions.len(), 2);

        // The within_range line: its condition as written, its reducer as a
        // string, what its series held, and no failure fields at all.
        let parked = &assertions[0];
        assert_eq!(parked["name"].as_str(), Some("parked"));
        assert_eq!(parked["reduce"].as_str(), Some("final"));
        let range = parked["condition"]["within_range"].as_array().unwrap();
        assert_eq!(range[0].as_float(), Some(-0.5));
        assert_eq!(range[1].as_float(), Some(0.5));
        assert_eq!(parked["samples"].as_integer(), Some(3));
        assert_eq!(parked["max_gap_secs"].as_float(), Some(1.5));
        assert_eq!(parked["final_age_secs"].as_float(), Some(0.25));
        assert_eq!(parked["status"].as_str(), Some("passed"));
        assert_eq!(parked["actual"].as_float(), Some(0.0));
        assert!(parked.get("reason").is_none());
        // The unit its value and bound are read in.
        assert_eq!(parked["unit"].as_str(), Some("m"));

        // The time_above line: its reducer is a table with its threshold.
        let fast = &assertions[1];
        assert_eq!(fast["target"].as_str(), Some("agent.car.channel.speed"));
        assert_eq!(fast["reduce"]["time_above"].as_float(), Some(5.0));
        assert_eq!(fast["unit"].as_str(), Some("s"));
        assert_eq!(fast["condition"]["less_than"].as_float(), Some(2.0));
        assert_eq!(fast["status"].as_str(), Some("failed"));
        assert_eq!(fast["actual"].as_float(), Some(7.0));
        assert_eq!(fast["reason"].as_str(), Some("condition_failed"));
        assert!(fast.get("final_age_secs").is_none());
        // Its window as written, and where it fell.
        assert_eq!(fast["window"]["start"]["at"].as_str(), Some("run.end"));
        assert_eq!(fast["window"]["start"]["plus"].as_float(), Some(-5.0));
        let placed = fast["window_secs"].as_array().unwrap();
        assert_eq!(placed[0].as_float(), Some(7.5));
        assert_eq!(placed[1].as_float(), Some(12.5));
        // A whole-run entry carries neither.
        assert!(parked.get("window").is_none());
        assert!(parked.get("window_secs").is_none());

        let measures = value["measures"].as_array().unwrap();
        assert_eq!(measures.len(), 1);
        assert_eq!(measures[0]["name"].as_str(), Some("goal_distance"));
        assert_eq!(measures[0]["value"].as_float(), Some(0.5));
        assert_eq!(measures[0]["unit"].as_str(), Some("m"));
        assert_eq!(measures[0]["covered"].as_bool(), Some(true));
        assert!(measures[0].get("reason").is_none());

        // An evaluation: what it read and from whom, how much it covered,
        // and each metric's series with its unit.
        let evaluations = value["evaluations"].as_array().unwrap();
        let twist = &evaluations[0];
        assert_eq!(twist["kind"].as_str(), Some("twist_tracking"));
        assert_eq!(twist["status"].as_str(), Some("derived"));
        assert!(twist.get("params").is_none());
        assert_eq!(twist["samples"].as_integer(), Some(100));
        assert_eq!(twist["uncovered"].as_integer(), Some(1));
        let inputs = twist["inputs"].as_array().unwrap();
        // The base counts its own gaps; the other where it left the base
        // uncovered.
        assert_eq!(inputs[0]["gaps"].as_integer(), Some(0));
        assert!(inputs[0].get("uncovered").is_none());
        assert_eq!(inputs[1]["uncovered"]["before_first"].as_integer(), Some(0));
        assert_eq!(inputs[1]["uncovered"]["in_gap"].as_integer(), Some(1));
        assert_eq!(inputs[1]["uncovered"]["after_last"].as_integer(), Some(0));
        assert_eq!(inputs[0]["role"].as_str(), Some("truth"));
        assert_eq!(inputs[0]["instance"].as_str(), Some("agent.car.base_link"));
        assert_eq!(inputs[0]["provenance"].as_str(), Some("exact"));
        assert!(inputs[0].get("refused").is_none());
        assert!(inputs[1].get("provenance").is_none());
        assert_eq!(inputs[0]["cap"]["seconds"].as_float(), Some(0.5));
        assert!(inputs[0]["cap"].get("multiple").is_none());
        assert_eq!(inputs[1]["cap"]["seconds"].as_float(), Some(0.0075));
        assert_eq!(inputs[1]["cap"]["multiple"].as_float(), Some(3.0));
        assert_eq!(inputs[1]["refused"].as_integer(), Some(2));
        let metrics = twist["metrics"].as_array().unwrap();
        assert_eq!(metrics[0]["path"].as_str(), Some("eval.inline.twist.surge"));
        assert_eq!(metrics[0]["unit"].as_str(), Some("m/s"));
        assert_eq!(metrics[0]["samples"].as_integer(), Some(99));
        // Offered and not read, so listed without samples.
        assert_eq!(metrics[1]["path"].as_str(), Some("eval.inline.twist.sway"));
        assert!(metrics[1].get("samples").is_none());
    }

    #[test]
    fn an_aborted_run_writes_each_cause_and_each_refusal() {
        let refused = || vec!["its agent's pipeline wasn't built".to_owned()];
        let report = Report::new(
            "unbuilt".to_string(),
            "test/scenarios/unbuilt_pipeline.toml".to_string(),
            None,
            Vec::new(),
            Vec::new(),
            0.0,
            0.1,
            TerminatedBy::Aborted {
                causes: vec![
                    AbortCause::PipelineNotBuilt {
                        agent: "truck".to_string(),
                        errors: vec!["nothing supplies the map".to_string()],
                    },
                    AbortCause::HostFault(HostFault::ForeignCollector {
                        at: FaultStep::Tick { at_secs: 1.5 },
                    }),
                ],
            },
            vec![time_above_failing(Verdict::Unusable {
                problems: refused(),
            })],
            vec![MeasureReportEntry::new(
                "goal_distance".to_string(),
                AssertionTarget::parse("agent.truck.node.eval.goal_distance")
                    .expect("valid target"),
                None,
                None,
                Window::default(),
                Reducer::Final,
                false,
                None,
                None,
                series(0),
            )
            .refused(refused())],
            vec![EvaluationReport {
                name: "agent.truck.tracking.twist".to_string(),
                application: Some("tracking".to_string()),
                agent: Some("truck".to_string()),
                kind: "twist_tracking".to_string(),
                outcome: EvaluationOutcome::Unusable {
                    problems: refused(),
                },
            }],
        );

        let value = written(&report);

        assert_eq!(value["status"].as_str(), Some("aborted"));
        let terminated_by = &value["terminated_by"];
        assert_eq!(terminated_by["kind"].as_str(), Some("aborted"));
        let causes = terminated_by["causes"].as_array().unwrap();
        let kinds: Vec<_> = causes.iter().map(|cause| cause["kind"].as_str()).collect();
        // The refused entries' cause is derived, after the unbuilt pipeline.
        assert_eq!(
            kinds,
            [
                Some("pipeline_not_built"),
                Some("unusable_entries"),
                Some("host_fault")
            ]
        );
        assert_eq!(causes[0]["agent"].as_str(), Some("truck"));
        assert_eq!(causes[2]["fault"].as_str(), Some("foreign_collector"));
        assert_eq!(causes[2]["at"]["step"].as_str(), Some("tick"));
        assert_eq!(causes[2]["at"]["at_secs"].as_float(), Some(1.5));
        for table in ["assertions", "measures", "evaluations"] {
            let entry = &value[table].as_array().unwrap()[0];
            assert_eq!(entry["status"].as_str(), Some("unusable"), "{table}");
            assert_eq!(
                entry["problems"].as_array().unwrap()[0].as_str(),
                Some("its agent's pipeline wasn't built"),
                "{table}"
            );
        }
        let evaluation = &value["evaluations"].as_array().unwrap()[0];
        assert_eq!(evaluation["kind"].as_str(), Some("twist_tracking"));
        assert_eq!(evaluation["status"].as_str(), Some("unusable"));
        assert!(evaluation.get("samples").is_none());
    }

    #[test]
    fn an_underived_evaluation_is_written_with_its_binding_and_no_count() {
        let mut report = sample_report();
        let mut bound = twist_bound();
        for input in &mut bound.inputs {
            input.refused = 0;
            input.coverage = None;
        }
        for metric in &mut bound.metrics {
            metric.samples = None;
        }
        report.evaluations[0].outcome = EvaluationOutcome::Underived(bound);

        let value = written(&report);

        let evaluation = &value["evaluations"].as_array().unwrap()[0];
        assert_eq!(evaluation["status"].as_str(), Some("underived"));
        for count in ["samples", "uncovered", "refused"] {
            assert!(evaluation.get(count).is_none(), "{count}");
        }
        let inputs = evaluation["inputs"].as_array().unwrap();
        assert_eq!(inputs[0]["role"].as_str(), Some("truth"));
        assert!(inputs[0].get("gaps").is_none());
        assert!(inputs[1].get("uncovered").is_none());
        let metrics = evaluation["metrics"].as_array().unwrap();
        assert_eq!(metrics[0]["path"].as_str(), Some("eval.inline.twist.surge"));
        assert!(metrics[0].get("samples").is_none());
    }

    #[test]
    fn what_binding_fixed_is_written_in_its_own_table_beside_its_params() {
        let mut report = sample_report();
        let roots = "agent.car.odom anchored at global.datum";
        bound_mut(&mut report).params = "align = \"declared\"".parse().expect("valid TOML");
        bound_mut(&mut report).binding =
            toml::Table::from_iter([("roots".to_owned(), roots.into())]);

        let value = written(&report);

        let evaluation = &value["evaluations"].as_array().unwrap()[0];
        assert_eq!(evaluation["binding"]["roots"].as_str(), Some(roots));
        assert!(evaluation.get("roots").is_none());
        assert_eq!(evaluation["params"]["align"].as_str(), Some("declared"));
    }

    #[test]
    fn a_suites_entry_names_its_application_agent_and_each_inputs_series() {
        let inline = written(&sample_report());
        let inline = &inline["evaluations"].as_array().unwrap()[0];
        assert!(inline.get("application").is_none());
        assert!(inline.get("agent").is_none());
        assert!(inline["inputs"].as_array().unwrap()[0]
            .get("series")
            .is_none());

        let mut report = sample_report();
        report.evaluations[0].application = Some("tracking".to_owned());
        report.evaluations[0].agent = Some("car".to_owned());
        bound_mut(&mut report).inputs[0].series = Some("truth.twist".to_owned());
        let value = written(&report);

        let evaluation = &value["evaluations"].as_array().unwrap()[0];
        assert_eq!(evaluation["application"].as_str(), Some("tracking"));
        assert_eq!(evaluation["agent"].as_str(), Some("car"));
        assert_eq!(
            evaluation["inputs"].as_array().unwrap()[0]["series"].as_str(),
            Some("truth.twist")
        );
    }

    #[test]
    fn a_report_with_no_evaluations_omits_them() {
        let mut report = sample_report();
        report.evaluations.clear();
        assert!(written(&report).get("evaluations").is_none());
    }

    #[test]
    fn an_entry_on_one_instance_carries_its_selector_and_instance() {
        let mut report = sample_report();
        let each = AssertionTarget::parse("agent.car.channel.truth.tf")
            .expect("valid target")
            .with_instance(InstanceSelector::Each);
        report.assertions[0] = AssertionReportEntry::new(
            "level".to_string(),
            each,
            None,
            Some("agent.car.base_link".to_string()),
            Window::default(),
            None,
            Some(ReportedUnit::Undeclared),
            Condition::LessThan(1.0),
            series(3),
            Verdict::Passed {
                actual: AssertionValue::Float(0.0),
            },
        );
        report.terminated_by = TerminatedBy::AssertionFailed {
            name: "level".to_string(),
            path: "agent.car.channel.truth.tf".to_string(),
            instance: Some("agent.car.base_link".to_string()),
        };

        let value = written(&report);

        let level = &value["assertions"].as_array().unwrap()[0];
        assert_eq!(
            level["target"]["path"].as_str(),
            Some("agent.car.channel.truth.tf")
        );
        assert_eq!(level["target"]["instance"].as_str(), Some("*"));
        assert_eq!(level["instance"].as_str(), Some("agent.car.base_link"));
        assert_eq!(level["unit"].as_str(), Some("undeclared"));
        let stop = &value["terminated_by"];
        assert_eq!(stop["kind"].as_str(), Some("assertion_failed"));
        assert_eq!(stop["path"].as_str(), Some("agent.car.channel.truth.tf"));
        assert_eq!(stop["instance"].as_str(), Some("agent.car.base_link"));
        // An unkeyed entry has no instance.
        assert!(value["assertions"].as_array().unwrap()[1]
            .get("instance")
            .is_none());
    }

    #[test]
    fn an_entry_on_every_agents_evaluation_carries_its_selector_and_agent() {
        let mut report = sample_report();
        let each =
            AssertionTarget::parse("eval.agent.*.tracking.twist.surge").expect("valid target");
        report.assertions[0] = AssertionReportEntry::new(
            "surge_settled".to_string(),
            each,
            Some("north".to_string()),
            None,
            Window::default(),
            None,
            Some(ReportedUnit::Declared(SiUnit::MetersPerSecond)),
            Condition::LessThan(1.0),
            series(3),
            Verdict::Passed {
                actual: AssertionValue::Float(0.0),
            },
        );

        let value = written(&report);

        let assertions = value["assertions"].as_array().unwrap();
        assert_eq!(
            assertions[0]["target"].as_str(),
            Some("eval.agent.*.tracking.twist.surge")
        );
        assert_eq!(assertions[0]["agent"].as_str(), Some("north"));
        assert!(assertions[0].get("instance").is_none());
        // A target naming its agent carries none.
        assert!(assertions[1].get("agent").is_none());
    }

    #[test]
    fn a_failure_with_nothing_observed_omits_the_absent_fields() {
        let mut report = sample_report();
        report.assertions[1] = time_above_failing(Verdict::Failed {
            actual: None,
            reason: FailureReason::NoSamples,
            failed_at_secs: None,
        });

        let value = written(&report);

        let failed = &value["assertions"].as_array().unwrap()[1];
        assert_eq!(failed["reason"].as_str(), Some("no_samples"));
        assert!(failed.get("actual").is_none());
        assert!(failed.get("failed_at_secs").is_none());
    }

    #[test]
    fn a_report_with_no_measures_omits_them() {
        let mut report = sample_report();
        report.measures.clear();
        assert!(written(&report).get("measures").is_none());
    }

    #[test]
    fn refused_writes_are_listed_only_when_there_are_some() {
        assert!(written(&sample_report()).get("refused_writes").is_none());

        let mut refused = sample_report();
        refused.refused_writes = vec!["agent 'car': truth.tf refused 3 write(s)".to_string()];
        let value = written(&refused);
        let lines = value["refused_writes"].as_array().unwrap();
        assert_eq!(lines.len(), 1);
        assert_eq!(
            lines[0].as_str(),
            Some("agent 'car': truth.tf refused 3 write(s)")
        );
    }

    #[test]
    fn recording_faults_are_listed_only_when_there_are_some() {
        assert!(written(&sample_report()).get("recording").is_none());

        let mut dropped = sample_report();
        dropped.recording = vec!["agent.car.node.estimator.x: 1 value(s)".to_string()];
        let value = written(&dropped);
        let lines = value["recording"].as_array().unwrap();
        assert_eq!(lines.len(), 1);
        assert_eq!(
            lines[0].as_str(),
            Some("agent.car.node.estimator.x: 1 value(s)")
        );
    }

    #[test]
    fn missing_parent_dir_is_io_error() {
        let report = sample_report();
        let result = write(&report, Path::new("/no/such/dir/report.toml"));
        assert!(matches!(result, Err(ReportWriteError::Io(_))));
    }
}
