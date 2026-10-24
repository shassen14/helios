//! _Post-step._ The Monte Carlo batch's report: the N runs' [`Report`]s
//! reduced to one. Each measure's value across the runs, by the cross-run
//! statistics, and each assertion's verdicts, counted. A run's numbers reach
//! a batch only as its measures and assertions, so whatever a run file
//! declares, a batch aggregates. A run that produced no report is counted,
//! never dropped: each row says how many runs it lacks, and why.
//!
//! The aggregator is [`AggregateReport`]'s constructor, so the report and the
//! function that builds it never drift apart.

pub mod console;
pub mod toml_writer;

use super::{AssertionReportEntry, AssertionStatus, MeasureReportEntry, Report, ReportedUnit};
use crate::{
    assertion::AssertionTarget,
    statistics::{across_runs::CrossRun, within_run::Reducer},
};

use serde::{ser::SerializeMap, Serialize, Serializer};

/// The statistics every measure row reports, in column order, until batch
/// assertions choose their own. Each reads numbers.
const MEASURE_STATISTICS: [CrossRun; 4] =
    [CrossRun::Mean, CrossRun::Std, CrossRun::Min, CrossRun::Max];

/// _Post-step._ The serialized summary of an N-run batch: how many runs ran,
/// the seed of each that had one, the runs that produced no report, and a row
/// per measure and per assertion, in run-file order.
///
/// Holds only report-owned types, as the single-run [`Report`] does, so this
/// schema stays stable while the per-run types churn.
#[derive(Debug, Clone, Serialize)]
pub struct AggregateReport {
    /// Every run the batch ran, whether or not it produced a report.
    run_count: usize,
    /// Only the seeds known: an unseeded batch reports none, never
    /// placeholder zeros. `--seed` seeds every run or none, so this is every
    /// run's seed in run order, or empty.
    seeds: Vec<u64>,
    /// The index of each run that produced no report, in run order: it never
    /// finalized, so nothing it recorded reached the batch.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    no_report: Vec<usize>,
    /// The statistics each measure row reports, in order.
    statistics: Vec<CrossRun>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    measures: Vec<MeasureAggregate>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    assertions: Vec<AssertionAggregate>,
}

impl AggregateReport {
    /// The batch summary of `runs`, in run order. A row is one entry, by
    /// name, agent and instance, gathered from every report that has it:
    /// under `instance = "*"` an instance may be absent from some runs.
    pub fn from_runs(runs: &[BatchRun]) -> Self {
        let reports: Vec<&Report> = runs.iter().filter_map(|run| run.report.as_ref()).collect();
        let no_report: Vec<usize> = runs
            .iter()
            .enumerate()
            .filter_map(|(index, run)| run.report.is_none().then_some(index))
            .collect();
        let missing = |present: usize| Missing {
            absent: reports.len() - present,
            no_report: no_report.len(),
        };
        let measures = rows(&reports, Report::measures, |entry| {
            (&entry.name, entry.agent(), entry.instance())
        })
        .iter()
        .filter_map(|entries| MeasureAggregate::of(entries, missing(entries.len())))
        .collect();
        let assertions = rows(&reports, Report::assertions, |entry| {
            (&entry.name, entry.agent(), entry.instance())
        })
        .iter()
        .filter_map(|entries| AssertionAggregate::of(entries, missing(entries.len())))
        .collect();
        Self {
            run_count: runs.len(),
            seeds: runs.iter().filter_map(|run| run.seed).collect(),
            no_report,
            statistics: MEASURE_STATISTICS.to_vec(),
            measures,
            assertions,
        }
    }
}

/// _Post-step._ What one run of a batch gave the aggregate: the seed it ran
/// with, if seeded, and its report, `None` when it never finalized.
#[derive(Debug, Clone)]
pub struct BatchRun {
    pub seed: Option<u64>,
    pub report: Option<Report>,
}

/// _Post-step._ One measure across the runs that gave it a number: how many
/// did, the samples they reduced, the runs it had no number in and why, and
/// the statistics of its numbers. Its counts add up to the batch's runs. A
/// row's own facts (target, reducer, unit) are its first run's, since every
/// run reads one run file.
#[derive(Debug, Clone, Serialize)]
pub struct MeasureAggregate {
    name: String,
    target: AssertionTarget,
    #[serde(skip_serializing_if = "Option::is_none")]
    agent: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    instance: Option<String>,
    reduce: Reducer,
    #[serde(skip_serializing_if = "Option::is_none")]
    unit: Option<ReportedUnit>,
    /// Runs that gave it a number, which the statistics reduce.
    runs: usize,
    /// The samples those runs reduced, summed: with each run's number, what
    /// pooling a per-sample statistic across runs (NEES) needs.
    samples: usize,
    /// Of `runs`, those whose window the run never reached the end of: each
    /// number is where its run stopped.
    #[serde(skip_serializing_if = "is_zero")]
    not_covered: usize,
    /// Runs that had it and gave no value: no samples, or a window never
    /// placed. Each run's report says why.
    #[serde(skip_serializing_if = "is_zero")]
    no_value: usize,
    /// Runs that refused it at setup, which gave it no value either.
    #[serde(skip_serializing_if = "is_zero")]
    unusable: usize,
    /// Runs whose value was a bool or a string, which no statistic reduces.
    #[serde(skip_serializing_if = "is_zero")]
    not_numbers: usize,
    /// Runs whose report hasn't it: an instance of `"*"` none of its series
    /// named.
    #[serde(skip_serializing_if = "is_zero")]
    absent: usize,
    /// Runs that produced no report.
    #[serde(skip_serializing_if = "is_zero")]
    no_report: usize,
    statistics: Statistics,
}

impl MeasureAggregate {
    /// The row of `entries`, one measure's entry from each run that has it,
    /// in run order, and the runs that haven't; `None` for no entries.
    fn of(entries: &[&MeasureReportEntry], missing: Missing) -> Option<Self> {
        let first = entries.first()?;
        let mut numbers = Vec::new();
        let (mut samples, mut not_covered, mut no_value, mut not_numbers) = (0, 0, 0, 0);
        let mut unusable = 0;
        for entry in entries {
            if entry.is_unusable() {
                unusable += 1;
                continue;
            }
            match entry.value().map(|value| value.as_number()) {
                Some(Some(number)) => {
                    numbers.push(number);
                    samples += entry.series().samples;
                    not_covered += usize::from(!entry.covered());
                }
                Some(None) => not_numbers += 1,
                None => no_value += 1,
            }
        }
        // Each of the fixed set reads numbers, so none is refused here.
        let statistics = MEASURE_STATISTICS
            .iter()
            .filter_map(|statistic| {
                let value = statistic.of_numbers(&numbers).ok().flatten()?;
                Some((*statistic, value))
            })
            .collect();
        Some(Self {
            name: first.name.clone(),
            target: first.target.clone(),
            agent: first.agent.clone(),
            instance: first.instance.clone(),
            reduce: first.reduce.clone(),
            unit: entries.iter().find_map(|entry| entry.unit()),
            runs: numbers.len(),
            samples,
            not_covered,
            no_value,
            unusable,
            not_numbers,
            absent: missing.absent,
            no_report: missing.no_report,
            statistics: Statistics(statistics),
        })
    }
}

/// _Post-step._ One assertion's verdicts across the batch's runs. A run that
/// didn't judge it, by an unreached window, an absent instance or no report,
/// is not a pass, as an unjudged assertion isn't in a single run.
#[derive(Debug, Clone, Serialize)]
pub struct AssertionAggregate {
    name: String,
    target: AssertionTarget,
    #[serde(skip_serializing_if = "Option::is_none")]
    agent: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    instance: Option<String>,
    /// The batch's runs: `passed + failed + unjudged + unusable + absent +
    /// no_report`.
    runs: usize,
    passed: usize,
    failed: usize,
    /// Runs that never reached the end of its window.
    #[serde(skip_serializing_if = "is_zero")]
    unjudged: usize,
    /// Runs that refused it at setup: the test, not the robot, is broken.
    #[serde(skip_serializing_if = "is_zero")]
    unusable: usize,
    /// Runs whose report hasn't it: an instance of `"*"` none of its series
    /// named.
    #[serde(skip_serializing_if = "is_zero")]
    absent: usize,
    /// Runs that produced no report.
    #[serde(skip_serializing_if = "is_zero")]
    no_report: usize,
    /// `passed` over `runs`, from 0 to 1.
    #[serde(skip_serializing_if = "Option::is_none")]
    pass_rate: Option<f64>,
}

impl AssertionAggregate {
    /// The row of `entries`, one assertion's entry from each run that has
    /// it, in run order, and the runs that haven't; `None` for no entries.
    fn of(entries: &[&AssertionReportEntry], missing: Missing) -> Option<Self> {
        let first = entries.first()?;
        let count = |status: AssertionStatus| {
            entries
                .iter()
                .filter(|entry| entry.status() == status)
                .count()
        };
        // Each run's verdict, a run without one never a pass.
        let verdicts: Vec<bool> = entries
            .iter()
            .map(|entry| entry.status() == AssertionStatus::Passed)
            .chain(std::iter::repeat_n(
                false,
                missing.absent + missing.no_report,
            ))
            .collect();
        Some(Self {
            name: first.name.clone(),
            target: first.target.clone(),
            agent: first.agent.clone(),
            instance: first.instance.clone(),
            runs: verdicts.len(),
            passed: count(AssertionStatus::Passed),
            failed: count(AssertionStatus::Failed),
            unjudged: count(AssertionStatus::Unjudged),
            unusable: count(AssertionStatus::Unusable),
            absent: missing.absent,
            no_report: missing.no_report,
            // `pass_rate` reads verdicts, so it is never refused here.
            pass_rate: CrossRun::PassRate.of_verdicts(&verdicts).ok().flatten(),
        })
    }
}

/// The runs of a batch one row has no entry from: those whose report hasn't
/// it, and those that produced no report.
#[derive(Debug, Clone, Copy)]
struct Missing {
    absent: usize,
    no_report: usize,
}

/// _Post-step._ A row's statistics, written as a table keyed by each
/// statistic's name, in the order the report lists them. A statistic with no
/// value (too few runs gave a number) is absent, never a null.
#[derive(Debug, Clone, PartialEq)]
pub struct Statistics(Vec<(CrossRun, f64)>);

impl Statistics {
    /// The value of `statistic`, if it has one.
    fn get(&self, statistic: &CrossRun) -> Option<f64> {
        self.0
            .iter()
            .find_map(|(named, value)| (named == statistic).then_some(*value))
    }
}

impl Serialize for Statistics {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut map = serializer.serialize_map(Some(self.0.len()))?;
        for (statistic, value) in &self.0 {
            map.serialize_entry(&statistic.to_string(), value)?;
        }
        map.end()
    }
}

/// Each entry `of` gives, grouped by `key` across `reports` into one row
/// per key, rows in the order their key first appears, each row's entries in
/// run order.
fn rows<'a, E, K: PartialEq + 'a>(
    reports: &[&'a Report],
    of: fn(&Report) -> &[E],
    key: impl Fn(&'a E) -> K,
) -> Vec<Vec<&'a E>> {
    let mut keys: Vec<K> = Vec::new();
    let mut rows: Vec<Vec<&E>> = Vec::new();
    for entry in reports.iter().flat_map(|report| of(report)) {
        let entry_key = key(entry);
        match keys.iter().position(|seen| *seen == entry_key) {
            Some(row) => rows[row].push(entry),
            None => {
                keys.push(entry_key);
                rows.push(vec![entry]);
            }
        }
    }
    rows
}

fn is_zero(count: &usize) -> bool {
    *count == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::{AssertionValue, Condition, Window},
        report::{
            DroppedValues, FailureReason, RunEnd, RunEntries, RunIdentity, RunStop, SeriesSummary,
            Verdict,
        },
        run::termination::TerminationReason,
    };

    use helios_core::kernel::si_unit::SiUnit;

    /// The `final` of a pose's x, `value` over `samples` samples, `None`
    /// when the run gave it none, its window reached when `covered`.
    pub(super) fn measure(
        name: &str,
        value: Option<f64>,
        samples: usize,
        covered: bool,
    ) -> MeasureReportEntry {
        measure_of(name, value.map(AssertionValue::Float), samples, covered)
    }

    fn measure_of(
        name: &str,
        value: Option<AssertionValue>,
        samples: usize,
        covered: bool,
    ) -> MeasureReportEntry {
        let reason = (value.is_none()).then_some(FailureReason::NoSamples);
        MeasureReportEntry::new(
            name.to_owned(),
            AssertionTarget::parse("eval.inline.pose.x").expect("valid target"),
            None,
            None,
            Window::default(),
            Reducer::Final,
            covered,
            value.map(Ok).or(reason.map(Err)),
            Some(ReportedUnit::Declared(SiUnit::Meters)),
            series(samples),
        )
    }

    /// A GPS NIS assertion reaching `verdict`.
    pub(super) fn assertion(name: &str, verdict: Verdict) -> AssertionReportEntry {
        AssertionReportEntry::new(
            name.to_owned(),
            AssertionTarget::parse("agent.car.node.primary.aiding.gps.nis").expect("valid target"),
            None,
            None,
            Window::default(),
            Some(Reducer::Final),
            Some(ReportedUnit::Declared(SiUnit::Dimensionless)),
            Condition::LessThan(7.0),
            series(1),
            verdict,
        )
    }

    /// What one run declares: its measures and its assertions.
    pub(super) type Run = (Vec<MeasureReportEntry>, Vec<AssertionReportEntry>);

    /// The aggregate of one run per item, run `i` seeded `i`.
    pub(super) fn batch(runs: &[Run]) -> AggregateReport {
        let runs: Vec<Option<Run>> = runs.iter().cloned().map(Some).collect();
        batch_with_gaps(&runs)
    }

    /// The aggregate of one run per item, run `i` seeded `i`, `None` for a
    /// run that produced no report.
    pub(super) fn batch_with_gaps(runs: &[Option<Run>]) -> AggregateReport {
        let runs: Vec<BatchRun> = runs
            .iter()
            .enumerate()
            .map(|(index, run)| {
                let seed = Some(index as u64);
                BatchRun {
                    seed,
                    report: run.as_ref().map(|(measures, assertions)| {
                        report(seed, measures.clone(), assertions.clone())
                    }),
                }
            })
            .collect();
        AggregateReport::from_runs(&runs)
    }

    fn report(
        seed: Option<u64>,
        measures: Vec<MeasureReportEntry>,
        assertions: Vec<AssertionReportEntry>,
    ) -> Report {
        Report::new(
            RunIdentity {
                run_name: "batch".to_owned(),
                scenario: "sim/scenarios/01_proving_ground.toml".to_owned(),
                master_seed: seed,
            },
            RunEnd {
                simulated_duration_secs: 20.0,
                wall_duration_secs: 1.0,
                stop: RunStop::Ended(TerminationReason::MaxSimulatedSeconds),
            },
            DroppedValues::default(),
            RunEntries {
                assertions,
                measures,
                evaluations: Vec::new(),
            },
        )
    }

    fn series(samples: usize) -> SeriesSummary {
        SeriesSummary {
            samples,
            max_gap_secs: None,
            final_age_secs: None,
            window_secs: None,
        }
    }

    fn passed() -> Verdict {
        Verdict::Passed {
            actual: AssertionValue::Float(1.0),
        }
    }

    #[test]
    fn a_measure_is_reduced_across_runs_with_its_runs_and_samples() {
        let runs: Vec<_> = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0]
            .into_iter()
            .map(|value| (vec![measure("final_x", Some(value), 3, true)], vec![]))
            .collect();

        let report = batch(&runs);

        let row = &report.measures[0];
        assert_eq!((row.runs, row.samples), (8, 24));
        assert_eq!(row.statistics.get(&CrossRun::Mean), Some(5.0));
        assert_eq!(row.statistics.get(&CrossRun::Min), Some(2.0));
        assert_eq!(row.statistics.get(&CrossRun::Max), Some(9.0));
        let std = row.statistics.get(&CrossRun::Std).expect("numbers");
        assert!((std - (32.0_f64 / 7.0).sqrt()).abs() < 1e-12, "{std}");
    }

    #[test]
    fn runs_without_a_number_are_counted_apart_and_not_reduced() {
        let report = batch(&[
            (vec![measure("final_x", Some(1.0), 4, true)], vec![]),
            (vec![measure("final_x", Some(3.0), 2, false)], vec![]),
            (vec![measure("final_x", None, 0, true)], vec![]),
            (
                vec![measure_of(
                    "final_x",
                    Some(AssertionValue::String("parked".to_owned())),
                    1,
                    true,
                )],
                vec![],
            ),
        ]);

        let row = &report.measures[0];
        assert_eq!(row.runs, 2);
        assert_eq!(row.samples, 6, "only the runs with a number");
        assert_eq!((row.not_covered, row.no_value, row.not_numbers), (1, 1, 1));
        assert_eq!(row.statistics.get(&CrossRun::Mean), Some(2.0));
    }

    #[test]
    fn a_nan_in_one_run_reaches_the_statistics() {
        let report = batch(&[
            (vec![measure("nis", Some(1.0), 1, true)], vec![]),
            (vec![measure("nis", Some(f64::NAN), 1, true)], vec![]),
        ]);

        let row = &report.measures[0];
        assert!(row.statistics.get(&CrossRun::Mean).is_some_and(f64::is_nan));
        assert!(row.statistics.get(&CrossRun::Max).is_some_and(f64::is_nan));
    }

    #[test]
    fn a_measure_no_run_gave_a_number_has_no_statistics() {
        let report = batch(&[(vec![measure("final_x", None, 0, true)], vec![])]);

        assert_eq!(report.measures[0].statistics, Statistics(Vec::new()));
    }

    #[test]
    fn each_agents_entry_of_one_name_is_its_own_row() {
        let on = |agent: &str, value: f64| {
            let entry = measure("surge_final", Some(value), 1, true);
            MeasureReportEntry {
                agent: Some(agent.to_owned()),
                ..entry
            }
        };
        let report = batch(&[
            (vec![on("north", 1.0), on("south", 3.0)], vec![]),
            (vec![on("north", 2.0), on("south", 4.0)], vec![]),
        ]);

        let rows: Vec<(Option<&str>, usize)> = report
            .measures
            .iter()
            .map(|row| (row.agent.as_deref(), row.runs))
            .collect();
        assert_eq!(rows, [(Some("north"), 2), (Some("south"), 2)]);
    }

    #[test]
    fn rows_are_by_name_in_run_file_order() {
        let report = batch(&[
            (
                vec![
                    measure("final_y", Some(1.0), 1, true),
                    measure("final_x", Some(1.0), 1, true),
                ],
                vec![],
            ),
            (
                vec![
                    measure("final_y", Some(2.0), 1, true),
                    measure("final_x", Some(2.0), 1, true),
                ],
                vec![],
            ),
        ]);

        let names: Vec<&str> = report
            .measures
            .iter()
            .map(|row| row.name.as_str())
            .collect();
        assert_eq!(names, ["final_y", "final_x"]);
        assert!(report.measures.iter().all(|row| row.runs == 2));
    }

    #[test]
    fn an_assertion_counts_each_verdict_and_unjudged_is_not_a_pass() {
        let failed = Verdict::Failed {
            actual: None,
            reason: FailureReason::NoSamples,
            failed_at_secs: None,
        };
        let report = batch(&[
            (vec![], vec![assertion("nis_final", passed())]),
            (vec![], vec![assertion("nis_final", passed())]),
            (vec![], vec![assertion("nis_final", failed)]),
            (
                vec![],
                vec![assertion("nis_final", Verdict::Unjudged { actual: None })],
            ),
        ]);

        let row = &report.assertions[0];
        assert_eq!(row.runs, 4);
        assert_eq!((row.passed, row.failed, row.unjudged), (2, 1, 1));
        assert_eq!(row.pass_rate, Some(0.5));
    }

    #[test]
    fn an_unusable_entry_is_counted_apart_never_as_a_failure() {
        let unusable = || Verdict::Unusable {
            problems: vec!["its agent's pipeline wasn't built".to_owned()],
        };
        let refused = || measure("final_x", None, 0, false).refused(vec!["refused".to_owned()]);
        let report = batch(&[
            (vec![refused()], vec![assertion("nis_final", unusable())]),
            (vec![refused()], vec![assertion("nis_final", unusable())]),
        ]);

        let row = &report.assertions[0];
        assert_eq!((row.runs, row.failed, row.unusable), (2, 0, 2));
        let row = &report.measures[0];
        assert_eq!((row.runs, row.no_value, row.unusable), (0, 0, 2));
    }

    #[test]
    fn the_seeds_are_carried_in_run_order_and_an_unseeded_batch_has_none() {
        let seeded = batch(&[(vec![], vec![]), (vec![], vec![]), (vec![], vec![])]);
        let unseeded = AggregateReport::from_runs(&[
            BatchRun {
                seed: None,
                report: Some(report(None, Vec::new(), Vec::new())),
            },
            BatchRun {
                seed: None,
                report: None,
            },
        ]);

        assert_eq!(seeded.seeds, vec![0, 1, 2]);
        assert!(unseeded.seeds.is_empty());
        assert_eq!(unseeded.run_count, 2);
    }

    #[test]
    fn a_run_with_no_report_is_counted_and_is_not_a_pass() {
        let report = batch_with_gaps(&[
            Some((
                vec![measure("final_x", Some(1.0), 1, true)],
                vec![assertion("nis_final", passed())],
            )),
            None,
            Some((
                vec![measure("final_x", Some(3.0), 1, true)],
                vec![assertion("nis_final", passed())],
            )),
        ]);

        assert_eq!(report.run_count, 3);
        assert_eq!(
            report.seeds,
            vec![0, 1, 2],
            "the seed that gave no report too"
        );
        assert_eq!(report.no_report, vec![1]);
        let measure = &report.measures[0];
        assert_eq!((measure.runs, measure.no_report), (2, 1));
        let assertion = &report.assertions[0];
        assert_eq!((assertion.runs, assertion.passed), (3, 2));
        assert_eq!(assertion.no_report, 1);
        assert_eq!(assertion.pass_rate, Some(2.0 / 3.0));
    }

    #[test]
    fn an_entry_a_report_lacks_is_counted_absent_and_is_not_a_pass() {
        let report = batch(&[
            (
                vec![measure("final_x", Some(1.0), 1, true)],
                vec![assertion("nis_final", passed())],
            ),
            (vec![], vec![]),
        ]);

        let measure = &report.measures[0];
        assert_eq!((measure.runs, measure.absent), (1, 1));
        let assertion = &report.assertions[0];
        assert_eq!(
            (assertion.runs, assertion.passed, assertion.absent),
            (2, 1, 1)
        );
        assert_eq!(assertion.pass_rate, Some(0.5));
    }

    #[test]
    fn one_run_with_a_number_has_no_std() {
        let report = batch(&[(vec![measure("final_x", Some(1.0), 1, true)], vec![])]);

        let row = &report.measures[0];
        assert_eq!(row.statistics.get(&CrossRun::Mean), Some(1.0));
        assert_eq!(row.statistics.get(&CrossRun::Std), None);
    }
}
