//! Golden runs: every run file under `configs/test/runs/` is run through
//! `helios_test_sim`, and what it produced is compared with the committed
//! golden file under `configs/test/golden/<platform>/`, at the same relative
//! path.
//!
//! Each platform (`macos-aarch64`, `linux-x86_64`, `windows-x86_64`) keeps its
//! own golden files: each platform's maths library rounds transcendentals its
//! own way, and the sim carries a last-bit difference into every later
//! number. On one platform, two runs give the same bytes.
//!
//! A run the sim starts leaves a report, compared whole except its wall time
//! (`<name>.report.toml`); its exit code must be the one its golden status
//! gives. A run refused before the sim starts leaves only its refusal on
//! stderr (`<name>.stderr`), and must exit 2. Any difference fails, so a
//! change that moves a number updates its golden file in the same commit.
//!
//! Run files under `negative/` are negative controls: each must fail, and one
//! that passes fails this check whatever its golden file says.
//!
//! Ignored by default: it needs the `sim` feature and runs for minutes.
//!
//! ```text
//! cargo test -p helios_test --features sim --test golden_runs -- --ignored
//! ```
//!
//! With `HELIOS_BLESS=1` set, it rewrites this platform's golden files from
//! this run instead of comparing, and a negative control that passes is
//! still refused.
//!
//! Every run also writes what it produced, in golden form, under
//! `target/tmp/golden_runs/golden/<platform>/`. For a platform you can't run
//! locally, CI uploads that folder; replacing `configs/test/golden/<platform>/`
//! with it blesses that platform.
//!
//! Beside it, one run file is run again with loading held for extra updates,
//! and again one fixed step per update: neither how long loading lasts nor
//! how many steps a frame runs may change a report.

#![cfg(feature = "sim")]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Run files, under the workspace root.
const RUNS: &str = "configs/test/runs";
/// Golden files, under the workspace root, one folder per platform, each
/// mirroring `RUNS`.
const GOLDEN: &str = "configs/test/golden";
/// This run's output, under the target's temp folder: each report as written,
/// and each result in golden form, one folder per platform as under `GOLDEN`.
const OUTPUT: &str = "golden_runs";
const REPORTS: &str = "reports";
const PRODUCED: &str = "golden";
/// What `helios_test_sim` resolves a run's scenario against.
const CONFIG_ROOT: &str = "configs";
/// The folder of negative controls, under `RUNS`.
const NEGATIVE: &str = "negative";
/// Set to rewrite the golden files instead of comparing.
const BLESS: &str = "HELIOS_BLESS";

/// The run file the time guards run twice each: a minute of closed loop
/// through the GPS EKF, where a last-bit difference in any stamp reaches the
/// reported numbers.
const TIME_GUARD_RUN: &str = "pose_error_proving_ground.toml";
/// Under `OUTPUT`, the loading guard's reports.
const LOADING: &str = "loading";
/// `helios_test_sim`'s hidden flag holding loading for extra updates, and the
/// number the guard holds it for. Before the run clock, 40 moved every
/// pose-error number of the guard's run.
const HOLD_LOADING: &str = "--hold-loading";
const HELD_UPDATES: &str = "40";
/// Under `OUTPUT`, the batch guard's reports.
const BATCH: &str = "batch";
/// `helios_test_sim`'s hidden flag fixing the steps each update runs, and
/// the batch the guard runs at, against the derived batch (20 at 400 Hz).
/// Before truth and sensors read one post-step world, the batch moved truth
/// by a step on all but its first step.
const STEPS_PER_UPDATE: &str = "--steps-per-update";
const SINGLE_STEP: &str = "1";

/// The one report line that differs between identical runs.
const WALL_TIME_KEY: &str = "wall_duration_secs = ";
/// The report line of a run that passed.
const PASSED_STATUS: &str = "status = \"passed\"";

const REPORT_SUFFIX: &str = "report.toml";
const STDERR_SUFFIX: &str = "stderr";

/// `helios_test_sim`'s exit codes: passed, failed, refused before the sim.
const EXIT_PASSED: i32 = 0;
const EXIT_FAILED: i32 = 1;
const EXIT_REFUSED: i32 = 2;

/// What one run produced, in the form its golden file keeps.
struct Outcome {
    exit: i32,
    golden: Golden,
}

/// A golden file: which kind it is, and what it holds.
struct Golden {
    kind: GoldenKind,
    text: String,
}

impl Golden {
    /// The golden file of the run at `run` under `RUNS`, as committed.
    fn committed(run: &Path) -> Option<Golden> {
        GoldenKind::ALL.into_iter().find_map(|kind| {
            let text = fs::read_to_string(kind.path_for(run)).ok()?;
            Some(Golden { kind, text })
        })
    }

    /// The exit code a run producing this golden file must have.
    fn exit(&self) -> i32 {
        match self.kind {
            GoldenKind::Report if self.text.lines().any(|line| line == PASSED_STATUS) => {
                EXIT_PASSED
            }
            GoldenKind::Report => EXIT_FAILED,
            GoldenKind::Refusal => EXIT_REFUSED,
        }
    }
}

/// The two kinds of golden file, by how far the run got.
#[derive(Clone, Copy)]
enum GoldenKind {
    /// The sim ran: its report, wall time removed.
    Report,
    /// The run was refused before the sim started: its stderr.
    Refusal,
}

impl GoldenKind {
    const ALL: [GoldenKind; 2] = [GoldenKind::Report, GoldenKind::Refusal];

    fn suffix(self) -> &'static str {
        match self {
            GoldenKind::Report => REPORT_SUFFIX,
            GoldenKind::Refusal => STDERR_SUFFIX,
        }
    }

    /// This kind's golden file, under this platform's folder in `GOLDEN`, of
    /// the run at `run` under `RUNS`.
    fn path_for(self, run: &Path) -> PathBuf {
        golden_root().join(self.relative_to_root(run))
    }

    /// This kind's golden file of the run at `run`, relative to a platform's
    /// folder.
    fn relative_to_root(self, run: &Path) -> PathBuf {
        run.with_extension(self.suffix())
    }
}

#[test]
#[ignore = "runs every committed run file through the sim; needs --features sim"]
fn every_run_matches_its_golden_file() {
    let bless = std::env::var_os(BLESS).is_some();
    let runs = run_files();
    assert!(!runs.is_empty(), "no run files under {RUNS}");

    let missing = !bless && !golden_root().exists();
    let produced = output().join(PRODUCED).join(platform());
    // Golden form left by an earlier run must not stand in for this one's.
    let _ = fs::remove_dir_all(&produced);

    let mut failures = Vec::new();
    for run in &runs {
        let outcome = execute(run);
        let negative = run.starts_with(NEGATIVE);
        if negative && outcome.exit == EXIT_PASSED {
            failures.push(format!(
                "{}: a negative control passed; it must fail",
                run.display()
            ));
            continue;
        }
        write_file(
            &produced.join(outcome.golden.kind.relative_to_root(run)),
            &outcome.golden.text,
        );
        if missing {
            continue;
        }
        if bless {
            write_golden(run, &outcome.golden);
        } else if let Some(failure) = compare(run, &outcome) {
            failures.push(failure);
        }
    }
    if missing {
        panic!(
            "no golden files for {platform} at {root}. This run's results, in \
             golden form, are in {produced}: check them and copy that folder \
             to {root}, or rerun with {BLESS}=1.",
            platform = platform(),
            root = golden_root().display(),
            produced = produced.display(),
        );
    }
    failures.extend(orphaned_golden_files(&runs, bless));

    assert!(
        failures.is_empty(),
        "{} difference(s) between the {} run file(s) and their golden files:\n\n{}\n\n\
         If a change is intended, rerun with {BLESS}=1 and commit the golden files; \
         for a platform you can't run locally, replace {root} with {produced}, \
         which CI uploads.",
        failures.len(),
        runs.len(),
        failures.join("\n\n"),
        root = golden_root().display(),
        produced = produced.display(),
    );
}

#[test]
#[ignore = "runs one run file through the sim twice; needs --features sim"]
fn loading_time_changes_no_report() {
    let run = Path::new(TIME_GUARD_RUN);
    let reports = output().join(LOADING);
    let prompt = execute_into(run, &reports.join("prompt"), &[]);
    let held = execute_into(run, &reports.join("held"), &[HOLD_LOADING, HELD_UPDATES]);

    assert_eq!(prompt.exit, held.exit, "{}: exit differs", run.display());
    let difference =
        first_labelled_difference(("prompt", &prompt.golden.text), ("held", &held.golden.text));
    if let Some(difference) = difference {
        panic!(
            "{}: holding loading for {HELD_UPDATES} updates changed the report, \
             so a stamp still counts loading time: {difference}",
            run.display()
        );
    }
}

#[test]
#[ignore = "runs one run file through the sim twice; needs --features sim"]
fn steps_per_update_change_no_report() {
    let run = Path::new(TIME_GUARD_RUN);
    let reports = output().join(BATCH);
    let batched = execute_into(run, &reports.join("batched"), &[]);
    let single = execute_into(
        run,
        &reports.join("single"),
        &[STEPS_PER_UPDATE, SINGLE_STEP],
    );

    assert_eq!(batched.exit, single.exit, "{}: exit differs", run.display());
    let difference = first_labelled_difference(
        ("batched", &batched.golden.text),
        ("single", &single.golden.text),
    );
    if let Some(difference) = difference {
        panic!(
            "{}: running {SINGLE_STEP} step per update changed the report, \
             so a value depends on where a frame ends: {difference}",
            run.display()
        );
    }
}

/// The workspace root, which run paths and the config root are relative to.
fn workspace() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate sits in the workspace")
        .to_path_buf()
}

/// The platform the golden files of this run are kept under: the OS and the
/// CPU architecture, which together choose the maths library and the
/// instructions a float goes through.
fn platform() -> String {
    format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH)
}

/// This platform's folder of golden files.
fn golden_root() -> PathBuf {
    workspace().join(GOLDEN).join(platform())
}

/// Where this run's output goes, under the target's temp folder.
fn output() -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(OUTPUT)
}

/// Every run file, relative to `RUNS`, sorted so failures list in one order.
fn run_files() -> Vec<PathBuf> {
    let root = workspace().join(RUNS);
    let mut runs = files_under(&root, &root, "toml");
    runs.sort();
    runs
}

/// Files under `dir` ending in `.{extension}`, relative to `root`.
fn files_under(root: &Path, dir: &Path, extension: &str) -> Vec<PathBuf> {
    let entries = fs::read_dir(dir).unwrap_or_else(|e| panic!("read {}: {e}", dir.display()));
    let mut files = Vec::new();
    for entry in entries {
        let path = entry.expect("a directory entry").path();
        if path.is_dir() {
            files.extend(files_under(root, &path, extension));
        } else if path.to_string_lossy().ends_with(&format!(".{extension}")) {
            let relative = path.strip_prefix(root).expect("found under its root");
            files.push(relative.to_path_buf());
        }
    }
    files
}

/// Runs `run` through `helios_test_sim` from the workspace root, as by hand.
fn execute(run: &Path) -> Outcome {
    execute_into(run, &output().join(REPORTS), &[])
}

/// Runs `run` as [`execute`] does, passing `extra` arguments and writing its
/// report under `reports`.
fn execute_into(run: &Path, reports: &Path, extra: &[&str]) -> Outcome {
    let report = reports.join(run.with_extension(REPORT_SUFFIX));
    fs::create_dir_all(report.parent().expect("a report has a folder"))
        .expect("create the report folder");
    // A report left by an earlier run must not stand in for this one's.
    let _ = fs::remove_file(&report);

    let output = Command::new(env!("CARGO_BIN_EXE_helios_test_sim"))
        .current_dir(workspace())
        .arg("--run")
        .arg(Path::new(RUNS).join(run))
        .arg("--config-root")
        .arg(CONFIG_ROOT)
        .arg("--out")
        .arg(&report)
        .args(extra)
        .output()
        .unwrap_or_else(|e| panic!("start helios_test_sim: {e}"));

    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    let Some(exit) = output.status.code() else {
        panic!("{}: helios_test_sim was killed:\n{stderr}", run.display());
    };
    let golden = if exit == EXIT_REFUSED {
        Golden {
            kind: GoldenKind::Refusal,
            text: stderr,
        }
    } else {
        let text = fs::read_to_string(&report).unwrap_or_else(|e| {
            panic!(
                "{}: exit {exit} and no report ({e}):\n{stderr}",
                run.display()
            )
        });
        Golden {
            kind: GoldenKind::Report,
            text: without_wall_time(&text),
        }
    };
    Outcome { exit, golden }
}

/// The report with its wall-time line removed.
fn without_wall_time(report: &str) -> String {
    report
        .lines()
        .filter(|line| !line.starts_with(WALL_TIME_KEY))
        .map(|line| format!("{line}\n"))
        .collect()
}

/// What differs between `outcome` and the committed golden file, if anything.
fn compare(run: &Path, outcome: &Outcome) -> Option<String> {
    let name = run.display();
    let Some(golden) = Golden::committed(run) else {
        return Some(format!(
            "{name}: no golden file at {}",
            outcome.golden.kind.path_for(run).display()
        ));
    };
    if outcome.exit != golden.exit() {
        return Some(format!(
            "{name}: exit {}, its golden file gives {}",
            outcome.exit,
            golden.exit()
        ));
    }
    first_difference(&golden.text, &outcome.golden.text)
        .map(|difference| format!("{name}: {difference}"))
}

/// The first line where `actual` departs from `golden`, if it does.
fn first_difference(golden: &str, actual: &str) -> Option<String> {
    first_labelled_difference(("golden", golden), ("actual", actual))
}

/// The first line where two texts depart, each shown under its label.
fn first_labelled_difference(
    (left_label, left): (&str, &str),
    (right_label, right): (&str, &str),
) -> Option<String> {
    let mut left_lines = left.lines();
    let mut right_lines = right.lines();
    let mut number = 1;
    loop {
        match (left_lines.next(), right_lines.next()) {
            (None, None) => return None,
            (left_line, right_line) if left_line != right_line => {
                return Some(format!(
                    "line {number} differs\n  {left_label}: {}\n  {right_label}: {}",
                    left_line.unwrap_or("<end of file>"),
                    right_line.unwrap_or("<end of file>")
                ));
            }
            _ => number += 1,
        }
    }
}

/// Writes `golden` for `run`, removing the golden file of the other kind.
fn write_golden(run: &Path, golden: &Golden) {
    write_file(&golden.kind.path_for(run), &golden.text);
    for kind in GoldenKind::ALL {
        if kind.suffix() != golden.kind.suffix() {
            let _ = fs::remove_file(kind.path_for(run));
        }
    }
}

/// Writes `text` to `path`, creating its folder.
fn write_file(path: &Path, text: &str) {
    fs::create_dir_all(path.parent().expect("a written file has a folder"))
        .expect("create the folder");
    fs::write(path, text).unwrap_or_else(|e| panic!("write {}: {e}", path.display()));
}

/// This platform's golden files whose run file is gone: removed when
/// blessing, else failures.
fn orphaned_golden_files(runs: &[PathBuf], bless: bool) -> Vec<String> {
    let root = golden_root();
    if !root.exists() {
        return Vec::new();
    }
    let mut failures = Vec::new();
    for suffix in GoldenKind::ALL.map(GoldenKind::suffix) {
        for golden in files_under(&root, &root, suffix) {
            let stem = golden.to_string_lossy();
            let run =
                PathBuf::from(stem.trim_end_matches(&format!(".{suffix}"))).with_extension("toml");
            if runs.contains(&run) {
                continue;
            }
            if bless {
                let _ = fs::remove_file(root.join(&golden));
            } else {
                failures.push(format!(
                    "{}: a golden file with no run file at {}",
                    golden.display(),
                    run.display()
                ));
            }
        }
    }
    failures
}
