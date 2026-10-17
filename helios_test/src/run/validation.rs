use crate::{
    assertion::{condition::OperandProblem, config::WindowProblem, EntryName},
    run::Run,
    statistics::within_run::Reducer,
};

use std::collections::HashSet;
use thiserror::Error;

/// _Pre-step._ Coherence problems a structurally-valid run file can still have. Parsing
/// proves the *shape* is right; these are the cross-field checks serde can't
/// express. Caught at load time so a nonsensical run fails fast instead of
/// being discovered partway through a simulation.
#[derive(Debug, PartialEq, Error)]
pub enum ValidationError {
    #[error(
        "`max_simulated_seconds` must be positive and finite, got {0}: the run would never \
         terminate"
    )]
    UnreachableBudget(f64),
    #[error(
        "`fail_fast` is set but the run declares no assertion judging every sample (one \
         without `reduce`) on a window known from the first tick (bounds in seconds after \
         run.start) that could fail early"
    )]
    FailFastWithoutEverySample,
    /// Names identify verdicts in the report and across runs, so two
    /// entries can't share one, whether assertions or measures.
    #[error("two entries are named '{0}'; names are unique across [[assert]] and [[measure]]")]
    DuplicateName(EntryName),
    #[error("assertion '{name}': {problem}")]
    UnusableOperand {
        name: EntryName,
        problem: OperandProblem,
    },
    #[error("'{0}': the time_above threshold is not a finite number")]
    NonFiniteThreshold(EntryName),
    #[error("'{name}': {problem}")]
    UnusableWindow {
        name: EntryName,
        problem: WindowProblem,
    },
}

/// Check a parsed [`Run`] for coherence. A flat checklist of guard clauses:
/// each looks for one specific incoherence and returns early with its matching
/// variant, so the report can name *which* thing was wrong. Falling through to
/// the end means every check passed.
///
/// Deliberately separate from parsing (`load`) — "can I read this?" and "is
/// this a sane run?" are different questions with different failures.
pub fn validate(run: &Run) -> Result<(), ValidationError> {
    let termination = run.termination();

    // The budget is the one stop every run is sure of. Elapsed time never
    // reaches a NaN or infinite budget, and a zero or negative one stops
    // before anything is judged.
    let budget = termination.max_simulated_seconds();
    if !(budget.is_finite() && budget > 0.0) {
        return Err(ValidationError::UnreachableBudget(budget));
    }

    // `fail_fast` stops on an every-sample failure judged as it arrives. A
    // reduced assertion is only judged at the end, and so is one whose
    // window isn't known until the run stops or an anchor occurs, so
    // without an every-sample one on a fixed window it watches nothing.
    let has_every_sample = run
        .assertions()
        .iter()
        .any(|assertion| assertion.judges_every_sample() && assertion.window.fixed());
    if termination.fail_fast() && !has_every_sample {
        return Err(ValidationError::FailFastWithoutEverySample);
    }

    let names = run
        .assertions()
        .iter()
        .map(|assertion| &assertion.name)
        .chain(run.measures().iter().map(|measure| &measure.name));
    let mut seen = HashSet::new();
    for name in names {
        if !seen.insert(name) {
            return Err(ValidationError::DuplicateName(name.clone()));
        }
    }

    for assertion in run.assertions() {
        if let Err(problem) = assertion.condition.check_operand() {
            return Err(ValidationError::UnusableOperand {
                name: assertion.name.clone(),
                problem,
            });
        }
    }

    let reducers = run
        .assertions()
        .iter()
        .filter_map(|assertion| Some((&assertion.name, assertion.reduce.as_ref()?)))
        .chain(
            run.measures()
                .iter()
                .map(|measure| (&measure.name, &measure.reduce)),
        );
    for (name, reducer) in reducers {
        if let Reducer::TimeAbove(threshold) = reducer {
            if !threshold.is_finite() {
                return Err(ValidationError::NonFiniteThreshold(name.clone()));
            }
        }
    }

    let windows = run
        .assertions()
        .iter()
        .map(|assertion| (&assertion.name, &assertion.window))
        .chain(
            run.measures()
                .iter()
                .map(|measure| (&measure.name, &measure.window)),
        );
    for (name, window) in windows {
        if let Err(problem) = window.check(budget) {
            return Err(ValidationError::UnusableWindow {
                name: name.clone(),
                problem,
            });
        }
    }

    validate_observation_covers_target(run)?;

    Ok(())
}

// Seam for a future check: every assertion target should be covered by the
// resolved observation preset. Stubbed to `Ok` because observation resolution
// doesn't exist yet; wiring the call site now means filling this in later
// touches only the body, never `validate`.
fn validate_observation_covers_target(_run: &Run) -> Result<(), ValidationError> {
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Build a `Run` by parsing TOML, so tests exercise the real type. Callers
    // supply the run-specific body (termination + any `[[assert]]` or
    // `[[measure]]` blocks); the header fields every run needs are filled in
    // here.
    fn run_with(body: &str) -> Run {
        let toml = format!(
            r#"
            schema_version = "2.0"
            scenario = {{ from = "s" }}
            observation = {{ from = "o" }}
            {body}
            "#
        );
        toml::from_str(&toml).unwrap()
    }

    const BUDGET: &str = "termination = { max_simulated_seconds = 5.0 }";

    const REDUCED: &str = r#"
        [[assert]]
        name = "reduced"
        target = "agent.car.node.estimator.x"
        reduce = "final"
        condition = { less_than = 1.0 }
    "#;

    const EVERY_SAMPLE: &str = r#"
        [[assert]]
        name = "every_sample"
        target = "agent.car.node.estimator.x"
        condition = { less_than = 1.0 }
    "#;

    /// One assertion named `name` on `condition`, under a plain budget.
    fn asserting(name: &str, reduce: &str, condition: &str) -> Run {
        run_with(&format!(
            "{BUDGET}\n[[assert]]\nname = \"{name}\"\n\
             target = \"agent.car.node.estimator.x\"\n{reduce}\ncondition = {condition}"
        ))
    }

    #[test]
    fn time_budget_alone_is_ok_without_assertions() {
        // A pure smoke run: the time budget guarantees termination, so zero
        // assertions is fine.
        assert_eq!(validate(&run_with(BUDGET)), Ok(()));
    }

    #[test]
    fn a_budget_the_clock_never_reaches_errors() {
        for budget in ["0.0", "-1.0", "nan", "inf"] {
            let run = run_with(&format!(
                "termination = {{ max_simulated_seconds = {budget} }}"
            ));
            assert!(
                matches!(validate(&run), Err(ValidationError::UnreachableBudget(_))),
                "budget {budget}"
            );
        }
    }

    #[test]
    fn fail_fast_without_assertions_errors() {
        let run = run_with("termination = { max_simulated_seconds = 5.0, fail_fast = true }");
        assert_eq!(
            validate(&run),
            Err(ValidationError::FailFastWithoutEverySample)
        );
    }

    #[test]
    fn fail_fast_over_only_reduced_assertions_errors() {
        // A reduced assertion can't fail before the end, so fail_fast would
        // never fire.
        let body =
            format!("termination = {{ max_simulated_seconds = 5.0, fail_fast = true }}{REDUCED}");
        assert_eq!(
            validate(&run_with(&body)),
            Err(ValidationError::FailFastWithoutEverySample)
        );
    }

    #[test]
    fn fully_specified_run_passes() {
        let body = format!(
            "termination = {{ max_simulated_seconds = 30.0, fail_fast = true }}{REDUCED}{EVERY_SAMPLE}"
        );
        assert_eq!(validate(&run_with(&body)), Ok(()));
    }

    #[test]
    fn a_name_used_twice_is_refused_across_assertions_and_measures() {
        let measure = r#"
            [[measure]]
            name = "reduced"
            target = "agent.car.node.estimator.x"
            reduce = "mean"
        "#;
        assert_eq!(
            validate(&run_with(&format!("{BUDGET}{REDUCED}{measure}"))),
            Err(ValidationError::DuplicateName(
                EntryName::parse("reduced").unwrap()
            ))
        );
        assert!(matches!(
            validate(&run_with(&format!("{BUDGET}{REDUCED}{REDUCED}"))),
            Err(ValidationError::DuplicateName(_))
        ));
    }

    #[test]
    fn unusable_operands_are_refused_naming_the_assertion() {
        for condition in [
            "{ less_than = nan }",
            "{ greater_than = inf }",
            "{ within_range = [0.7, 0.3] }",
            "{ within_range = [0.0, nan] }",
            "{ equals = 0.0 }",
            "{ not_equals = 1.5 }",
        ] {
            assert!(
                matches!(
                    validate(&asserting("bounded", "", condition)),
                    Err(ValidationError::UnusableOperand { ref name, .. }) if name.as_str() == "bounded"
                ),
                "{condition}"
            );
        }
    }

    #[test]
    fn usable_operands_pass() {
        for condition in [
            "{ less_than = 25 }",
            "{ equals = 0 }",
            "{ not_equals = \"idle\" }",
            "{ within_range = [0.3, 0.7] }",
        ] {
            assert_eq!(
                validate(&asserting("bounded", "", condition)),
                Ok(()),
                "{condition}"
            );
        }
    }

    #[test]
    fn a_non_finite_time_above_threshold_is_refused_on_an_assertion_or_a_measure() {
        assert!(matches!(
            validate(&asserting(
                "above",
                "reduce = { time_above = nan }",
                "{ less_than = 1.0 }"
            )),
            Err(ValidationError::NonFiniteThreshold(_))
        ));
        let measure = r#"
            [[measure]]
            name = "above"
            target = "agent.car.node.estimator.x"
            reduce = { time_above = inf }
        "#;
        assert!(matches!(
            validate(&run_with(&format!("{BUDGET}{measure}"))),
            Err(ValidationError::NonFiniteThreshold(_))
        ));
    }

    #[test]
    fn fail_fast_over_only_windows_not_fixed_errors() {
        // Judged at finalize, so it can't stop the run.
        let body = r#"
            termination = { max_simulated_seconds = 5.0, fail_fast = true }
            [[assert]]
            name = "late"
            target = "agent.car.node.estimator.x"
            window = { start = { at = "run.end", plus = -1.0 } }
            condition = { less_than = 1.0 }
        "#;
        assert_eq!(
            validate(&run_with(body)),
            Err(ValidationError::FailFastWithoutEverySample)
        );
    }

    #[test]
    fn fail_fast_over_a_fixed_window_passes() {
        let body = r#"
            termination = { max_simulated_seconds = 5.0, fail_fast = true }
            [[assert]]
            name = "settled"
            target = "agent.car.node.estimator.x"
            window = { start = 1.0, end = 4.0 }
            condition = { less_than = 1.0 }
        "#;
        assert_eq!(validate(&run_with(body)), Ok(()));
    }

    #[test]
    fn an_unusable_window_is_refused_naming_its_entry() {
        // Past the 5 s budget, on a measure.
        let measure = r#"
            [[measure]]
            name = "late"
            target = "agent.car.node.estimator.x"
            window = { end = 7.0 }
            reduce = "mean"
        "#;
        assert_eq!(
            validate(&run_with(&format!("{BUDGET}{measure}"))),
            Err(ValidationError::UnusableWindow {
                name: EntryName::parse("late").unwrap(),
                problem: WindowProblem::PastBudget {
                    end: 7.0,
                    budget: 5.0,
                },
            })
        );
    }
}
