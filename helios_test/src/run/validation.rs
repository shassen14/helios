use thiserror::Error;

use crate::{assertion::Assertion, run::Run};

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
        "`fail_fast` is set but the run declares no continuous assertion that could fail early"
    )]
    FailFastWithoutContinuous,
    // todo!() WithinRangeWithoutBounds
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

    // `fail_fast` stops on a continuous failure; a terminal assertion is only
    // judged at the end, so without a continuous one it watches nothing.
    let has_continuous = run
        .assertions()
        .iter()
        .any(|assertion| matches!(assertion, Assertion::Continuous(_)));
    if termination.fail_fast() && !has_continuous {
        return Err(ValidationError::FailFastWithoutContinuous);
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
    // supply the run-specific body (termination + any `[[assert]]` blocks); the
    // header fields every run needs are filled in here.
    fn run_with(body: &str) -> Run {
        let toml = format!(
            r#"
            schema_version = "1.0"
            scenario = {{ from = "s" }}
            observation = {{ from = "o" }}
            {body}
            "#
        );
        toml::from_str(&toml).unwrap()
    }

    const TERMINAL: &str = r#"
        [[assert]]
        when = "at_completion"
        target = "agent.car.estimator.x"
        condition = "equals"
        value = 0.0
    "#;

    const CONTINUOUS: &str = r#"
        [[assert]]
        when = "continuous"
        target = "agent.car.estimator.x"
        condition = "less_than"
        value = 1.0
    "#;

    #[test]
    fn time_budget_alone_is_ok_without_assertions() {
        // A pure smoke run: the time budget guarantees termination, so zero
        // assertions is fine.
        let run = run_with("termination = { max_simulated_seconds = 5.0 }");
        assert_eq!(validate(&run), Ok(()));
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
            Err(ValidationError::FailFastWithoutContinuous)
        );
    }

    #[test]
    fn fail_fast_over_only_terminal_assertions_errors() {
        // A terminal assertion can't fail before the end, so fail_fast would
        // never fire.
        let body =
            format!("termination = {{ max_simulated_seconds = 5.0, fail_fast = true }}{TERMINAL}");
        assert_eq!(
            validate(&run_with(&body)),
            Err(ValidationError::FailFastWithoutContinuous)
        );
    }

    #[test]
    fn fully_specified_run_passes() {
        let body = format!(
            "termination = {{ max_simulated_seconds = 30.0, fail_fast = true }}{TERMINAL}{CONTINUOUS}"
        );
        assert_eq!(validate(&run_with(&body)), Ok(()));
    }
}
