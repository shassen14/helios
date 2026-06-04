use thiserror::Error;

use crate::run::Run;

/// _Pre-step._ Coherence problems a structurally-valid run file can still have. Parsing
/// proves the *shape* is right; these are the cross-field checks serde can't
/// express. Caught at load time so a nonsensical run fails fast instead of
/// being discovered partway through a simulation.
#[derive(Debug, PartialEq, Error)]
pub enum ValidationError {
    #[error("no stop condition declared: the run would never terminate")]
    NoTermination,
    #[error("`on_assertion` is set but the run declares no assertions for it to watch")]
    OnAssertionWithoutAssertions,
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

    // A run with no stop trigger at all never terminates.
    if termination.max_simulated_seconds().is_none() && termination.on_assertion().is_none() {
        return Err(ValidationError::NoTermination);
    }

    // An `on_assertion` trigger fires when assertions reach a pass/fail state,
    // so with zero assertions it can never fire — the trigger watches nothing.
    if termination.on_assertion().is_some() && run.assertions().is_empty() {
        return Err(ValidationError::OnAssertionWithoutAssertions);
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

    const ASSERT: &str = r#"
        [[assert]]
        when = "at_completion"
        target = "agent.car.x"
        condition = "equals"
        value = 0.0
    "#;

    #[test]
    fn no_trigger_is_no_termination() {
        // Empty termination table → both triggers `None` → never stops.
        let run = run_with("termination = {}");
        assert_eq!(validate(&run), Err(ValidationError::NoTermination));
    }

    #[test]
    fn time_budget_alone_is_ok_without_assertions() {
        // A pure smoke run: the time budget guarantees termination, so zero
        // assertions is fine.
        let run = run_with("termination = { max_simulated_seconds = 5.0 }");
        assert_eq!(validate(&run), Ok(()));
    }

    #[test]
    fn on_assertion_without_assertions_errors() {
        let run = run_with(r#"termination = { on_assertion = "any_passed" }"#);
        assert_eq!(
            validate(&run),
            Err(ValidationError::OnAssertionWithoutAssertions)
        );
    }

    #[test]
    fn on_assertion_with_an_assertion_is_ok() {
        let body = format!(r#"termination = {{ on_assertion = "any_passed" }}{ASSERT}"#);
        let run = run_with(&body);
        assert_eq!(validate(&run), Ok(()));
    }

    #[test]
    fn fully_specified_run_passes() {
        let body = format!(
            r#"termination = {{ max_simulated_seconds = 30.0, on_assertion = "any_failed" }}{ASSERT}"#
        );
        let run = run_with(&body);
        assert_eq!(validate(&run), Ok(()));
    }
}
