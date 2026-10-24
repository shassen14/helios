//! The run's stop: a budget the clock reaches, and a `fail_fast` with
//! something it could stop on.

use super::ValidationError;
use crate::run::Run;

/// The run's time budget, once it is one the clock reaches and `fail_fast`,
/// when set, has an assertion it could stop on.
pub(super) fn check_termination(run: &Run) -> Result<f64, ValidationError> {
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
    // window isn't known until the run stops or an anchor occurs, and one
    // on a derived series, which exists only then. Without an every-sample
    // one on a fixed window of a recorded series it watches nothing.
    let has_every_sample = run.assertions().iter().any(|assertion| {
        assertion.judges_every_sample()
            && assertion.window.fixed()
            && assertion.target.is_recorded()
    });
    if termination.fail_fast() && !has_every_sample {
        return Err(ValidationError::FailFastWithoutEverySample);
    }

    Ok(budget)
}

#[cfg(test)]
mod tests {
    use super::{super::fixtures::*, *};

    #[test]
    fn time_budget_alone_is_ok_without_assertions() {
        // A pure smoke run: the time budget guarantees termination, so zero
        // assertions is fine.
        assert_eq!(check(&run_with(BUDGET)), Ok(()));
    }

    #[test]
    fn a_budget_the_clock_never_reaches_errors() {
        for budget in ["0.0", "-1.0", "nan", "inf"] {
            let run = run_with(&format!(
                "termination = {{ max_simulated_seconds = {budget} }}"
            ));
            assert!(
                matches!(check(&run), Err(ValidationError::UnreachableBudget(_))),
                "budget {budget}"
            );
        }
    }

    #[test]
    fn fail_fast_without_assertions_errors() {
        let run = run_with("termination = { max_simulated_seconds = 5.0, fail_fast = true }");
        assert_eq!(
            check(&run),
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
            check(&run_with(&body)),
            Err(ValidationError::FailFastWithoutEverySample)
        );
    }

    #[test]
    fn fully_specified_run_passes() {
        let body = format!(
            "termination = {{ max_simulated_seconds = 30.0, fail_fast = true }}{REDUCED}{EVERY_SAMPLE}"
        );
        assert_eq!(check(&run_with(&body)), Ok(()));
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
            check(&run_with(body)),
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
        assert_eq!(check(&run_with(body)), Ok(()));
    }

    #[test]
    fn fail_fast_watches_no_derived_series() {
        // A derived series exists only at finalize, so an every-sample
        // assertion on it can't stop the run early.
        let every_sample = "[[assert]]\nname = \"a\"\ntarget = \"eval.inline.twist.surge\"\n\
             condition = { less_than = 1.0 }";
        let text = format!(
            "termination = {{ max_simulated_seconds = 5.0, fail_fast = true }}\n{every_sample}\n\
             [[evaluate]]\nname = \"twist\"\nkind = \"twist_tracking\"\n[evaluate.inputs]\n\
             truth = \"agent.car.channel.truth.twist\"\nreference = \"agent.car.channel.reference\"\n"
        );
        assert_eq!(
            check(&run_with(&text)),
            Err(ValidationError::FailFastWithoutEverySample)
        );
    }
}
