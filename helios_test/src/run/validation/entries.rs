//! What every assertion, measure and evaluation must be on its own: a name
//! no other entry has, operands a condition can compare against, a
//! threshold that is a number, and a window the run can place.

use super::ValidationError;
use crate::{assertion::TargetRoot, run::Run, statistics::within_run::Reducer};

use std::collections::HashSet;

/// Whether every entry is unique by name, each condition's operand usable,
/// each `time_above` threshold finite, and each window placeable within
/// `budget` and anchored on a recorded series.
pub(super) fn check_entries(run: &Run, budget: f64) -> Result<(), ValidationError> {
    let names = run
        .assertions()
        .iter()
        .map(|assertion| &assertion.name)
        .chain(run.measures().iter().map(|measure| &measure.name))
        .chain(run.evaluations().iter().map(|evaluation| &evaluation.name));
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
        for anchor in window.anchors() {
            if let TargetRoot::Derived(derived) = anchor.root() {
                return Err(ValidationError::AnchorOnDerived {
                    name: name.clone(),
                    anchor: derived.clone(),
                });
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{super::fixtures::*, *};
    use crate::assertion::{config::WindowProblem, EntryName};

    #[test]
    fn a_name_used_twice_is_refused_across_assertions_and_measures() {
        let measure = r#"
            [[measure]]
            name = "reduced"
            target = "agent.car.node.estimator.x"
            reduce = "mean"
        "#;
        assert_eq!(
            check(&run_with(&format!("{BUDGET}{REDUCED}{measure}"))),
            Err(ValidationError::DuplicateName(
                EntryName::parse("reduced").unwrap()
            ))
        );
        assert!(matches!(
            check(&run_with(&format!("{BUDGET}{REDUCED}{REDUCED}"))),
            Err(ValidationError::DuplicateName(_))
        ));
    }

    #[test]
    fn an_evaluation_shares_the_name_space_of_assertions_and_measures() {
        let run = evaluating(|twist| twist, &max_of("twist", "eval.inline.twist.surge"));
        assert_eq!(
            check(&run),
            Err(ValidationError::DuplicateName(
                EntryName::parse("twist").unwrap()
            ))
        );
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
                    check(&asserting("bounded", "", condition)),
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
                check(&asserting("bounded", "", condition)),
                Ok(()),
                "{condition}"
            );
        }
    }

    #[test]
    fn a_non_finite_time_above_threshold_is_refused_on_an_assertion_or_a_measure() {
        assert!(matches!(
            check(&asserting(
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
            check(&run_with(&format!("{BUDGET}{measure}"))),
            Err(ValidationError::NonFiniteThreshold(_))
        ));
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
            check(&run_with(&format!("{BUDGET}{measure}"))),
            Err(ValidationError::UnusableWindow {
                name: EntryName::parse("late").unwrap(),
                problem: WindowProblem::PastBudget {
                    end: 7.0,
                    budget: 5.0,
                },
            })
        );
    }

    #[test]
    fn a_window_anchored_after_a_derived_series_is_refused() {
        let anchored = "[[assert]]\nname = \"a\"\ntarget = \"agent.car.node.estimator.x\"\n\
             window = { start = { after = \"eval.inline.twist.surge\" } }\n\
             condition = { less_than = 1.0 }";
        let run = evaluating(|twist| twist, anchored);
        assert!(matches!(
            check(&run),
            Err(ValidationError::AnchorOnDerived { .. })
        ));
    }
}
