use crate::{
    assertion::{condition::OperandProblem, config::WindowProblem, EntryName, TargetRoot},
    evaluation::{kind::EvaluatorKind, path::DerivedName, Align, EvaluatorRegistry},
    run::Run,
    statistics::within_run::Reducer,
};

use std::collections::{HashMap, HashSet};
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
         without `reduce`) of a recorded series on a window known from the first tick (bounds \
         in seconds after run.start) that could fail early; a derived series exists only at \
         finalize"
    )]
    FailFastWithoutEverySample,
    /// Names identify verdicts in the report and across runs, so two
    /// entries can't share one, whether assertions, measures or
    /// evaluations.
    #[error(
        "two entries are named '{0}'; names are unique across [[assert]], [[measure]] and \
         [[evaluate]]"
    )]
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
    /// A window anchored after a derived series. An anchor is the first
    /// stamp of a recorded series; a derived one exists only at finalize.
    #[error("'{name}': its window is anchored after '{anchor}', a derived series; anchor it on a recorded one")]
    AnchorOnDerived {
        name: EntryName,
        anchor: DerivedName,
    },
    #[error("evaluation '{name}': no evaluator kind '{kind}' (there are: {known})")]
    UnknownKind {
        name: EntryName,
        kind: String,
        known: String,
    },
    #[error("evaluation '{name}': its kind reads role '{role}', and [evaluate.inputs] names no series for it")]
    MissingInput { name: EntryName, role: &'static str },
    #[error("evaluation '{name}': its kind has no role '{role}' (it reads {roles})")]
    UnknownInput {
        name: EntryName,
        role: String,
        roles: String,
    },
    #[error("evaluation '{name}': its input '{role}' names a derived series; an evaluation reads recorded series")]
    DerivedInput { name: EntryName, role: String },
    #[error("evaluation '{name}': {problem}")]
    UnusableParams { name: EntryName, problem: String },
    #[error(
        "evaluation '{name}': align = \"{align}\" is not built yet; its first case will be \
         {first_case}. Only \"declared\" is built"
    )]
    AlignNotBuilt {
        name: EntryName,
        align: Align,
        first_case: &'static str,
    },
    #[error("'{name}': '{target}' names no [[evaluate]] entry")]
    UnknownEvaluation {
        name: EntryName,
        target: DerivedName,
    },
    #[error(
        "'{name}': '{target}' names a metric its evaluation doesn't derive (it derives {metrics})"
    )]
    UnknownMetric {
        name: EntryName,
        target: DerivedName,
        metrics: String,
    },
}

/// Check a parsed [`Run`] for coherence. A flat checklist of guard clauses:
/// each looks for one specific incoherence and returns early with its matching
/// variant, so the report can name *which* thing was wrong. Falling through to
/// the end means every check passed.
///
/// Deliberately separate from parsing (`load`) — "can I read this?" and "is
/// this a sane run?" are different questions with different failures.
/// Evaluations are checked against the kinds in `evaluators`, which are
/// code, so a misspelled kind, role or parameter stops the run before a sim
/// starts.
pub fn validate(run: &Run, evaluators: &EvaluatorRegistry) -> Result<(), ValidationError> {
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

    let kinds = validate_evaluations(run, evaluators)?;
    validate_derived_targets(run, &kinds)?;

    validate_observation_covers_target(run)?;

    Ok(())
}

/// Each evaluation's kind, by the evaluation's name, once every evaluation
/// names a registered kind, a series for each of its roles and no other,
/// recorded series only, parameters the kind reads and a frame policy that
/// is built.
fn validate_evaluations<'a, 'r>(
    run: &'a Run,
    evaluators: &'r EvaluatorRegistry,
) -> Result<HashMap<&'a EntryName, &'r dyn EvaluatorKind>, ValidationError> {
    let mut kinds = HashMap::new();
    for evaluation in run.evaluations() {
        let name = &evaluation.name;
        let Some(kind) = evaluators.get(&evaluation.kind) else {
            return Err(ValidationError::UnknownKind {
                name: name.clone(),
                kind: evaluation.kind.clone(),
                known: evaluators.names().join(", "),
            });
        };
        let roles = kind.roles();
        if let Some(role) = roles
            .iter()
            .find(|role| !evaluation.inputs.contains_key(role.name))
        {
            return Err(ValidationError::MissingInput {
                name: name.clone(),
                role: role.name,
            });
        }
        for (role, target) in &evaluation.inputs {
            if !roles.iter().any(|declared| declared.name == role) {
                return Err(ValidationError::UnknownInput {
                    name: name.clone(),
                    role: role.clone(),
                    roles: roles
                        .iter()
                        .map(|role| role.name)
                        .collect::<Vec<_>>()
                        .join(", "),
                });
            }
            if !target.is_recorded() {
                return Err(ValidationError::DerivedInput {
                    name: name.clone(),
                    role: role.clone(),
                });
            }
        }
        if let Err(problem) = kind.check_params(&evaluation.params) {
            return Err(ValidationError::UnusableParams {
                name: name.clone(),
                problem,
            });
        }
        if let Some(first_case) = evaluation.align.unbuilt_until() {
            return Err(ValidationError::AlignNotBuilt {
                name: name.clone(),
                align: evaluation.align,
                first_case,
            });
        }
        kinds.insert(name, kind);
    }
    Ok(kinds)
}

/// Whether each derived series an assertion or measure names is a metric
/// of an evaluation the run declares.
fn validate_derived_targets(
    run: &Run,
    kinds: &HashMap<&EntryName, &dyn EvaluatorKind>,
) -> Result<(), ValidationError> {
    let targets = run
        .assertions()
        .iter()
        .map(|assertion| (&assertion.name, &assertion.target))
        .chain(
            run.measures()
                .iter()
                .map(|measure| (&measure.name, &measure.target)),
        );
    for (name, target) in targets {
        let TargetRoot::Derived(derived) = target.root() else {
            continue;
        };
        let Some(kind) = kinds.get(derived.evaluation()) else {
            return Err(ValidationError::UnknownEvaluation {
                name: name.clone(),
                target: derived.clone(),
            });
        };
        let metrics = kind.metrics();
        if !metrics.iter().any(|metric| metric.name == derived.metric()) {
            return Err(ValidationError::UnknownMetric {
                name: name.clone(),
                target: derived.clone(),
                metrics: metrics
                    .iter()
                    .map(|metric| metric.name)
                    .collect::<Vec<_>>()
                    .join(", "),
            });
        }
    }
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

    use crate::evaluation::standard_evaluators;

    /// `run` checked against the standard evaluator kinds.
    fn check(run: &Run) -> Result<(), ValidationError> {
        validate(run, &standard_evaluators())
    }

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

    /// A run with one `twist_tracking` evaluation named "twist", its body
    /// edited by `edit`, and `rest` (more blocks).
    fn evaluating(edit: impl Fn(String) -> String, rest: &str) -> Run {
        let twist = edit(
            "[[evaluate]]\nname = \"twist\"\nkind = \"twist_tracking\"\n\
             [evaluate.inputs]\n\
             truth = { path = \"agent.car.channel.truth.twist\", instance = \"agent.car.base_link\" }\n\
             reference = \"agent.car.channel.reference\"\n"
                .to_owned(),
        );
        run_with(&format!("{BUDGET}\n{rest}\n{twist}"))
    }

    /// An assertion named `name` on `target`, reduced by its max.
    fn max_of(name: &str, target: &str) -> String {
        format!(
            "[[assert]]\nname = \"{name}\"\ntarget = \"{target}\"\nreduce = \"max\"\n\
             condition = {{ less_than = 1.0 }}"
        )
    }

    #[test]
    fn an_evaluation_with_an_assertion_on_its_metric_is_ok() {
        let run = evaluating(|twist| twist, &max_of("a", "eval.inline.twist.surge_error"));
        assert_eq!(check(&run), Ok(()));
    }

    #[test]
    fn an_unknown_kind_lists_the_known_ones() {
        let run = evaluating(|twist| twist.replace("twist_tracking", "goal_distance"), "");
        let error = check(&run).expect_err("no such kind");
        assert!(matches!(error, ValidationError::UnknownKind { .. }));
        assert!(
            error.to_string().ends_with("(there are: twist_tracking)"),
            "{error}"
        );
    }

    #[test]
    fn a_missing_or_unknown_role_is_refused() {
        let missing = evaluating(
            |twist| twist.replace("reference = \"agent.car.channel.reference\"\n", ""),
            "",
        );
        assert_eq!(
            check(&missing),
            Err(ValidationError::MissingInput {
                name: EntryName::parse("twist").unwrap(),
                role: "reference",
            })
        );

        let unknown = evaluating(|twist| twist + "goal = \"agent.car.channel.mission\"\n", "");
        assert!(matches!(
            check(&unknown),
            Err(ValidationError::UnknownInput { role, .. }) if role == "goal"
        ));
    }

    #[test]
    fn an_input_naming_a_derived_series_is_refused() {
        let run = evaluating(
            |twist| {
                twist.replace(
                    "\"agent.car.channel.reference\"",
                    "\"eval.inline.other.surge_error\"",
                )
            },
            "",
        );
        assert!(matches!(
            check(&run),
            Err(ValidationError::DerivedInput { .. })
        ));
    }

    #[test]
    fn a_parameter_the_kind_doesnt_read_is_refused() {
        let run = evaluating(|twist| twist + "[evaluate.params]\ngain = 2.0\n", "");
        assert!(matches!(
            check(&run),
            Err(ValidationError::UnusableParams { .. })
        ));
    }

    #[test]
    fn a_frame_policy_not_built_names_its_first_case() {
        let run = evaluating(
            |twist| twist.replace("kind = ", "align = \"best_fit\"\nkind = "),
            "",
        );
        let error = check(&run).expect_err("best_fit is not built");
        assert!(
            error.to_string().contains("the first hardware or SLAM run"),
            "{error}"
        );
    }

    #[test]
    fn a_derived_target_must_name_a_declared_evaluation_and_one_of_its_metrics() {
        let no_evaluation = evaluating(|twist| twist, &max_of("a", "eval.inline.goal.distance"));
        assert!(matches!(
            check(&no_evaluation),
            Err(ValidationError::UnknownEvaluation { .. })
        ));

        let no_metric = evaluating(|twist| twist, &max_of("a", "eval.inline.twist.speed_error"));
        let error = check(&no_metric).expect_err("no such metric");
        assert!(
            error.to_string().ends_with(
                "(it derives surge_error, sway_error, heave_error, roll_rate_error, \
                     pitch_rate_error, yaw_rate_error)"
            ),
            "{error}"
        );
    }

    #[test]
    fn an_evaluation_shares_the_name_space_of_assertions_and_measures() {
        let run = evaluating(
            |twist| twist,
            &max_of("twist", "eval.inline.twist.surge_error"),
        );
        assert_eq!(
            check(&run),
            Err(ValidationError::DuplicateName(
                EntryName::parse("twist").unwrap()
            ))
        );
    }

    #[test]
    fn a_window_anchored_after_a_derived_series_is_refused() {
        let anchored = "[[assert]]\nname = \"a\"\ntarget = \"agent.car.node.estimator.x\"\n\
             window = { start = { after = \"eval.inline.twist.surge_error\" } }\n\
             condition = { less_than = 1.0 }";
        let run = evaluating(|twist| twist, anchored);
        assert!(matches!(
            check(&run),
            Err(ValidationError::AnchorOnDerived { .. })
        ));
    }

    #[test]
    fn fail_fast_watches_no_derived_series() {
        // A derived series exists only at finalize, so an every-sample
        // assertion on it can't stop the run early.
        let every_sample = "[[assert]]\nname = \"a\"\ntarget = \"eval.inline.twist.surge_error\"\n\
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
