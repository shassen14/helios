//! Evaluations against the kinds they name, and the derived series an
//! assertion or measure reads against the evaluations that offer them.

use super::ValidationError;
use crate::{
    assertion::{EntryName, TargetRoot},
    evaluation::{kind::EvaluatorKind, EvaluatorRegistry},
    run::Run,
};

use std::collections::HashMap;

/// Each evaluation's kind, by the evaluation's name, once every evaluation
/// names a registered kind, a series for each of its roles and no other,
/// recorded series only, parameters the kind reads, gaps for its roles only,
/// and a frame policy that is built and that the kind applies.
pub(super) fn check_evaluations<'a, 'r>(
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
        if let Some(role) = evaluation
            .gap
            .keys()
            .find(|role| !roles.iter().any(|declared| declared.name == *role))
        {
            return Err(ValidationError::UnknownGapRole {
                name: name.clone(),
                role: role.clone(),
                roles: roles
                    .iter()
                    .map(|role| role.name)
                    .collect::<Vec<_>>()
                    .join(", "),
            });
        }
        kinds.insert(name, kind);
    }
    Ok(kinds)
}

/// Whether each derived series an assertion or measure names is a metric
/// of an evaluation the run declares.
pub(super) fn check_derived_targets(
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
        // A menu known only once bound is checked at setup.
        let Some(metrics) = kind.menu() else {
            continue;
        };
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

#[cfg(test)]
mod tests {
    use super::{super::fixtures::*, *};

    #[test]
    fn an_evaluation_with_an_assertion_on_its_metric_is_ok() {
        let run = evaluating(|twist| twist, &max_of("a", "eval.inline.twist.surge"));
        assert_eq!(check(&run), Ok(()));
    }

    #[test]
    fn a_gap_naming_no_role_of_the_kind_is_refused_listing_its_roles() {
        let gap = |role: &str| {
            check(&evaluating(
                |twist| format!("{twist}[evaluate.gap.{role}]\nseconds = 0.5\n"),
                "",
            ))
        };

        assert_eq!(gap("reference"), Ok(()));
        assert_eq!(
            gap("goal"),
            Err(ValidationError::UnknownGapRole {
                name: EntryName::parse("twist").unwrap(),
                role: "goal".to_owned(),
                roles: "truth, reference".to_owned(),
            })
        );
    }

    #[test]
    fn an_unknown_kind_lists_the_known_ones() {
        let run = evaluating(|twist| twist.replace("twist_tracking", "goal_distance"), "");
        let error = check(&run).expect_err("no such kind");
        assert!(matches!(error, ValidationError::UnknownKind { .. }));
        assert!(
            error
                .to_string()
                .ends_with("(there are: pose, pose_error, twist, twist_tracking)"),
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
                    "\"eval.inline.other.surge\"",
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
    fn a_frame_policy_on_a_kind_that_relates_no_frames_is_a_parameter_it_doesnt_read() {
        let run = evaluating(
            |twist| twist + "[evaluate.params]\nalign = \"first_pose\"\n",
            "",
        );
        let error = check(&run).expect_err("twist_tracking reads no parameters");
        assert_eq!(
            error.to_string(),
            "evaluation 'twist': twist_tracking: unknown field `align`, there are no fields"
        );
    }

    #[test]
    fn a_derived_target_must_name_a_declared_evaluation_and_one_of_its_metrics() {
        let no_evaluation = evaluating(|twist| twist, &max_of("a", "eval.inline.goal.distance"));
        assert!(matches!(
            check(&no_evaluation),
            Err(ValidationError::UnknownEvaluation { .. })
        ));

        let no_metric = evaluating(|twist| twist, &max_of("a", "eval.inline.twist.surge_error"));
        let error = check(&no_metric).expect_err("no such metric");
        assert!(
            error.to_string().ends_with(
                "(it offers surge, sway, heave, roll_rate, pitch_rate, yaw_rate, speed, velocity)"
            ),
            "{error}"
        );
    }
}
