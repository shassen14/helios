//! Evaluations against the kinds they name, inline and in each suite, and
//! the derived series an assertion or measure reads against the evaluations
//! that offer them.

use super::ValidationError;
use crate::{
    assertion::{EntryName, TargetRoot},
    evaluation::{
        kind::EvaluatorKind,
        path::{AgentEntry, DerivedName, EvaluationName},
        suite::{AgentSelection, Suite},
        Evaluation, EvaluatorRegistry,
    },
    run::Run,
};

use std::collections::HashMap;

/// The kind of each evaluation a run declares: an inline one's by its name,
/// a suite's by the name it was applied under and its entry's.
pub(super) struct Kinds<'a, 'r> {
    inline: HashMap<&'a EntryName, &'r dyn EvaluatorKind>,
    suites: HashMap<&'a EntryName, AppliedKinds<'a, 'r>>,
}

/// One applied suite, and its entries' kinds by name.
struct AppliedKinds<'a, 'r> {
    suite: &'a Suite,
    entries: HashMap<&'a EntryName, &'r dyn EvaluatorKind>,
}

/// Each evaluation's kind, once every evaluation, inline or in a suite,
/// names a registered kind, a series for each of its roles and no other,
/// parameters the kind reads and gaps for its roles only; an inline one
/// reads recorded series only, and a suite's reads, in each role, a series
/// carrying the payload type the role reads, when the series' type is fixed.
/// A suite's entries are named apart.
pub(super) fn check_evaluations<'a, 'r>(
    run: &'a Run,
    evaluators: &'r EvaluatorRegistry,
) -> Result<Kinds<'a, 'r>, ValidationError> {
    let mut inline = HashMap::new();
    for evaluation in run.evaluations() {
        let kind = check_evaluation(evaluation, evaluators, |_, role, target| {
            if target.is_recorded() {
                Ok(())
            } else {
                Err(ValidationError::DerivedInput {
                    name: evaluation.name.clone(),
                    role: role.to_owned(),
                })
            }
        })?;
        inline.insert(&evaluation.name, kind);
    }

    let mut suites = HashMap::new();
    for suite in run.suites() {
        let in_suite = |error| ValidationError::Suite {
            suite: suite.application.name.clone(),
            from: suite.application.from.clone(),
            error: Box::new(error),
        };
        let mut entries = HashMap::new();
        for evaluation in &suite.config.evaluations {
            if entries.contains_key(&evaluation.name) {
                return Err(in_suite(ValidationError::DuplicateSuiteEntry(
                    evaluation.name.clone(),
                )));
            }
            // A series whose type is fixed is refused here, before a sim
            // starts; the reference's type is known only once bound.
            let kind = check_evaluation(evaluation, evaluators, |kind, role, &series| {
                let index = kind
                    .roles()
                    .iter()
                    .position(|declared| declared.name == role);
                match index.map(|index| kind.series_key(index, series)) {
                    Some(Err(problem)) => Err(ValidationError::UnfitSeries {
                        name: evaluation.name.clone(),
                        problem,
                    }),
                    _ => Ok(()),
                }
            })
            .map_err(in_suite)?;
            entries.insert(&evaluation.name, kind);
        }
        suites.insert(&suite.application.name, AppliedKinds { suite, entries });
    }
    Ok(Kinds { inline, suites })
}

/// `evaluation`'s kind, once it names a registered kind, an input for each
/// of its roles and no other, each passing `check_input` (given the kind,
/// the role's name and its input), parameters the kind reads, and gaps for
/// its roles only.
fn check_evaluation<'r, I>(
    evaluation: &Evaluation<I>,
    evaluators: &'r EvaluatorRegistry,
    check_input: impl Fn(&dyn EvaluatorKind, &str, &I) -> Result<(), ValidationError>,
) -> Result<&'r dyn EvaluatorKind, ValidationError> {
    let name = &evaluation.name;
    let Some(kind) = evaluators.get(&evaluation.kind) else {
        return Err(ValidationError::UnknownKind {
            name: name.clone(),
            kind: evaluation.kind.clone(),
            known: evaluators.names().join(", "),
        });
    };
    let roles = kind.roles();
    let role_names = || {
        roles
            .iter()
            .map(|role| role.name)
            .collect::<Vec<_>>()
            .join(", ")
    };
    if let Some(role) = roles
        .iter()
        .find(|role| !evaluation.inputs.contains_key(role.name))
    {
        return Err(ValidationError::MissingInput {
            name: name.clone(),
            role: role.name,
        });
    }
    for (role, input) in &evaluation.inputs {
        if !roles.iter().any(|declared| declared.name == role) {
            return Err(ValidationError::UnknownInput {
                name: name.clone(),
                role: role.clone(),
                roles: role_names(),
            });
        }
        check_input(kind, role, input)?;
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
            roles: role_names(),
        });
    }
    Ok(kind)
}

/// Whether each derived series an assertion or measure names is a metric
/// of an evaluation the run declares: an inline entry, or an entry of a
/// suite applied under that name, to that agent when the suite lists its
/// agents. Under `"*"`, whether the run has the agent is known only at
/// setup.
pub(super) fn check_derived_targets(run: &Run, kinds: &Kinds) -> Result<(), ValidationError> {
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
        let kind = kind_of(derived, kinds).map_err(|why| match why {
            None => ValidationError::UnknownEvaluation {
                name: name.clone(),
                target: derived.clone(),
            },
            Some(why) => ValidationError::UnappliedSuiteTarget {
                name: name.clone(),
                target: derived.clone(),
                why,
            },
        })?;
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

/// The kind of the evaluation `derived` is a metric of, or why the run
/// declares none: `None` for an inline name, which no entry has, or what
/// is wrong with a suite's.
fn kind_of<'r>(
    derived: &DerivedName,
    kinds: &Kinds<'_, 'r>,
) -> Result<&'r dyn EvaluatorKind, Option<String>> {
    match derived.evaluation() {
        EvaluationName::Inline(entry) => kinds.inline.get(entry).copied().ok_or(None),
        EvaluationName::Agent(named) => {
            let AgentEntry {
                agent,
                application,
                entry,
            } = &**named;
            let Some(applied) = kinds.suites.get(application) else {
                return Err(Some(format!(
                    "names no [[suite]]: none is named '{application}'"
                )));
            };
            let Some(kind) = applied.entries.get(entry) else {
                let mut names: Vec<&str> =
                    applied.entries.keys().map(|name| name.as_str()).collect();
                names.sort_unstable();
                return Err(Some(format!(
                    "names no entry of suite '{application}': it has no '{entry}' (it has {})",
                    names.join(", ")
                )));
            };
            if let AgentSelection::Listed(agents) = &applied.suite.application.agents {
                if !agents.contains(agent) {
                    let listed: Vec<&str> = agents.iter().map(|agent| agent.as_str()).collect();
                    return Err(Some(format!(
                        "names agent '{}', and suite '{application}' is applied to {} only",
                        agent.as_str(),
                        listed.join(", ")
                    )));
                }
            }
            Ok(*kind)
        }
    }
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

    #[test]
    fn a_suite_with_an_assertion_on_its_metric_on_a_listed_agent_is_ok() {
        let run = applying(
            &max_of("a", "eval.agent.car.tracking.twist.surge"),
            &[("tracking", "[\"car\"]", TWIST_SUITE)],
        );
        assert_eq!(check(&run), Ok(()));
    }

    #[test]
    fn a_suites_entries_are_checked_as_inline_ones_naming_the_suite() {
        let run = applying(
            "",
            &[(
                "tracking",
                "\"*\"",
                &TWIST_SUITE.replace("twist_tracking", "goal_distance"),
            )],
        );
        let error = check(&run).expect_err("no such kind");
        assert!(
            matches!(&error, ValidationError::Suite { error, .. }
                if matches!(**error, ValidationError::UnknownKind { .. })),
            "{error:?}"
        );
        assert!(
            error.to_string().starts_with(
                "suite 'tracking' (tracking.toml): evaluation 'twist': no evaluator kind"
            ),
            "{error}"
        );
    }

    #[test]
    fn a_fixed_type_series_read_by_a_role_of_another_type_is_refused_at_load() {
        let run = applying(
            "",
            &[(
                "tracking",
                "\"*\"",
                &TWIST_SUITE.replace("\"truth.twist\"", "\"truth.pose\""),
            )],
        );
        let error = check(&run).expect_err("a pose is not a twist");
        assert!(
            matches!(&error, ValidationError::Suite { error, .. }
                if matches!(**error, ValidationError::UnfitSeries { .. })),
            "{error:?}"
        );
        assert!(
            error.to_string().contains("`truth.pose` carries"),
            "{error}"
        );
    }

    #[test]
    fn two_entries_of_one_suite_may_not_share_a_name() {
        let run = applying(
            "",
            &[("tracking", "\"*\"", &format!("{TWIST_SUITE}{TWIST_SUITE}"))],
        );
        assert!(matches!(
            check(&run),
            Err(ValidationError::Suite { error, .. })
                if *error == ValidationError::DuplicateSuiteEntry(EntryName::parse("twist").unwrap())
        ));
    }

    #[test]
    fn a_suite_shares_the_name_space_of_the_runs_entries() {
        let run = applying(
            &max_of("tracking", "agent.car.node.estimator.x"),
            &[("tracking", "\"*\"", TWIST_SUITE)],
        );
        assert_eq!(
            check(&run),
            Err(ValidationError::DuplicateName(
                EntryName::parse("tracking").unwrap()
            ))
        );
    }

    #[test]
    fn a_suite_metric_names_an_applied_suite_its_entry_and_a_listed_agent() {
        let suites = [("tracking", "[\"car\"]", TWIST_SUITE)];
        for (target, why) in [
            (
                "eval.agent.car.steering.twist.surge",
                "none is named 'steering'",
            ),
            (
                "eval.agent.car.tracking.yaw.surge",
                "it has no 'yaw' (it has twist)",
            ),
            (
                "eval.agent.truck.tracking.twist.surge",
                "applied to car only",
            ),
        ] {
            let error = check(&applying(&max_of("a", target), &suites)).expect_err(target);
            assert!(
                matches!(error, ValidationError::UnappliedSuiteTarget { .. }),
                "{target}: {error:?}"
            );
            assert!(error.to_string().contains(why), "{target}: {error}");
        }
    }

    #[test]
    fn under_every_agent_any_agent_passes_load_and_the_metric_is_still_checked() {
        let suites = [("tracking", "\"*\"", TWIST_SUITE)];
        assert_eq!(
            check(&applying(
                &max_of("a", "eval.agent.truck.tracking.twist.surge"),
                &suites
            )),
            Ok(())
        );
        assert!(matches!(
            check(&applying(
                &max_of("a", "eval.agent.truck.tracking.twist.surge_error"),
                &suites
            )),
            Err(ValidationError::UnknownMetric { .. })
        ));
    }
}
