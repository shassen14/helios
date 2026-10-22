//! Checks, once the pipelines exist, that the run can build every
//! evaluation: each input names one series its agent records, carrying the
//! payload type its role reads, that the collector requests; and the kind
//! accepts the series bound, comparing their frames and the facts their
//! writers declared (the frame values are expressed in and its convention,
//! the rate, the writer), and gives each capped role a cap. Each evaluation
//! that can be built is bound in the collector, so its inputs feed it from
//! the first tick.

use super::{
    locate::{find, spellings, Catalog},
    problems::{join, TargetProblem},
};
use crate::{
    assertion::{AssertionTarget, EntryName},
    collector::{instance_of, EvaluationId, ObservationCollector},
    evaluation::{
        kind::{BoundInput, Cap, Metric, Role},
        Evaluation, EvaluationDefaults, EvaluatorRegistry, Gap,
    },
};

use helios_runtime::{
    observe::key::{RecordedKey, Subject},
    port::{Determinism, ObservableValue, Writer},
};

use thiserror::Error;

/// Between the problems of one evaluation. No problem's message holds it.
const PROBLEM_SEPARATOR: &str = " and ";

/// What a scalar emission carries, for a refusal naming its payload.
const SCALAR: &str = "a number";

/// _Per-step._ An evaluation setup built and bound: what the report says of
/// it, and what an entry reading one of its metrics needs.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedEvaluation {
    pub name: EntryName,
    pub kind: &'static str,
    /// The entry's `[evaluate.params]`, as the run file gave them.
    pub params: toml::Table,
    pub id: EvaluationId,
    /// One per role, in the kind's order.
    pub inputs: Vec<PlannedInput>,
    /// Every metric the bound evaluator offers, in its order, read or not.
    pub metrics: Vec<Metric>,
    /// Whether each derived value holds until the next: when its base is a
    /// channel's, whose value holds until it is written again.
    pub held: bool,
    /// Whether any input is, which makes every metric so.
    pub nondeterministic: bool,
}

/// _Per-step._ One role's input as setup bound it: the selector the run file
/// gave, the key it named, who writes it, and the widest gap it spans.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedInput {
    pub role: &'static str,
    pub target: AssertionTarget,
    pub key: RecordedKey,
    /// `None` only when no description names the series' writer.
    pub writer: Option<Writer>,
    /// `None` for the base and a role held however old.
    pub cap: Option<Cap>,
}

/// _Pre-step._ An evaluation the run can't build, and every reason why.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error("evaluation '{name}': {}", join(.problems, PROBLEM_SEPARATOR))]
pub struct UnusableEvaluation {
    pub name: EntryName,
    pub problems: Vec<EvaluationProblem>,
}

/// _Pre-step._ One reason the run can't build an evaluation.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum EvaluationProblem {
    /// A role's input that names no series the run can feed it.
    #[error("its input '{role}' on '{target}': {problem}")]
    Input {
        role: &'static str,
        target: AssertionTarget,
        problem: TargetProblem,
    },
    /// The kind can't compare the series bound, in its own words.
    #[error("{0}")]
    Refused(String),
    /// A gap given to a role the kind lines up with no cap.
    #[error("its [evaluate.gap.{0}] is unused: the kind gives the {0} no cap, so drop it")]
    UnusedGap(&'static str),
}

/// Each of `evaluations` the run can build, bound in `collector`, and each
/// it can't, both in run-file order. A role its entry gives no gap takes
/// `defaults`'.
pub(super) fn resolve_evaluations(
    evaluations: &[Evaluation],
    defaults: Option<&EvaluationDefaults>,
    registry: &EvaluatorRegistry,
    catalogs: &[Catalog],
    collector: &mut ObservationCollector,
) -> (Vec<PlannedEvaluation>, Vec<UnusableEvaluation>) {
    let mut planned = Vec::new();
    let mut unusable = Vec::new();
    for evaluation in evaluations {
        match plan(evaluation, defaults, registry, catalogs, collector) {
            Ok(evaluation) => planned.push(evaluation),
            Err(problems) => unusable.push(UnusableEvaluation {
                name: evaluation.name.clone(),
                problems,
            }),
        }
    }
    (planned, unusable)
}

/// `evaluation` built and bound in `collector`, or every reason it can't
/// be: each input's problems, or, once every input is bound, the kind's,
/// or each gap it gives a role with no cap.
fn plan(
    evaluation: &Evaluation,
    defaults: Option<&EvaluationDefaults>,
    registry: &EvaluatorRegistry,
    catalogs: &[Catalog],
    collector: &mut ObservationCollector,
) -> Result<PlannedEvaluation, Vec<EvaluationProblem>> {
    // Load checked the kind exists and every role has an input.
    let Some(kind) = registry.get(&evaluation.kind) else {
        return Err(vec![EvaluationProblem::Refused(format!(
            "no evaluator kind '{}'",
            evaluation.kind
        ))]);
    };
    let roles = kind.roles();
    let mut bound = Vec::with_capacity(roles.len());
    let mut problems = Vec::new();
    for role in &roles {
        let Some(target) = evaluation.inputs.get(role.name) else {
            problems.push(EvaluationProblem::Refused(format!(
                "no input for role '{}'",
                role.name
            )));
            continue;
        };
        match locate_input(target, role, catalogs, collector) {
            Ok(input) => bound.push(input),
            Err(found) => {
                problems.extend(found.into_iter().map(|problem| EvaluationProblem::Input {
                    role: role.name,
                    target: target.clone(),
                    problem,
                }))
            }
        }
    }
    if !problems.is_empty() {
        return Err(problems);
    }

    let inputs: Vec<BoundInput> = bound.iter().map(|input| input.facts.clone()).collect();
    let default = defaults.map(|defaults| Gap::Multiple(defaults.gap_multiple));
    let gaps: Vec<Option<Gap>> = roles
        .iter()
        .map(|role| evaluation.gap.get(role.name).copied().or(default))
        .collect();
    let evaluator = kind
        .build(&evaluation.params, &inputs, &gaps)
        .map_err(|refusals| {
            refusals
                .into_iter()
                .map(EvaluationProblem::Refused)
                .collect::<Vec<_>>()
        })?;
    let metrics = evaluator.metrics();
    let caps = evaluator.caps();
    // Whether a role is capped is the bound kind's, so an unused gap is
    // refused here, not at load.
    let unused: Vec<EvaluationProblem> = roles
        .iter()
        .zip(&caps)
        .filter(|(role, cap)| cap.is_none() && evaluation.gap.contains_key(role.name))
        .map(|(role, _)| EvaluationProblem::UnusedGap(role.name))
        .collect();
    if !unused.is_empty() {
        return Err(unused);
    }
    let id = collector.bind_evaluation(
        evaluator,
        bound
            .iter()
            .map(|input| input.planned.key.clone())
            .collect(),
    );

    Ok(PlannedEvaluation {
        name: evaluation.name.clone(),
        kind: kind.name(),
        params: evaluation.params.clone(),
        id,
        held: bound.first().is_some_and(|base| base.held),
        nondeterministic: bound.iter().any(|input| input.nondeterministic),
        metrics,
        inputs: bound
            .into_iter()
            .zip(caps)
            .map(|(input, cap)| PlannedInput {
                cap,
                ..input.planned
            })
            .collect(),
    })
}

/// One role's input, found.
struct Bound {
    planned: PlannedInput,
    /// What the kind is told of it.
    facts: BoundInput,
    /// Whether it is a channel's, whose value holds until the next.
    held: bool,
    nondeterministic: bool,
}

/// The one series `target` names for `role`, or every reason the run can't
/// feed it to the role: it must be one series, of the role's payload type,
/// requested by the collector.
fn locate_input(
    target: &AssertionTarget,
    role: &Role,
    catalogs: &[Catalog],
    collector: &ObservationCollector,
) -> Result<Bound, Vec<TargetProblem>> {
    let (catalog, selected) = find(target, catalogs)?;
    let [(subject, observable)] = selected.as_slice() else {
        let subjects: Vec<&Subject> = selected.iter().map(|(subject, _)| *subject).collect();
        return Err(vec![TargetProblem::InputOnSeveral(spellings(&subjects))]);
    };

    let mut problems = Vec::new();
    let carries = match observable.value() {
        ObservableValue::Scalar { .. } => Some(SCALAR),
        ObservableValue::Typed { type_id, .. } if type_id == role.payload => None,
        ObservableValue::Typed { type_name, .. } => Some(type_name),
    };
    if let Some(carries) = carries {
        problems.push(TargetProblem::WrongPayload {
            carries,
            wants: role.payload_name,
        });
    }
    if !collector.requests(target) {
        problems.push(TargetProblem::Unrequested);
    }
    if !problems.is_empty() {
        return Err(problems);
    }

    // An emission has no description: its node writes it, at no declared
    // rate, in no declared frame.
    let (writer, rate_hz, expressed_in) = match subject {
        Subject::Emission { node, .. } => (
            Some(Writer::Node {
                node: node.to_string(),
            }),
            None,
            None,
        ),
        Subject::Channel { channel, instance } => catalog
            .descriptions
            .iter()
            .find(|description| {
                description.key() == channel
                    && (description.instances().is_empty()
                        || instance
                            .as_ref()
                            .is_some_and(|instance| description.instances().contains(instance)))
            })
            .map_or((None, None, None), |description| {
                (
                    Some(description.writer().clone()),
                    description.rate_hz(),
                    instance
                        .as_ref()
                        .and_then(|instance| description.expressed_in(instance))
                        .cloned(),
                )
            }),
    };
    let convention = expressed_in
        .as_ref()
        .and_then(|frame| catalog.frame_conventions.convention_of(frame));
    Ok(Bound {
        held: matches!(subject, Subject::Channel { .. }),
        nondeterministic: observable.determinism() == Determinism::Nondeterministic,
        facts: BoundInput {
            agent: catalog.agent.clone(),
            instance: instance_of(subject).map(str::to_owned),
            expressed_in,
            convention,
            rate_hz,
            writer: writer.clone(),
        },
        planned: PlannedInput {
            role: role.name,
            target: target.clone(),
            key: RecordedKey {
                agent: catalog.agent.clone(),
                subject: (*subject).clone(),
            },
            writer,
            // The kind gives it once bound.
            cap: None,
        },
    })
}
