//! Checks, once the pipelines exist, that the run can build every
//! evaluation, inline and each suite's entry on each agent it is applied
//! to: each input names one series its agent records, carrying the payload
//! type its role reads (a suite's agent series is present when the agent's
//! catalog has its channel and instance); and the kind
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
    assertion::AssertionTarget,
    collector::{instance_of, EvaluationId, ObservationCollector},
    evaluation::{
        kind::{BoundInput, Cap, Metric, Role},
        path::EvaluationName,
        suite::{self, AgentSelection, Suite},
        AgentSeries, Evaluation, EvaluationDefaults, EvaluatorRegistry, Gap,
    },
};

use helios_core::prelude::AgentId;
use helios_runtime::{
    observe::key::{RecordedKey, Subject},
    pipeline::StateDescription,
    port::{Determinism, ObservableValue, Writer},
};

use std::collections::BTreeMap;
use thiserror::Error;

/// Between the problems of one evaluation. No problem's message holds it.
const PROBLEM_SEPARATOR: &str = " and ";

/// What a scalar emission carries, for a refusal naming its payload.
const SCALAR: &str = "a number";

/// Between the agents, or the instances, a problem lists.
const LIST_SEPARATOR: &str = ", ";

/// _Per-step._ An evaluation setup built and bound: what the report says of
/// it, and what an entry reading one of its metrics needs.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedEvaluation {
    pub name: EvaluationName,
    pub kind: &'static str,
    /// The entry's `[evaluate.params]`, as the run file gave them.
    pub params: toml::Table,
    /// What the bound evaluator fixed beside them, as it states it.
    pub binding: toml::Table,
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
/// gave or a suite's agent series was bound to, the key it named, who writes
/// it, and the widest gap it spans.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedInput {
    pub role: &'static str,
    /// The agent series a suite's entry read, `None` for an inline entry.
    pub series: Option<AgentSeries>,
    pub target: AssertionTarget,
    pub key: RecordedKey,
    /// `None` only when no description names the series' writer.
    pub writer: Option<Writer>,
    /// `None` for a role held however old.
    pub cap: Option<Cap>,
}

/// _Pre-step._ An evaluation the run can't build, and every reason why.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error("evaluation '{name}': {}", join(.problems, PROBLEM_SEPARATOR))]
pub struct UnusableEvaluation {
    pub name: EvaluationName,
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
    /// A suite applied to an agent the run doesn't have, with those it has.
    #[error("the run has no agent by that name (it has {})", .0.join(LIST_SEPARATOR))]
    UnknownAgent(Vec<String>),
    /// A suite's role reading an agent series the agent doesn't write, at
    /// the selector it was bound to.
    #[error("{}", missing_series(.role, *.series, .target, .agent, .declared.as_deref()))]
    MissingSeries {
        role: &'static str,
        series: AgentSeries,
        target: AssertionTarget,
        agent: AgentId,
        /// The instances its channel declares, when it has the channel
        /// and not the series' instance.
        declared: Option<Vec<String>>,
    },
    /// A gap given to a role the kind lines up with no cap.
    #[error("its [evaluate.gap.{0}] is unused: the kind gives the {0} no cap, so drop it")]
    UnusedGap(&'static str),
}

/// _Per-step._ What a run declares to evaluate: its inline entries with the
/// run's `[evaluation]`, and its suites, each with its own.
#[derive(Debug, Clone, Copy, Default)]
pub(in crate::runner) struct RunEvaluations<'r> {
    pub(in crate::runner) inline: &'r [Evaluation],
    pub(in crate::runner) defaults: Option<&'r EvaluationDefaults>,
    pub(in crate::runner) suites: &'r [Suite],
}

/// Each evaluation the run can build, bound in `collector`, and each it
/// can't, both in run-file order: the inline entries, then each suite's
/// entries on each agent it is applied to, agent by agent (under `"*"`, in
/// the order of `catalogs`). A role its entry gives no gap takes its own
/// file's `[evaluation]`: the run's for an inline entry, the suite's for a
/// suite's.
pub(super) fn resolve_evaluations(
    run: RunEvaluations,
    registry: &EvaluatorRegistry,
    catalogs: &[Catalog],
    collector: &mut ObservationCollector,
) -> (Vec<PlannedEvaluation>, Vec<UnusableEvaluation>) {
    let mut planned = Vec::new();
    let mut unusable = Vec::new();
    let mut sort = |name: EvaluationName, result| match result {
        Ok(evaluation) => planned.push(evaluation),
        Err(problems) => unusable.push(UnusableEvaluation { name, problems }),
    };
    for evaluation in run.inline {
        let name = EvaluationName::Inline(evaluation.name.clone());
        let result = plan(
            &name,
            evaluation,
            Origin::Inline,
            run.defaults,
            registry,
            catalogs,
            collector,
        );
        sort(name, result);
    }
    for suite in run.suites {
        let agents: Vec<&AgentId> = match &suite.application.agents {
            AgentSelection::Every => catalogs.iter().map(|catalog| catalog.agent).collect(),
            AgentSelection::Listed(agents) => agents.iter().collect(),
        };
        for agent in agents {
            for entry in &suite.config.evaluations {
                let name = EvaluationName::agent(
                    agent.clone(),
                    suite.application.name.clone(),
                    entry.name.clone(),
                );
                let origin = Origin::Suite {
                    agent,
                    series: &entry.inputs,
                };
                let result = bind(entry, agent, registry, catalogs).and_then(|evaluation| {
                    plan(
                        &name,
                        &evaluation,
                        origin,
                        suite.config.defaults.as_ref(),
                        registry,
                        catalogs,
                        collector,
                    )
                });
                sort(name, result);
            }
        }
    }
    (planned, unusable)
}

/// The inline entry the suite's `entry` stands for on `agent`, once the run
/// has the agent, or every reason it can't be bound. Whether the agent
/// writes each series, and with its role's type, is the input check's, at
/// [`plan`], as for an inline entry.
fn bind(
    entry: &Evaluation<AgentSeries>,
    agent: &AgentId,
    registry: &EvaluatorRegistry,
    catalogs: &[Catalog],
) -> Result<Evaluation, Vec<EvaluationProblem>> {
    if !catalogs.iter().any(|catalog| catalog.agent == agent) {
        return Err(vec![EvaluationProblem::UnknownAgent(
            catalogs
                .iter()
                .map(|catalog| catalog.agent.as_str().to_owned())
                .collect(),
        )]);
    }
    // Load checked the kind exists and each fixed series fits its role.
    let Some(kind) = registry.get(&entry.kind) else {
        return Err(vec![EvaluationProblem::Refused(format!(
            "no evaluator kind '{}'",
            entry.kind
        ))]);
    };
    suite::bind(entry, agent, kind).map_err(|reasons| {
        reasons
            .into_iter()
            .map(EvaluationProblem::Refused)
            .collect::<Vec<_>>()
    })
}

/// Where an evaluation to plan was written: in the run file, or as a
/// suite's entry bound to one agent.
#[derive(Clone, Copy)]
enum Origin<'a> {
    Inline,
    Suite {
        agent: &'a AgentId,
        /// The agent series each role reads, by role.
        series: &'a BTreeMap<String, AgentSeries>,
    },
}

impl Origin<'_> {
    /// The agent series `role` reads, for a suite's entry.
    fn series(&self, role: &str) -> Option<AgentSeries> {
        match self {
            Origin::Inline => None,
            Origin::Suite { series, .. } => series.get(role).copied(),
        }
    }

    /// What is wrong with `role`'s input on `target`: for a suite's entry, a
    /// series its agent doesn't write is named by the series and its
    /// supplier, not by the selector it was bound to.
    fn input_problem(
        &self,
        role: &'static str,
        target: &AssertionTarget,
        problem: TargetProblem,
    ) -> EvaluationProblem {
        let input = |problem| EvaluationProblem::Input {
            role,
            target: target.clone(),
            problem,
        };
        let Origin::Suite { agent, series } = self else {
            return input(problem);
        };
        let Some(&series) = series.get(role) else {
            return input(problem);
        };
        let declared = match &problem {
            TargetProblem::Unreported => None,
            TargetProblem::UndeclaredInstance { declared, .. } => Some(declared.clone()),
            _ => return input(problem),
        };
        EvaluationProblem::MissingSeries {
            role,
            series,
            target: target.clone(),
            agent: (*agent).clone(),
            declared,
        }
    }
}

/// Why `agent` has no `series` at `target` for `role`: it has no channel
/// there, so no supplier of the series, or the channel is declared only at
/// other instances, by whichever writers declare it.
fn missing_series(
    role: &str,
    series: AgentSeries,
    target: &AssertionTarget,
    agent: &AgentId,
    declared: Option<&[String]>,
) -> String {
    let agent = agent.as_str();
    let supplier = series.supplier();
    match declared {
        None => format!(
            "its {role} reads `{series}` at {target}, and agent '{agent}' has no {supplier}"
        ),
        Some(declared) => format!(
            "its {role} reads `{series}` at {target}, and agent '{agent}' declares that \
             channel only at {}: its {supplier} writes no `{series}`",
            declared.join(LIST_SEPARATOR)
        ),
    }
}

/// `evaluation`, named `name` and written where `origin` says, built and
/// bound in `collector`, or every reason it can't be: each input's
/// problems, or, once every input is bound, the kind's, or each gap it
/// gives a role with no cap. A role it gives no gap takes `defaults`'.
fn plan(
    name: &EvaluationName,
    evaluation: &Evaluation,
    origin: Origin,
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
        match locate_input(target, role, origin.series(role.name), catalogs) {
            Ok(input) => bound.push(input),
            Err(found) => problems.extend(
                found
                    .into_iter()
                    .map(|problem| origin.input_problem(role.name, target, problem)),
            ),
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
    let binding = evaluator.binding();
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
            .map(|input| (input.planned.target.clone(), input.planned.key.clone()))
            .collect(),
    );

    Ok(PlannedEvaluation {
        name: name.clone(),
        kind: kind.name(),
        params: evaluation.params.clone(),
        binding,
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

/// The one series `target` names for `role`, bound from `series` for a
/// suite's entry, or every reason the run can't feed it to the role: it
/// must be one series, of the role's payload type.
fn locate_input(
    target: &AssertionTarget,
    role: &Role,
    series: Option<AgentSeries>,
    catalogs: &[Catalog],
) -> Result<Bound, Vec<TargetProblem>> {
    let (catalog, selected) = find(target, catalogs)?;
    let [(subject, observable)] = selected.as_slice() else {
        let subjects: Vec<&Subject> = selected.iter().map(|(subject, _)| *subject).collect();
        return Err(vec![TargetProblem::InputOnSeveral(spellings(&subjects))]);
    };

    let carries = match observable.value() {
        ObservableValue::Scalar { .. } => Some(SCALAR),
        ObservableValue::Typed { type_id, .. } if type_id == role.payload => None,
        ObservableValue::Typed { type_name, .. } => Some(type_name),
    };
    if let Some(carries) = carries {
        return Err(vec![TargetProblem::WrongPayload {
            carries,
            wants: role.payload_name,
        }]);
    }

    // An emission has no description: its node writes it, at no declared
    // rate, in no declared frame. A keyed instance is expressed in the frame
    // its writer declared it with, a state in the root its writer holds it in.
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
                        .or_else(|| {
                            catalog
                                .state_schemas
                                .of_channel(channel)
                                .map(StateDescription::root)
                        })
                        .cloned(),
                )
            }),
    };
    let convention = expressed_in
        .as_ref()
        .and_then(|frame| catalog.frame_conventions.convention_of(frame));
    let placement = expressed_in.as_ref().and_then(|frame| {
        catalog
            .frame_placements
            .iter()
            .find(|placement| placement.frame() == frame)
            .cloned()
    });
    Ok(Bound {
        held: matches!(subject, Subject::Channel { .. }),
        nondeterministic: observable.determinism() == Determinism::Nondeterministic,
        facts: BoundInput {
            agent: catalog.agent.clone(),
            instance: instance_of(subject).map(str::to_owned),
            expressed_in,
            convention,
            placement,
            rate_hz,
            writer: writer.clone(),
        },
        planned: PlannedInput {
            role: role.name,
            series,
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

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{
        assertion::{extract::standard_extractors, Assertion, EntryName},
        evaluation::standard_evaluators,
        runner::resolve::{
            plan::{resolve, EntryOwner, SeriesSource},
            ResolvedPlan, TargetProblem, Unusable,
        },
    };

    use helios_core::interchange::motion::StampedTwist;
    use helios_core::prelude::FrameId;
    use helios_core::spatial::conventions::Flu;
    use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};
    use helios_core::spatial::transforms::Convention;
    use helios_core::spatial::FrameAwareState;
    use helios_runtime::{
        build_pipeline,
        channels::{self, estimate},
        observe::{
            path::channel_path,
            sink::{ObservationSink, WatchRequest},
        },
        pipeline::Placement,
        port::Provenance,
        AutonomyPipeline, AutonomyRegistry, AutonomyStackConfig, BodyCapabilities, BodyChannel,
        HostDeclaration, PipelineBuilder,
    };

    use std::collections::HashMap;

    const CAR: &str = "car";
    const TRUCK: &str = "truck";
    /// The rate the fixture body declares for its truth.
    const TRUTH_RATE_HZ: f64 = 100.0;

    /// A suite of one `twist` entry on `truth.twist`, its gap in periods.
    const SPEED_SUITE: &str = r#"
        [evaluation]
        gap_multiple = 3.0

        [[evaluate]]
        name = "speed"
        kind = "twist"
        [evaluate.inputs]
        twist = "truth.twist"
    "#;

    /// A suite of one `pose` entry on `estimate.pose`.
    const POSE_SUITE: &str = r#"
        [[evaluate]]
        name = "pose"
        kind = "pose"
        [evaluate.inputs]
        pose = "estimate.pose"
    "#;

    /// `agent`'s pipeline: no nodes, and a body whose truth publishes
    /// `truth.twist` at `frame`.
    fn truth_body(agent: &str, frame: FrameId) -> AutonomyPipeline {
        PipelineBuilder::new(AgentId::new(agent))
            .with_body_capabilities(BodyCapabilities {
                publishes: vec![BodyChannel::new(
                    channels::truth_twist(),
                    "truth",
                    Provenance::Exact,
                )
                .with_instances::<StampedTwist<Flu>>([frame])
                .with_rate(TRUTH_RATE_HZ)],
                ..Default::default()
            })
            .build()
            .expect("a body with no nodes builds")
    }

    /// `agent`'s truth body, publishing its own `base_link`'s twist.
    fn at_base_link(agent: &str) -> AutonomyPipeline {
        truth_body(agent, FrameId::base_link(AgentId::new(agent)))
    }

    /// `suite` applied under `name` to `agents`, as `[[suite]]` writes it.
    fn applied(name: &str, agents: &str, suite: &str) -> Suite {
        Suite {
            application: toml::from_str(&format!(
                "name = \"{name}\"\nfrom = \"suite.toml\"\nagents = {agents}"
            ))
            .expect("a valid application"),
            config: toml::from_str(suite).expect("a valid suite"),
        }
    }

    /// `suites` and `assertions` resolved against `agents`, with `defaults`
    /// the run's.
    fn resolved(
        suites: &[Suite],
        defaults: Option<&EvaluationDefaults>,
        assertions: &[Assertion],
        agents: &[(&str, AutonomyPipeline)],
    ) -> (ObservationCollector, Result<ResolvedPlan, Unusable>) {
        let evaluations = RunEvaluations {
            inline: &[],
            defaults,
            suites,
        };
        let pipelines: Vec<(AgentId, &AutonomyPipeline)> = agents
            .iter()
            .map(|(agent, pipeline)| (AgentId::new(*agent), pipeline))
            .collect();
        let mut collector = ObservationCollector::new(standard_extractors());
        let plan = resolve(
            evaluations,
            assertions,
            &[],
            &standard_evaluators(),
            &pipelines,
            &mut collector,
        );
        (collector, plan)
    }

    fn agent_scoped(agent: &str, application: &str, entry: &str) -> EvaluationName {
        EvaluationName::agent(
            AgentId::new(agent),
            EntryName::parse(application).unwrap(),
            EntryName::parse(entry).unwrap(),
        )
    }

    /// The problems of the one evaluation the run refused.
    fn refused(plan: Result<ResolvedPlan, Unusable>) -> UnusableEvaluation {
        let unusable = plan.expect_err("the evaluation is refused");
        let [evaluation] = unusable.evaluations.as_slice() else {
            panic!("one refused evaluation: {unusable}");
        };
        evaluation.clone()
    }

    /// `CAR`'s pipeline with a mock oracle as its estimate, pinned through
    /// its truth to [`datum`]: the relay forwards its state onto the estimate.
    fn oracle_estimate() -> AutonomyPipeline {
        let agent = AgentId::new(CAR);
        let stack: AutonomyStackConfig = toml::from_str(
            r#"
            [nodes.truth]
            kind = "MockOracle"

            [estimate]
            source = "truth"
            "#,
        )
        .expect("a valid stack");
        let body = BodyCapabilities {
            publishes: vec![
                BodyChannel::new(channels::truth_tf(), "truth", Provenance::Exact)
                    .with_instances::<StampedTransform>([FrameEdge {
                        child: FrameId::base_link(agent.clone()),
                        parent: datum(),
                    }]),
                BodyChannel::new(channels::truth_twist(), "truth", Provenance::Exact)
                    .with_instances::<StampedTwist<Flu>>([FrameId::base_link(agent.clone())]),
            ],
            ..Default::default()
        };
        let host = HostDeclaration {
            tick_rate_hz: TRUTH_RATE_HZ,
            world_frames: HashMap::from([(datum(), Convention::Enu)]),
        };
        build_pipeline(&stack, &AutonomyRegistry::default(), agent, body, host)
            .unwrap_or_else(|errors| panic!("an oracle estimate builds: {errors:?}"))
    }

    fn datum() -> FrameId {
        FrameId::global("datum")
    }

    /// The estimate seam's state is held in `odom`, which its writer gives
    /// before the first state arrives, so a role reading it learns the frame,
    /// its axes and its anchor at setup.
    #[test]
    fn the_estimates_state_is_bound_in_the_root_its_source_holds_it_in() {
        let agent = AgentId::new(CAR);
        let pipeline = oracle_estimate();
        let catalogs = [Catalog::new(&agent, &pipeline)];
        let target = AssertionTarget::parse(channel_path(CAR, &estimate::estimate()))
            .expect("a valid target");

        let bound = locate_input(
            &target,
            &Role::of::<FrameAwareState>("estimate"),
            Some(AgentSeries::EstimateState),
            &catalogs,
        )
        .unwrap_or_else(|problems| panic!("the estimate binds: {problems:?}"));

        let odom = FrameId::odom(agent);
        assert_eq!(bound.facts.expressed_in, Some(odom.clone()));
        assert_eq!(bound.facts.convention, Some(Convention::Enu));
        let placement = bound.facts.placement.expect("the oracle defines odom");
        assert_eq!(placement.frame(), &odom);
        let Placement::Anchored(anchor) = placement.placement() else {
            panic!("odom is pinned through the truth: {placement:?}");
        };
        assert_eq!(anchor.parent, datum());
    }

    #[test]
    fn a_suite_applied_to_every_agent_binds_its_entry_on_each_and_records_both() {
        let (collector, plan) = resolved(
            &[applied("motion", "\"*\"", SPEED_SUITE)],
            None,
            &[],
            &[(CAR, at_base_link(CAR)), (TRUCK, at_base_link(TRUCK))],
        );

        let plan = plan.expect("both agents have truth");
        let names: Vec<&EvaluationName> = plan.evaluations().iter().map(|e| &e.name).collect();
        assert_eq!(
            names,
            [
                &agent_scoped(CAR, "motion", "speed"),
                &agent_scoped(TRUCK, "motion", "speed")
            ]
        );
        let input = &plan.evaluations()[1].inputs[0];
        assert_eq!(input.series, Some(AgentSeries::TruthTwist));
        assert_eq!(input.target.path(), "agent.truck.channel.truth.twist");
        assert_eq!(
            collector.request(),
            WatchRequest::Paths(
                [
                    "agent.car.channel.truth.twist".to_owned(),
                    "agent.truck.channel.truth.twist".to_owned(),
                ]
                .into_iter()
                .collect()
            )
        );
    }

    #[test]
    fn a_listed_agent_the_run_lacks_is_refused_naming_those_it_has() {
        let (_, plan) = resolved(
            &[applied("motion", "[\"truck\"]", SPEED_SUITE)],
            None,
            &[],
            &[(CAR, at_base_link(CAR))],
        );

        let evaluation = refused(plan);
        assert_eq!(evaluation.name, agent_scoped(TRUCK, "motion", "speed"));
        assert_eq!(
            evaluation.to_string(),
            "evaluation 'agent.truck.motion.speed': the run has no agent by that name (it has car)"
        );
    }

    #[test]
    fn an_agent_without_the_series_is_refused_naming_its_supplier() {
        let (_, plan) = resolved(
            &[applied("estimator", "[\"car\"]", POSE_SUITE)],
            None,
            &[],
            &[(CAR, at_base_link(CAR))],
        );

        assert_eq!(
            refused(plan).to_string(),
            "evaluation 'agent.car.estimator.pose': its pose reads `estimate.pose` at \
             agent.car.channel.tf[instance=agent.car.base_link], and agent 'car' has no \
             estimate seam"
        );
    }

    #[test]
    fn a_series_whose_instance_the_agent_doesnt_declare_lists_those_it_does() {
        let antenna = FrameId::agent(AgentId::new(CAR), "antenna");
        let (_, plan) = resolved(
            &[applied("motion", "[\"car\"]", SPEED_SUITE)],
            None,
            &[],
            &[(CAR, truth_body(CAR, antenna))],
        );

        assert_eq!(
            refused(plan).to_string(),
            "evaluation 'agent.car.motion.speed': its twist reads `truth.twist` at \
             agent.car.channel.truth.twist[instance=agent.car.base_link], and agent 'car' \
             declares that channel only at agent.car.antenna: its truth source writes no \
             `truth.twist`"
        );
    }

    #[test]
    fn an_entrys_gap_comes_before_its_suites_and_the_runs_never_applies() {
        let run_default: EvaluationDefaults =
            toml::from_str("gap_multiple = 5.0").expect("valid defaults");
        let planned = |suite: &str| {
            resolved(
                &[applied("motion", "[\"car\"]", suite)],
                Some(&run_default),
                &[],
                &[(CAR, at_base_link(CAR))],
            )
            .1
        };
        let cap_multiple = |suite: &str| {
            planned(suite).expect("the car has truth").evaluations()[0].inputs[0]
                .cap
                .and_then(|cap| cap.multiple())
        };

        let own_gap = format!("{SPEED_SUITE}\n[evaluate.gap.twist]\nmultiple = 2.0\n");
        assert_eq!(cap_multiple(SPEED_SUITE), Some(3.0));
        assert_eq!(cap_multiple(&own_gap), Some(2.0));

        // The run's 5.0 would make the suite's numbers depend on the run.
        let no_suite_default = SPEED_SUITE.replace("[evaluation]\n        gap_multiple = 3.0", "");
        let refusal = refused(planned(&no_suite_default)).to_string();
        assert!(
            refusal.contains("none is given: give [evaluation] gap_multiple in the entry's file"),
            "{refusal}"
        );
    }

    fn max_of(target: &str) -> Assertion {
        toml::from_str(&format!(
            "name = \"fast\"\ntarget = \"{target}\"\nreduce = \"max\"\n\
             condition = {{ less_than = 1.0 }}"
        ))
        .expect("a valid assertion")
    }

    #[test]
    fn an_assertion_reads_a_suite_entrys_metric_on_its_agent() {
        let (_, plan) = resolved(
            &[applied("motion", "\"*\"", SPEED_SUITE)],
            None,
            &[max_of("eval.agent.truck.motion.speed.speed")],
            &[(CAR, at_base_link(CAR)), (TRUCK, at_base_link(TRUCK))],
        );

        assert!(plan.is_ok(), "{:?}", plan.err().map(|u| u.to_string()));
    }

    #[test]
    fn under_every_agent_a_metric_of_an_agent_the_run_lacks_is_refused() {
        let (_, plan) = resolved(
            &[applied("motion", "\"*\"", SPEED_SUITE)],
            None,
            &[max_of("eval.agent.truck.motion.speed.speed")],
            &[(CAR, at_base_link(CAR))],
        );

        let unusable = plan.expect_err("no truck");
        assert_eq!(unusable.entries[0].problems, [TargetProblem::UnknownAgent]);
    }

    #[test]
    fn under_every_agents_evaluation_one_assertion_judges_each_agent_apart() {
        let (_, plan) = resolved(
            &[applied("motion", "\"*\"", SPEED_SUITE)],
            None,
            &[
                max_of("eval.agent.*.motion.speed.speed"),
                max_of("eval.agent.truck.motion.speed.speed"),
            ],
            &[(CAR, at_base_link(CAR)), (TRUCK, at_base_link(TRUCK))],
        );

        let plan = plan.expect("both agents have truth");
        let entries: Vec<(EntryOwner, Option<&str>, &SeriesSource)> = plan
            .entries
            .iter()
            .map(|entry| {
                (
                    entry.owner,
                    entry.agent.as_ref().map(AgentId::as_str),
                    &entry.source,
                )
            })
            .collect();
        // One entry per agent, in the order the evaluations were planned;
        // the concrete target names its agent and carries none.
        assert_eq!(
            entries,
            [
                (
                    EntryOwner::Assertion(0),
                    Some(CAR),
                    &SeriesSource::Derived {
                        evaluation: 0,
                        metric: 0
                    }
                ),
                (
                    EntryOwner::Assertion(0),
                    Some(TRUCK),
                    &SeriesSource::Derived {
                        evaluation: 1,
                        metric: 0
                    }
                ),
                (
                    EntryOwner::Assertion(1),
                    None,
                    &SeriesSource::Derived {
                        evaluation: 1,
                        metric: 0
                    }
                ),
            ]
        );
    }

    #[test]
    fn under_every_agents_evaluation_a_refused_agent_refuses_the_entry_naming_it() {
        let antenna = FrameId::agent(AgentId::new(TRUCK), "antenna");
        let (_, plan) = resolved(
            &[applied("motion", "\"*\"", SPEED_SUITE)],
            None,
            &[max_of("eval.agent.*.motion.speed.speed")],
            &[
                (CAR, at_base_link(CAR)),
                (TRUCK, truth_body(TRUCK, antenna)),
            ],
        );

        let unusable = plan.expect_err("the truck's evaluation is refused");
        assert_eq!(
            unusable.evaluations[0].name,
            agent_scoped(TRUCK, "motion", "speed")
        );
        assert_eq!(
            unusable.entries[0].problems,
            [TargetProblem::EvaluationsUnusable(vec![
                "agent.truck.motion.speed".to_owned()
            ])]
        );
        assert!(
            unusable.to_string().ends_with(
                "'fast' on 'eval.agent.*.motion.speed.speed': it selects every agent's \
                 evaluation, and these can't run: agent.truck.motion.speed"
            ),
            "{unusable}"
        );
    }

    #[test]
    fn under_every_agents_evaluation_a_run_with_none_planned_is_refused() {
        let (_, plan) = resolved(
            &[applied("motion", "\"*\"", SPEED_SUITE)],
            None,
            &[max_of("eval.agent.*.motion.speed.speed")],
            &[],
        );

        let unusable = plan.expect_err("no agent, no evaluation");
        assert_eq!(
            unusable.entries[0].problems,
            [TargetProblem::NoEvaluationSelected]
        );
    }
}
