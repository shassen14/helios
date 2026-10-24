//! The spelling of a derived series' key: `eval.inline.<evaluation>.<metric>`
//! for an inline evaluation, `eval.agent.<agent>.<application>.<entry>.<metric>`
//! for a suite's entry bound to one agent. Joined here from a planned
//! evaluation's name, as `observe::path` does for recorded keys; a run file's
//! target, which may select every agent with `*`, is parsed here too.

use crate::assertion::{target::WILDCARD, EntryName};

use helios_core::kernel::identifier::{is_level, LEVEL_SEPARATOR};
use helios_core::prelude::AgentId;

use std::{fmt, sync::Arc};
use thiserror::Error;

/// The root every derived series is spelled under.
const EVAL_ROOT: &str = "eval";

/// The scope of an evaluation declared in the run file itself.
const INLINE_SCOPE: &str = "inline";

/// The scope of a suite's evaluation bound to one agent.
const AGENT_SCOPE: &str = "agent";

/// The scope of an evaluation over the whole run. Refused until one is
/// built.
const RUN_SCOPE: &str = "run";

/// _Shared (pre + per + post)._ Which evaluation a derived series is a metric
/// of: an inline entry, or a suite's entry bound to one agent under the name
/// the run file applied the suite with.
/// Boxed for a suite's entry, so a name of either kind, and every target
/// holding one, stays small.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum EvaluationName {
    Inline(EntryName),
    Agent(Box<AgentEntry>),
}

impl EvaluationName {
    /// The name of the suite's `entry`, applied under `application`, bound
    /// to `agent`.
    pub fn agent(agent: AgentId, application: EntryName, entry: EntryName) -> Self {
        EvaluationName::Agent(Box::new(AgentEntry {
            agent,
            application,
            entry,
        }))
    }

    /// The agent and the application, for a suite's entry.
    pub fn applied(&self) -> Option<(&AgentId, &EntryName)> {
        match self {
            EvaluationName::Inline(_) => None,
            EvaluationName::Agent(applied) => Some((&applied.agent, &applied.application)),
        }
    }

    /// Its scope and the levels under it, as its derived keys spell them
    /// after the root: `inline.<entry>` or
    /// `agent.<agent>.<application>.<entry>`.
    fn write_scoped(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EvaluationName::Inline(entry) => write_inline(f, entry),
            EvaluationName::Agent(applied) => write_applied(f, applied.agent.as_str(), applied),
        }
    }
}

/// _Shared (pre + per + post)._ A suite's entry on an agent: the agent (`A`,
/// one agent for a planned evaluation, or a target's [`AgentSelector`]), the
/// name the run file applied the suite under, and the entry's.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct AgentEntry<A = AgentId> {
    pub agent: A,
    pub application: EntryName,
    pub entry: EntryName,
}

/// An inline entry's name alone, as the run file gives it; a suite's entry
/// as `agent.<agent>.<application>.<entry>`, what its derived keys spell
/// after the root. The one spelling reports and refusals name it by, so a
/// suite's entry on two agents is never two evaluations of one name.
impl fmt::Display for EvaluationName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EvaluationName::Inline(entry) => write!(f, "{entry}"),
            EvaluationName::Agent(_) => self.write_scoped(f),
        }
    }
}

/// _Shared (pre + per + post)._ The key of one metric an evaluation derives:
/// the evaluation's name and the metric's.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct DerivedName {
    evaluation: EvaluationName,
    metric: Arc<str>,
}

impl DerivedName {
    /// The key of `metric` of `evaluation`. A metric's name is one level, as
    /// a kind declares it.
    pub fn new(evaluation: EvaluationName, metric: &str) -> Self {
        debug_assert!(is_level(metric), "a metric name is one level: '{metric}'");
        Self {
            evaluation,
            metric: metric.into(),
        }
    }

    /// The evaluation that derives it.
    pub fn evaluation(&self) -> &EvaluationName {
        &self.evaluation
    }

    pub fn metric(&self) -> &str {
        &self.metric
    }
}

impl fmt::Display for DerivedName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sep = LEVEL_SEPARATOR;
        write!(f, "{EVAL_ROOT}{sep}")?;
        self.evaluation.write_scoped(f)?;
        write!(f, "{sep}{}", self.metric)
    }
}

/// _Shared (pre + per + post)._ A derived series as a run file's target
/// names it: the evaluation, which for a suite's entry may select every
/// agent with `*`, and the metric. Never a key: setup matches it against the
/// planned evaluations' names, each naming one agent.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct DerivedTarget {
    evaluation: EvaluationSelector,
    metric: Arc<str>,
}

impl DerivedTarget {
    /// Whether `path` is spelled under the derived root, so it is a derived
    /// target or no target at all, never a recorded one.
    pub fn is_derived(path: &str) -> bool {
        path.split(LEVEL_SEPARATOR).next() == Some(EVAL_ROOT)
    }

    /// The target `path` spells, or what is wrong with it.
    pub fn parse(path: &str) -> Result<Self, DerivedProblem> {
        let mut parts = path.splitn(3, LEVEL_SEPARATOR);
        if parts.next() != Some(EVAL_ROOT) {
            return Err(DerivedProblem::Root);
        }
        let scope = parts.next();
        let rest = parts.next().unwrap_or_default();
        let (evaluation, metric) = match scope {
            Some(INLINE_SCOPE) => {
                let mut parts = rest.splitn(2, LEVEL_SEPARATOR);
                let entry = entry_name(parts.next(), DerivedProblem::Evaluation)?;
                (EvaluationSelector::Inline(entry), parts.next())
            }
            Some(AGENT_SCOPE) => {
                let mut parts = rest.splitn(4, LEVEL_SEPARATOR);
                let agent = parts
                    .next()
                    .and_then(AgentSelector::parse)
                    .ok_or(DerivedProblem::Agent)?;
                let application = entry_name(parts.next(), DerivedProblem::Application)?;
                let entry = entry_name(parts.next(), DerivedProblem::Evaluation)?;
                (
                    EvaluationSelector::Agent(Box::new(AgentEntry {
                        agent,
                        application,
                        entry,
                    })),
                    parts.next(),
                )
            }
            Some(RUN_SCOPE) => return Err(DerivedProblem::RunScope),
            _ => return Err(DerivedProblem::Scope),
        };
        let metric = metric
            .filter(|metric| is_level(metric))
            .ok_or(DerivedProblem::Metric)?;
        Ok(Self {
            evaluation,
            metric: metric.into(),
        })
    }

    /// The evaluation, or evaluations, whose metric it names.
    pub fn evaluation(&self) -> &EvaluationSelector {
        &self.evaluation
    }

    pub fn metric(&self) -> &str {
        &self.metric
    }

    /// Whether it selects every agent's evaluation of one suite entry.
    pub fn selects_each_agent(&self) -> bool {
        matches!(&self.evaluation, EvaluationSelector::Agent(applied) if applied.agent == AgentSelector::Each)
    }

    /// Whether `evaluation` is one it selects.
    pub fn selects(&self, evaluation: &EvaluationName) -> bool {
        match (&self.evaluation, evaluation) {
            (EvaluationSelector::Inline(selected), EvaluationName::Inline(entry)) => {
                selected == entry
            }
            (EvaluationSelector::Agent(selected), EvaluationName::Agent(applied)) => {
                selected.application == applied.application
                    && selected.entry == applied.entry
                    && match &selected.agent {
                        AgentSelector::One(agent) => *agent == applied.agent,
                        AgentSelector::Each => true,
                    }
            }
            _ => false,
        }
    }
}

/// As it was spelled: `*` where it selects every agent.
impl fmt::Display for DerivedTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sep = LEVEL_SEPARATOR;
        write!(f, "{EVAL_ROOT}{sep}")?;
        match &self.evaluation {
            EvaluationSelector::Inline(entry) => write_inline(f, entry)?,
            EvaluationSelector::Agent(applied) => {
                write_applied(f, &applied.agent.to_string(), applied)?
            }
        }
        write!(f, "{sep}{}", self.metric)
    }
}

/// _Shared (pre + per)._ Which evaluation a derived target names: an inline
/// entry, or a suite's entry on one agent or on each.
/// Boxed for a suite's entry, as [`EvaluationName`] is.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum EvaluationSelector {
    Inline(EntryName),
    Agent(Box<AgentEntry<AgentSelector>>),
}

/// _Shared (pre + per)._ Which agents a derived target names a suite's entry
/// on: one, by name, or each (`*`) the suite's application bound, every one
/// its own entry in the report.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum AgentSelector {
    One(AgentId),
    Each,
}

impl AgentSelector {
    /// The selector `part` spells: `*`, or one agent's name, one level.
    fn parse(part: &str) -> Option<Self> {
        if part == WILDCARD {
            Some(AgentSelector::Each)
        } else {
            is_level(part).then(|| AgentSelector::One(AgentId::new(part)))
        }
    }
}

impl fmt::Display for AgentSelector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AgentSelector::One(agent) => f.write_str(agent.as_str()),
            AgentSelector::Each => f.write_str(WILDCARD),
        }
    }
}

/// An inline entry's scope and name: `inline.<entry>`.
fn write_inline(f: &mut fmt::Formatter<'_>, entry: &EntryName) -> fmt::Result {
    write!(f, "{INLINE_SCOPE}{LEVEL_SEPARATOR}{entry}")
}

/// A suite entry's scope and levels, its agent spelled `agent`:
/// `agent.<agent>.<application>.<entry>`.
fn write_applied<A>(
    f: &mut fmt::Formatter<'_>,
    agent: &str,
    applied: &AgentEntry<A>,
) -> fmt::Result {
    let sep = LEVEL_SEPARATOR;
    write!(
        f,
        "{AGENT_SCOPE}{sep}{agent}{sep}{}{sep}{}",
        applied.application, applied.entry
    )
}

/// The entry name `part` spells, or `problem` when it is missing or not one.
fn entry_name(part: Option<&str>, problem: DerivedProblem) -> Result<EntryName, DerivedProblem> {
    part.and_then(|name| EntryName::parse(name).ok())
        .ok_or(problem)
}

/// _Pre-step._ What is wrong with a target under the derived root.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum DerivedProblem {
    #[error("it doesn't start with '{EVAL_ROOT}'")]
    Root,
    #[error("the word after '{EVAL_ROOT}' is missing or not '{INLINE_SCOPE}' or '{AGENT_SCOPE}'")]
    Scope,
    /// The whole run's scope, spelled before any evaluation over a whole run
    /// exists.
    #[error(
        "'{EVAL_ROOT}.{RUN_SCOPE}' names an evaluation over the whole run, which is not built yet"
    )]
    RunScope,
    #[error("the agent's name is missing, or neither one level nor '{WILDCARD}'")]
    Agent,
    #[error("the suite's name, as [[suite]] applies it, is missing or not one level")]
    Application,
    #[error("the evaluation's name is missing or not one level")]
    Evaluation,
    #[error("the metric is missing or not one level")]
    Metric,
}

#[cfg(test)]
mod tests {
    use super::*;

    const SURGE: &str = "eval.inline.twist.surge_error";
    const AGENT_SURGE: &str = "eval.agent.car.tracking.twist.surge";
    const EACH_SURGE: &str = "eval.agent.*.tracking.twist.surge";

    fn name(text: &str) -> EntryName {
        EntryName::parse(text).unwrap()
    }

    fn agent_scoped(agent: &str) -> EvaluationName {
        EvaluationName::agent(AgentId::new(agent), name("tracking"), name("twist"))
    }

    #[test]
    fn a_derived_target_round_trips_through_its_spelling() {
        let target = DerivedTarget::parse(SURGE).expect("valid derived target");

        assert_eq!(
            target.evaluation(),
            &EvaluationSelector::Inline(name("twist"))
        );
        assert_eq!(target.metric(), "surge_error");
        assert_eq!(target.to_string(), SURGE);
    }

    #[test]
    fn an_agent_scoped_target_round_trips_through_its_spelling() {
        let target = DerivedTarget::parse(AGENT_SURGE).expect("valid derived target");

        assert_eq!(
            target.evaluation(),
            &EvaluationSelector::Agent(Box::new(AgentEntry {
                agent: AgentSelector::One(AgentId::new("car")),
                application: name("tracking"),
                entry: name("twist"),
            }))
        );
        assert_eq!(target.metric(), "surge");
        assert_eq!(target.to_string(), AGENT_SURGE);
        assert!(!target.selects_each_agent());
    }

    #[test]
    fn a_wildcard_agent_round_trips_and_selects_each_agent() {
        let target = DerivedTarget::parse(EACH_SURGE).expect("valid derived target");

        assert!(target.selects_each_agent());
        assert_eq!(target.to_string(), EACH_SURGE);
    }

    #[test]
    fn a_key_is_spelled_as_the_target_naming_it() {
        let key = DerivedName::new(agent_scoped("car"), "surge");

        assert_eq!(key.to_string(), AGENT_SURGE);
        assert_eq!(key.evaluation().to_string(), "agent.car.tracking.twist");
        assert_eq!(
            DerivedName::new(EvaluationName::Inline(name("twist")), "surge_error").to_string(),
            SURGE
        );
    }

    #[test]
    fn a_target_selects_its_own_evaluation_and_a_wildcard_every_agents() {
        let one = DerivedTarget::parse(AGENT_SURGE).unwrap();
        let each = DerivedTarget::parse(EACH_SURGE).unwrap();
        let inline = DerivedTarget::parse(SURGE).unwrap();
        let other_entry =
            EvaluationName::agent(AgentId::new("car"), name("tracking"), name("pose"));
        let other_application =
            EvaluationName::agent(AgentId::new("car"), name("motion"), name("twist"));

        assert!(one.selects(&agent_scoped("car")));
        assert!(!one.selects(&agent_scoped("truck")));
        assert!(each.selects(&agent_scoped("car")) && each.selects(&agent_scoped("truck")));
        assert!(!each.selects(&other_entry) && !each.selects(&other_application));
        assert!(!each.selects(&EvaluationName::Inline(name("twist"))));
        assert!(inline.selects(&EvaluationName::Inline(name("twist"))));
        assert!(!inline.selects(&agent_scoped("car")));
    }

    #[test]
    fn only_the_eval_root_is_derived() {
        assert!(DerivedTarget::is_derived(SURGE));
        assert!(DerivedTarget::is_derived("eval"));
        assert!(!DerivedTarget::is_derived("agent.car.channel.eval"));
        assert!(!DerivedTarget::is_derived("evaluation.inline.a.b"));
    }

    #[test]
    fn each_malformed_part_is_named() {
        let cases = [
            ("eval", DerivedProblem::Scope),
            ("eval.outline.twist.surge", DerivedProblem::Scope),
            ("eval.inline", DerivedProblem::Evaluation),
            ("eval.inline.two-words.surge", DerivedProblem::Evaluation),
            ("eval.inline.*.surge", DerivedProblem::Evaluation),
            ("eval.inline.twist", DerivedProblem::Metric),
            ("eval.inline.twist.surge.x", DerivedProblem::Metric),
            ("eval.agent", DerivedProblem::Agent),
            (
                "eval.agent.my-car.tracking.twist.surge",
                DerivedProblem::Agent,
            ),
            (
                "eval.agent.car*.tracking.twist.surge",
                DerivedProblem::Agent,
            ),
            ("eval.agent.car", DerivedProblem::Application),
            ("eval.agent.*.*.twist.surge", DerivedProblem::Application),
            ("eval.agent.car.tracking", DerivedProblem::Evaluation),
            ("eval.agent.car.tracking.twist", DerivedProblem::Metric),
            ("eval.agent.*.tracking.twist.*", DerivedProblem::Metric),
            (
                "eval.agent.car.tracking.twist.surge.x",
                DerivedProblem::Metric,
            ),
        ];
        for (path, problem) in cases {
            assert_eq!(DerivedTarget::parse(path), Err(problem), "{path}");
        }
    }

    #[test]
    fn the_run_scope_is_refused_as_not_built() {
        let error = DerivedTarget::parse("eval.run.fleet.spacing").expect_err("not built");

        assert_eq!(error, DerivedProblem::RunScope);
        assert!(error.to_string().contains("not built yet"), "{error}");
    }
}
