//! The spelling of a derived series' key: `eval.inline.<evaluation>.<metric>`
//! for an inline evaluation, `eval.agent.<agent>.<application>.<entry>.<metric>`
//! for a suite's entry bound to one agent. Parsed and joined here, as
//! `observe::path` does for recorded keys.

use crate::assertion::EntryName;

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
        let sep = LEVEL_SEPARATOR;
        match self {
            EvaluationName::Inline(entry) => write!(f, "{INLINE_SCOPE}{sep}{entry}"),
            EvaluationName::Agent(applied) => write!(
                f,
                "{AGENT_SCOPE}{sep}{}{sep}{}{sep}{}",
                applied.agent.as_str(),
                applied.application,
                applied.entry
            ),
        }
    }
}

/// _Shared (pre + per + post)._ A suite's entry bound to one agent: the
/// agent, the name the run file applied the suite under, and the entry's.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct AgentEntry {
    pub agent: AgentId,
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

    /// Whether `path` is spelled under the derived root, so it is a derived
    /// key or no key at all, never a recorded one.
    pub fn is_derived(path: &str) -> bool {
        path.split(LEVEL_SEPARATOR).next() == Some(EVAL_ROOT)
    }

    /// The key `path` spells, or what is wrong with it.
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
                (EvaluationName::Inline(entry), parts.next())
            }
            Some(AGENT_SCOPE) => {
                let mut parts = rest.splitn(4, LEVEL_SEPARATOR);
                let agent = parts
                    .next()
                    .filter(|agent| is_level(agent))
                    .map(AgentId::new)
                    .ok_or(DerivedProblem::Agent)?;
                let application = entry_name(parts.next(), DerivedProblem::Application)?;
                let entry = entry_name(parts.next(), DerivedProblem::Evaluation)?;
                (
                    EvaluationName::agent(agent, application, entry),
                    parts.next(),
                )
            }
            Some(RUN_SCOPE) => return Err(DerivedProblem::RunScope),
            _ => return Err(DerivedProblem::Scope),
        };
        let metric = metric
            .filter(|metric| is_level(metric))
            .ok_or(DerivedProblem::Metric)?;
        Ok(Self::new(evaluation, metric))
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

/// The entry name `part` spells, or `problem` when it is missing or not one.
fn entry_name(part: Option<&str>, problem: DerivedProblem) -> Result<EntryName, DerivedProblem> {
    part.and_then(|name| EntryName::parse(name).ok())
        .ok_or(problem)
}

/// _Pre-step._ What is wrong with a path under the derived root.
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
    #[error("the agent's name is missing or not one level")]
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

    #[test]
    fn a_derived_key_round_trips_through_its_spelling() {
        let name = DerivedName::parse(SURGE).expect("valid derived key");

        assert_eq!(
            name.evaluation(),
            &EvaluationName::Inline(EntryName::parse("twist").unwrap())
        );
        assert_eq!(name.metric(), "surge_error");
        assert_eq!(name.to_string(), SURGE);
    }

    #[test]
    fn an_agent_scoped_key_round_trips_through_its_spelling() {
        let name = DerivedName::parse(AGENT_SURGE).expect("valid derived key");

        assert_eq!(
            name.evaluation(),
            &EvaluationName::agent(
                AgentId::new("car"),
                EntryName::parse("tracking").unwrap(),
                EntryName::parse("twist").unwrap(),
            )
        );
        assert_eq!(name.metric(), "surge");
        assert_eq!(name.to_string(), AGENT_SURGE);
        assert_eq!(name.evaluation().to_string(), "agent.car.tracking.twist");
    }

    #[test]
    fn only_the_eval_root_is_derived() {
        assert!(DerivedName::is_derived(SURGE));
        assert!(DerivedName::is_derived("eval"));
        assert!(!DerivedName::is_derived("agent.car.channel.eval"));
        assert!(!DerivedName::is_derived("evaluation.inline.a.b"));
    }

    #[test]
    fn each_malformed_part_is_named() {
        let cases = [
            ("eval", DerivedProblem::Scope),
            ("eval.outline.twist.surge", DerivedProblem::Scope),
            ("eval.inline", DerivedProblem::Evaluation),
            ("eval.inline.two-words.surge", DerivedProblem::Evaluation),
            ("eval.inline.twist", DerivedProblem::Metric),
            ("eval.inline.twist.surge.x", DerivedProblem::Metric),
            ("eval.agent", DerivedProblem::Agent),
            (
                "eval.agent.my-car.tracking.twist.surge",
                DerivedProblem::Agent,
            ),
            ("eval.agent.car", DerivedProblem::Application),
            ("eval.agent.car.tracking", DerivedProblem::Evaluation),
            ("eval.agent.car.tracking.twist", DerivedProblem::Metric),
            (
                "eval.agent.car.tracking.twist.surge.x",
                DerivedProblem::Metric,
            ),
        ];
        for (path, problem) in cases {
            assert_eq!(DerivedName::parse(path), Err(problem), "{path}");
        }
    }

    #[test]
    fn the_run_scope_is_refused_as_not_built() {
        let error = DerivedName::parse("eval.run.fleet.spacing").expect_err("not built");

        assert_eq!(error, DerivedProblem::RunScope);
        assert!(error.to_string().contains("not built yet"), "{error}");
    }
}
