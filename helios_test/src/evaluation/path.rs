//! The spelling of a derived series' key, `eval.inline.<evaluation>.<metric>`:
//! parsed and joined here, as `observe::path` does for recorded keys.

use crate::assertion::EntryName;

use helios_core::kernel::identifier::{is_level, LEVEL_SEPARATOR};

use std::{fmt, sync::Arc};
use thiserror::Error;

/// The root every derived series is spelled under.
const EVAL_ROOT: &str = "eval";

/// The scope of an evaluation declared in the run file itself.
const INLINE_SCOPE: &str = "inline";

/// The scopes a suite's evaluations are spelled under, one agent's or the
/// whole run's. Refused until suites are built.
const SUITE_SCOPES: [&str; 2] = ["agent", "run"];

/// _Shared (pre + per + post)._ The key of one metric an inline evaluation
/// derives: the evaluation's name and the metric's, spelled
/// `eval.inline.<evaluation>.<metric>`.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct DerivedName {
    evaluation: EntryName,
    metric: Arc<str>,
}

impl DerivedName {
    /// The key of `metric` of the evaluation named `evaluation`. A metric's
    /// name is one level, as a kind declares it.
    pub fn new(evaluation: EntryName, metric: &str) -> Self {
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
        let mut parts = path.splitn(4, LEVEL_SEPARATOR);
        if parts.next() != Some(EVAL_ROOT) {
            return Err(DerivedProblem::Root);
        }
        match parts.next() {
            Some(INLINE_SCOPE) => {}
            Some(scope) if SUITE_SCOPES.contains(&scope) => {
                return Err(DerivedProblem::SuiteScope(scope.to_owned()))
            }
            _ => return Err(DerivedProblem::Scope),
        }
        let evaluation = parts
            .next()
            .and_then(|name| EntryName::parse(name).ok())
            .ok_or(DerivedProblem::Evaluation)?;
        let metric = parts
            .next()
            .filter(|metric| is_level(metric))
            .ok_or(DerivedProblem::Metric)?;
        Ok(Self::new(evaluation, metric))
    }

    /// The evaluation that derives it.
    pub fn evaluation(&self) -> &EntryName {
        &self.evaluation
    }

    pub fn metric(&self) -> &str {
        &self.metric
    }
}

impl fmt::Display for DerivedName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sep = LEVEL_SEPARATOR;
        write!(
            f,
            "{EVAL_ROOT}{sep}{INLINE_SCOPE}{sep}{}{sep}{}",
            self.evaluation, self.metric
        )
    }
}

/// _Pre-step._ What is wrong with a path under the derived root.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum DerivedProblem {
    #[error("it doesn't start with '{EVAL_ROOT}'")]
    Root,
    #[error("the word after '{EVAL_ROOT}' is missing or not '{INLINE_SCOPE}'")]
    Scope,
    /// A suite's scope, spelled before suites exist.
    #[error("'{EVAL_ROOT}.{0}' names a suite's evaluation, and suites are not built yet")]
    SuiteScope(String),
    #[error("the evaluation's name is missing or not one level")]
    Evaluation,
    #[error("the metric is missing or not one level")]
    Metric,
}

#[cfg(test)]
mod tests {
    use super::*;

    const SURGE: &str = "eval.inline.twist.surge_error";

    #[test]
    fn a_derived_key_round_trips_through_its_spelling() {
        let name = DerivedName::parse(SURGE).expect("valid derived key");

        assert_eq!(name.evaluation().as_str(), "twist");
        assert_eq!(name.metric(), "surge_error");
        assert_eq!(name.to_string(), SURGE);
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
        ];
        for (path, problem) in cases {
            assert_eq!(DerivedName::parse(path), Err(problem), "{path}");
        }
    }

    #[test]
    fn a_suite_scope_is_refused_as_not_built() {
        let error = DerivedName::parse("eval.agent.car.estimator.position_error")
            .expect_err("suites are not built");

        assert_eq!(error, DerivedProblem::SuiteScope("agent".to_owned()));
        assert!(
            error.to_string().contains("suites are not built"),
            "{error}"
        );
    }
}
