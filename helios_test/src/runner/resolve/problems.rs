//! Why the run can't evaluate an assertion, measure or evaluation:
//! [`TargetProblem`] per target, gathered per entry and for the whole run.

use super::evaluations::UnusableEvaluation;
use crate::{
    assertion::{condition::KindProblem, AssertionTarget, EntryName},
    statistics::within_run::ReduceError,
};

use std::fmt::{self, Display};
use thiserror::Error;

/// Between the problems of one assertion. No problem's message holds it.
const PROBLEM_SEPARATOR: &str = " and ";

/// Between the instances a problem lists.
pub(super) const INSTANCE_SEPARATOR: &str = ", ";

/// Between the metrics a problem lists.
const METRIC_SEPARATOR: &str = ", ";

/// Between the unusable evaluations and entries in one message. An entry's
/// own message holds a `,` but never this.
const UNUSABLE_SEPARATOR: &str = "; ";

/// _Pre-step._ An assertion or measure the run can't evaluate, and every
/// reason why.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error("'{name}' on '{target}': {}", join(.problems, PROBLEM_SEPARATOR))]
pub struct UnusableEntry {
    /// Unique across assertions and measures, so it says which.
    pub(in crate::runner) name: EntryName,
    pub(in crate::runner) target: AssertionTarget,
    pub(in crate::runner) problems: Vec<TargetProblem>,
}

/// _Pre-step._ Every evaluation, assertion and measure the run can't
/// evaluate, so the operator sees every one to fix at once.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Unusable {
    pub evaluations: Vec<UnusableEvaluation>,
    pub entries: Vec<UnusableEntry>,
}

impl Unusable {
    pub(super) fn is_empty(&self) -> bool {
        self.evaluations.is_empty() && self.entries.is_empty()
    }
}

impl fmt::Display for Unusable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let evaluations = self.evaluations.iter().map(ToString::to_string);
        let entries = self.entries.iter().map(ToString::to_string);
        let all: Vec<String> = evaluations.chain(entries).collect();
        f.write_str(&all.join(UNUSABLE_SEPARATOR))
    }
}

/// _Pre-step._ One reason the run can't judge an assertion.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum TargetProblem {
    /// The scenario has no agent of that name with a pipeline.
    #[error("no agent by that name has a pipeline")]
    UnknownAgent,
    /// Its agent reports nothing at that node and leaf, or has no channel
    /// by that name.
    #[error("its agent reports nothing there")]
    Unreported,
    /// A node path whose leaf is a channel that node writes. A channel is
    /// recorded under its own name, never its writer's, so the path to
    /// watch it is the one held here.
    #[error("a channel is not a node's leaf: to watch what the node writes there, use '{0}'")]
    ChannelUnderNode(String),
    /// A keyed channel written in several instances, named with no
    /// instance. The path names every one, listed, and one series is wanted.
    #[error(
        "it names several instances of a keyed channel ({}): name one with \
         instance = \"…\", or judge each with instance = \"*\"",
        .0.join(INSTANCE_SEPARATOR)
    )]
    SeveralInstances(Vec<String>),
    /// An instance the channel's writers never declared, with those they
    /// did.
    #[error(
        "its channel declares no instance '{instance}' (it declares {})",
        .declared.join(INSTANCE_SEPARATOR)
    )]
    UndeclaredInstance {
        instance: String,
        declared: Vec<String>,
    },
    /// An instance named on an emission or a channel that isn't keyed.
    #[error("it names an instance, which only a keyed channel has")]
    InstanceOnUnkeyed,
    /// A window anchored on each instance of a keyed channel: an anchor is
    /// the first stamp of one series.
    #[error(
        "it names several series ({}), where a window is anchored on one",
        .0.join(INSTANCE_SEPARATOR)
    )]
    AnchorOnSeveral(Vec<String>),
    /// A channel whose payload type has no extractor, so no value from it
    /// could be compared.
    #[error("it carries {0}, which has no extractor")]
    NoExtractor(&'static str),
    /// A nondeterministic leaf, asserted on without
    /// `allow_nondeterministic = true`.
    #[error("it is nondeterministic (set allow_nondeterministic = true to assert on it)")]
    Nondeterministic,
    /// `time_above` on an emission. It holds each sample until the next,
    /// which is right for a channel's state and wrong for an emission's
    /// events: a NIS doesn't stay at its value between updates.
    #[error("time_above holds each value until the next, which is wrong for an emission's events")]
    TimeAboveOnEmission,
    /// A reducer that can't reduce the kind of value the series holds.
    #[error(transparent)]
    Reduce(ReduceError),
    /// A condition that can't judge the kind of value it is given: the
    /// series', or its reducer's.
    #[error(transparent)]
    Condition(KindProblem),
    /// A series the window is anchored `after` that the run can't read, for
    /// the reason held.
    #[error("its window's anchor '{anchor}': {problem}")]
    Anchor {
        anchor: AssertionTarget,
        problem: Box<TargetProblem>,
    },
    /// A derived series of an evaluation the run can't build, refused for
    /// its own reasons.
    #[error("its evaluation can't run")]
    EvaluationUnusable,
    /// A metric its evaluation doesn't offer, when the offered ones are
    /// known only once the evaluation is bound, too late for load to check.
    #[error("its evaluation offers no such metric (it offers {})", .0.join(METRIC_SEPARATOR))]
    UnofferedMetric(Vec<&'static str>),
    /// An instance named on a derived series, which has none.
    #[error("it names an instance, which a derived series doesn't have")]
    InstanceOnDerived,
    /// An evaluation input naming several series, where a role reads one.
    #[error(
        "it names several series ({}), where an input reads one: name one with instance = \"…\"",
        .0.join(INSTANCE_SEPARATOR)
    )]
    InputOnSeveral(Vec<String>),
    /// An evaluation input whose series carries another payload than its
    /// role reads.
    #[error("it carries {carries}, and the role reads {wants}")]
    WrongPayload {
        carries: &'static str,
        wants: &'static str,
    },
}

/// `items` on one line, `separator` between each.
pub(super) fn join(items: &[impl Display], separator: &str) -> String {
    items
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(separator)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runner::resolve::fixtures::{channel, unusable, EDGES, LABEL};

    #[test]
    fn the_message_names_the_assertion_its_target_and_every_problem() {
        let label = channel(LABEL);
        let problems = vec![
            TargetProblem::Nondeterministic,
            TargetProblem::NoExtractor("String"),
        ];

        assert_eq!(
            unusable(4, label, problems).to_string(),
            "'e4' on 'agent.car.channel.label': it is nondeterministic \
             (set allow_nondeterministic = true to assert on it) and it carries String, \
             which has no extractor"
        );
    }

    #[test]
    fn several_instances_are_listed_in_the_message() {
        assert_eq!(
            unusable(
                1,
                channel(EDGES),
                vec![TargetProblem::SeveralInstances(vec![
                    "base".to_string(),
                    "wheel".to_string()
                ])]
            )
            .to_string(),
            "'e1' on 'agent.car.channel.edges': it names several instances of a keyed \
             channel (base, wheel): name one with instance = \"…\", or judge each with \
             instance = \"*\""
        );
    }
}
