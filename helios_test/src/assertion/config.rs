use crate::{
    assertion::{condition::Condition, name::EntryName, target::AssertionTarget},
    statistics::within_run::Reducer,
};

use serde::Deserialize;

/// _Pre-step._ One `[[assert]]` entry: a series, optionally reduced to one
/// value, and the condition it must meet.
///
/// Without `reduce`, every sample must meet the condition: each is judged as
/// it arrives, a failure latches at its sample's stamp, and `fail_fast` can
/// stop the run on it. With `reduce`, the series' one reduced value is
/// judged, once, at the end (`reduce = "final"` judges the latest sample).
///
/// `deny_unknown_fields` is the typo-catcher: a misspelled key in a run file
/// is a parse error rather than a silently defaulted field. A test harness
/// that passes on a typo is worse than none.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Assertion {
    pub name: EntryName,
    pub target: AssertionTarget,
    #[serde(default)]
    pub reduce: Option<Reducer>,
    pub condition: Condition,
    /// Lets the target be a nondeterministic leaf, one whose value varies
    /// between runs on the same inputs (a tick's duration). Off by default, so
    /// a run file that asserts on one says so where it's read. Unused on a
    /// deterministic leaf.
    #[serde(default)]
    pub allow_nondeterministic: bool,
}

impl Assertion {
    /// Whether every sample is judged, rather than one reduced value.
    pub fn judges_every_sample(&self) -> bool {
        self.reduce.is_none()
    }
}

/// _Pre-step._ One `[[measure]]` entry: a series reduced to one value that
/// the report lists without a bound. A measure never fails a run on its
/// value.
///
/// Its own table rather than an assertion with no condition, so a forgotten
/// `condition` is a parse error instead of a silent pass. `reduce` is
/// required: with no bound to judge every sample against, an unreduced
/// measure would have no value.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Measure {
    pub name: EntryName,
    pub target: AssertionTarget,
    pub reduce: Reducer,
    /// As on [`Assertion`].
    #[serde(default)]
    pub allow_nondeterministic: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::AssertionValue;

    #[test]
    fn an_every_sample_assertion_has_no_reducer() {
        let parsed: Assertion = toml::from_str(
            r#"
            name = "speed_capped"
            target = "agent.car.node.estimator.speed"
            condition = { less_than = 5.0 }
            "#,
        )
        .unwrap();
        assert!(parsed.judges_every_sample());
        assert_eq!(parsed.condition, Condition::LessThan(5.0));
        assert!(!parsed.allow_nondeterministic);
    }

    #[test]
    fn a_reduced_assertion_reads_its_reducer() {
        let parsed: Assertion = toml::from_str(
            r#"
            name = "gps_never_dropped"
            target = "agent.car.node.estimator.aiding.gps.dropped"
            reduce = "count"
            condition = { equals = 0 }
            allow_nondeterministic = true
            "#,
        )
        .unwrap();
        assert_eq!(parsed.reduce, Some(Reducer::Count));
        assert_eq!(parsed.condition, Condition::Equals(AssertionValue::Int(0)));
        assert!(parsed.allow_nondeterministic);
    }

    #[test]
    fn an_assertion_needs_a_name_and_a_condition() {
        let unnamed = r#"
            target = "agent.car.node.estimator.x"
            condition = { less_than = 1.0 }
        "#;
        let unconditioned = r#"
            name = "x"
            target = "agent.car.node.estimator.x"
        "#;
        assert!(toml::from_str::<Assertion>(unnamed).is_err());
        assert!(toml::from_str::<Assertion>(unconditioned).is_err());
    }

    #[test]
    fn the_old_format_is_refused() {
        // `when`, `value`, `low` and `high` are gone.
        let old = r#"
            name = "x"
            when = "continuous"
            target = "agent.car.node.estimator.x"
            condition = { less_than = 1.0 }
        "#;
        assert!(toml::from_str::<Assertion>(old).is_err());
    }

    #[test]
    fn a_target_that_is_not_a_watcher_path_is_refused() {
        let toml = r#"
            name = "x"
            target = "agent.car.node.estimator"
            condition = { less_than = 1.0 }
        "#;
        assert!(toml::from_str::<Assertion>(toml).is_err());
    }

    #[test]
    fn a_measure_reads_its_reducer() {
        let parsed: Measure = toml::from_str(
            r#"
            name = "goal_distance"
            target = "agent.car.node.estimator.goal_distance"
            reduce = "final"
            "#,
        )
        .unwrap();
        assert_eq!(parsed.reduce, Reducer::Final);
    }

    #[test]
    fn a_measure_without_a_reducer_or_with_a_condition_is_refused() {
        let unreduced = r#"
            name = "goal_distance"
            target = "agent.car.node.estimator.goal_distance"
        "#;
        let bounded = r#"
            name = "goal_distance"
            target = "agent.car.node.estimator.goal_distance"
            reduce = "final"
            condition = { less_than = 1.0 }
        "#;
        assert!(toml::from_str::<Measure>(unreduced).is_err());
        assert!(toml::from_str::<Measure>(bounded).is_err());
    }
}
