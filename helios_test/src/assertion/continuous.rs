use crate::assertion::{
    condition::{AssertionValue, Condition},
    target::AssertionTarget,
};

use serde::Deserialize;

// A continuous assertion is checked every tick across the run ("continuous"),
// e.g. "speed stays below the limit the whole time".
//
// Why a separate type from `TerminalAssertion` despite the identical shape
// today: the two are evaluated at different times and will diverge. This is the
// type expected to grow — a "must hold for N seconds" duration, a startup grace
// period, a tolerance window are all continuous-only concepts. Keeping it
// separate means those fields land here without polluting the terminal type
// with `Option`s it never uses, and `deny_unknown_fields` keeps protecting both.
// The "which kind am I" discriminant lives at the enum level in `mod.rs`
// (`when = "continuous"` / `when = "at_completion"`), not on the struct.
//
// `#[serde(deny_unknown_fields)]` is the typo-catcher: a misspelled key in a run
// file is a hard parse error rather than a silently-defaulted field. A test
// harness that passes on a typo is worse than none.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContinuousAssertion {
    pub target: AssertionTarget,
    pub condition: Condition,
    pub value: AssertionValue,
    // `low`/`high` are meaningful only when `condition == WithinRange`; they
    // live here rather than as a payload on `Condition` so the condition stays
    // a flat string in TOML and all of an assertion's data sits in one table.
    // The cost is two representable illegal states: `WithinRange` with no
    // bounds, and bounds set alongside a non-range condition. Neither is
    // rejected here — the struct stays pure data. Validation is the
    // intended guard; the evaluator's `InvalidBounds` failure is the backstop.
    pub low: Option<f64>,
    pub high: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_minimal_less_than_assertion() {
        let toml = r#"
            target = "agent.car.estimator.speed"
            condition = "less_than"
            value = 5.0
        "#;
        let parsed: ContinuousAssertion = toml::from_str(toml).unwrap();
        assert_eq!(
            parsed.target,
            AssertionTarget::new("agent.car.estimator.speed")
        );
        assert_eq!(parsed.condition, Condition::LessThan);
        assert_eq!(parsed.value, AssertionValue::Float(5.0));
        // Absent bounds default to None.
        assert_eq!(parsed.low, None);
        assert_eq!(parsed.high, None);
    }

    #[test]
    fn parses_within_range_with_bounds() {
        let toml = r#"
            target = "agent.car.estimator.altitude"
            condition = "within_range"
            value = 0.0
            low = -0.5
            high = 0.5
        "#;
        let parsed: ContinuousAssertion = toml::from_str(toml).unwrap();
        assert_eq!(parsed.condition, Condition::WithinRange);
        assert_eq!(parsed.low, Some(-0.5));
        assert_eq!(parsed.high, Some(0.5));
    }

    #[test]
    fn rejects_unknown_field() {
        // The `deny_unknown_fields` typo-catcher in action.
        let toml = r#"
            target = "agent.car.estimator.speed"
            condition = "less_than"
            valeu = 5.0
        "#;
        assert!(toml::from_str::<ContinuousAssertion>(toml).is_err());
    }
}
