use crate::assertion::{
    condition::{AssertionValue, Condition},
    target::AssertionTarget,
};

use serde::Deserialize;

/// _Pre-step._ A run-file assertion checked once, at the end of the run.
// A terminal assertion is checked once, at the end of the run ("at_completion").
//
// Why a separate type from `ContinuousAssertion` despite the identical shape:
// the two are evaluated at different times and will diverge. A continuous
// assertion will grow fields a terminal one never wants — a "must hold for N
// seconds" duration, a startup grace period, a tolerance window. Sharing one
// struct would force every continuous-only field to become a meaningless
// `Option` on the terminal type, and `deny_unknown_fields` could no longer tell
// a typo from a field that's simply irrelevant to this variant. Two types cost
// a little duplication now and buy clean divergence later. The "which kind am
// I" discriminant lives at the enum level in `mod.rs` (`when = "at_completion"`
// / `when = "continuous"`), so neither struct carries a kind tag.
//
// `#[serde(deny_unknown_fields)]` is the typo-catcher. Without it, a run file
// with `valeu = 0.0` (misspelled) deserializes silently with `value` defaulting
// and the test asserts against garbage. A test harness that silently passes is
// worse than none, so we reject unknown keys at load time.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerminalAssertion {
    pub target: AssertionTarget,
    pub condition: Condition,
    pub value: AssertionValue,
    // `low`/`high` are meaningful only when `condition == WithinRange`; they
    // live here rather than as a payload on `Condition` so the condition stays
    // a flat string in TOML and all of an assertion's data sits in one table.
    // The cost is two representable illegal states: `WithinRange` with no
    // bounds, and bounds set alongside a non-range condition. Neither is
    // rejected here — the structs stay pure data. The `validate` pass is the
    // intended guard; the evaluator's `InvalidBounds` failure is the backstop.
    pub low: Option<f64>,
    pub high: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_minimal_equals_assertion() {
        let toml = r#"
            target = "agent.car.estimator.position"
            condition = "equals"
            value = 0.0
        "#;
        let parsed: TerminalAssertion = toml::from_str(toml).unwrap();
        assert_eq!(
            parsed.target,
            AssertionTarget::new("agent.car.estimator.position")
        );
        assert_eq!(parsed.condition, Condition::Equals);
        assert_eq!(parsed.value, AssertionValue::Float(0.0));
        // Absent bounds default to None — they're optional for good reason.
        assert_eq!(parsed.low, None);
        assert_eq!(parsed.high, None);
    }

    #[test]
    fn parses_within_range_with_bounds() {
        let toml = r#"
            target = "agent.car.estimator.speed"
            condition = "within_range"
            value = 0.0
            low = 1.0
            high = 5.0
        "#;
        let parsed: TerminalAssertion = toml::from_str(toml).unwrap();
        assert_eq!(parsed.condition, Condition::WithinRange);
        assert_eq!(parsed.low, Some(1.0));
        assert_eq!(parsed.high, Some(5.0));
    }

    #[test]
    fn rejects_unknown_field() {
        // The `deny_unknown_fields` typo-catcher: a misspelled key is a hard
        // parse error, not a silent default.
        let toml = r#"
            target = "agent.car.estimator.position"
            condition = "equals"
            valeu = 0.0
        "#;
        assert!(toml::from_str::<TerminalAssertion>(toml).is_err());
    }
}
