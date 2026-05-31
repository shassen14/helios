pub mod condition;
pub mod continuous;
pub mod evaluator;
pub mod extract;
pub mod target;
pub mod terminal;

pub use condition::{AssertionValue, Condition};
use continuous::ContinuousAssertion;
pub use target::AssertionTarget;
use terminal::TerminalAssertion;

use serde::Deserialize;

// The `when` field is the discriminant: `when = "at_completion"` selects a
// terminal assertion, `when = "continuous"` a continuous one. Internally tagged
// (not a nested table) so a run file reads as one flat assertion entry. The
// per-variant rename maps the Rust variant name to the author-facing string.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(tag = "when", rename_all = "snake_case")]
pub enum Assertion {
    #[serde(rename = "at_completion")]
    Terminal(TerminalAssertion),
    Continuous(ContinuousAssertion),
}

// Three outcomes, not two: `Pending` is distinct from `Failed` because a target
// whose channel hasn't published yet is not a failure — it just has no value to
// judge. A continuous assertion sits `Pending` until first data arrives; a
// terminal one resolved as `Pending` at finalize means "never observed".
// Derives are required by the evaluator's tests — `assert_eq!` against a result
// needs `PartialEq`, and failure output needs `Debug`.
#[derive(Clone, Debug, PartialEq)]
pub enum AssertionResult {
    Passed,
    Failed {
        kind: FailureKind,
        actual: Option<AssertionValue>,
    },
    Pending,
}

// Each variant names the evaluator stage that rejected the assertion, so the
// report can phrase the cause precisely instead of a generic "failed":
// target didn't resolve, no extractor for the payload type, types didn't match,
// the condition returned false, or `WithinRange` lacked usable bounds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FailureKind {
    UnresolvedTarget,
    NoExtractor,
    TypeMismatch,
    ConditionFailed,
    InvalidBounds,
}

impl Assertion {
    pub fn target(&self) -> &AssertionTarget {
        match self {
            Assertion::Terminal(t) => &t.target,
            Assertion::Continuous(c) => &c.target,
        }
    }

    pub fn condition(&self) -> &Condition {
        match self {
            Assertion::Terminal(t) => &t.condition,
            Assertion::Continuous(c) => &c.condition,
        }
    }

    pub fn expected(&self) -> &AssertionValue {
        match self {
            Assertion::Terminal(t) => &t.value,
            Assertion::Continuous(c) => &c.value,
        }
    }

    // `Some` only when both bounds are present; either missing → `None`. The
    // evaluator reads this for `WithinRange` and treats `None` as `InvalidBounds`.
    pub fn bounds(&self) -> Option<(f64, f64)> {
        match self {
            Assertion::Terminal(t) => Some((t.low?, t.high?)),
            Assertion::Continuous(c) => Some((c.low?, c.high?)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_terminal_via_when_tag() {
        let toml = r#"
            when = "at_completion"
            target = "agent.car.estimator.position"
            condition = "equals"
            value = 0.0
        "#;
        let parsed: Assertion = toml::from_str(toml).unwrap();
        assert!(matches!(parsed, Assertion::Terminal(_)));
        assert_eq!(parsed.condition(), &Condition::Equals);
        assert_eq!(parsed.expected(), &AssertionValue::Float(0.0));
        assert_eq!(parsed.bounds(), None);
    }

    #[test]
    fn parses_continuous_via_when_tag() {
        let toml = r#"
            when = "continuous"
            target = "agent.car.estimator.speed"
            condition = "within_range"
            value = 0.0
            low = 1.0
            high = 5.0
        "#;
        let parsed: Assertion = toml::from_str(toml).unwrap();
        assert!(matches!(parsed, Assertion::Continuous(_)));
        assert_eq!(parsed.bounds(), Some((1.0, 5.0)));
    }

    #[test]
    fn target_accessor_reads_both_variants() {
        let terminal = r#"
            when = "at_completion"
            target = "agent.car.a"
            condition = "equals"
            value = 1
        "#;
        let continuous = r#"
            when = "continuous"
            target = "agent.car.b"
            condition = "equals"
            value = 1
        "#;
        let t: Assertion = toml::from_str(terminal).unwrap();
        let c: Assertion = toml::from_str(continuous).unwrap();
        assert_eq!(t.target(), &AssertionTarget::new("agent.car.a"));
        assert_eq!(c.target(), &AssertionTarget::new("agent.car.b"));
    }

    #[test]
    fn rejects_unknown_field_through_tagged_enum() {
        // Confirms `deny_unknown_fields` on the variant struct still bites when
        // reached through the internally-tagged `when` discriminant.
        let toml = r#"
            when = "at_completion"
            target = "agent.car.x"
            condition = "equals"
            valeu = 0.0
        "#;
        assert!(toml::from_str::<Assertion>(toml).is_err());
    }

    #[test]
    fn rejects_unknown_when_value() {
        let toml = r#"
            when = "sometimes"
            target = "agent.car.x"
            condition = "equals"
            value = 0.0
        "#;
        assert!(toml::from_str::<Assertion>(toml).is_err());
    }
}
