use serde::{Deserialize, Serialize};
use thiserror::Error;

/// _Shared (pre + post)._ A comparable value: read from a run file and written
/// back into the report.
// `untagged` lets TOML `value = 0` deserialize as `Int(0)` rather than `{ Int = 0 }`.
// Variant order is load-bearing: serde tries each in declaration order, so the most
// specific TOML type (bool) must come before the most permissive (string).
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(untagged)]
pub enum AssertionValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    String(String),
}

impl AssertionValue {
    // The type gate for equality: `equals` / `not_equals` compare `kind()`
    // before comparing values, so `Int(5)` vs `Float(5.0)` is a `TypeMismatch`
    // error, never a silent coercion.
    pub fn kind(&self) -> ValueKind {
        match self {
            AssertionValue::Bool(_) => ValueKind::Bool,
            AssertionValue::Int(_) => ValueKind::Int,
            AssertionValue::Float(_) => ValueKind::Float,
            AssertionValue::String(_) => ValueKind::String,
        }
    }

    /// The value as f64 if it is a number, for comparisons and reducers that
    /// read an int and a float alike. An i64 past 2^53 rounds, which no
    /// threshold today comes near.
    pub fn as_number(&self) -> Option<f64> {
        match self {
            AssertionValue::Int(x) => Some(*x as f64),
            AssertionValue::Float(x) => Some(*x),
            AssertionValue::Bool(_) | AssertionValue::String(_) => None,
        }
    }
}

/// _Shared (pre + per)._ Which variant of [`AssertionValue`] a series holds.
/// Known at setup for every series (an extractor's from its return type, an
/// emission's as a float), so a condition or reducer that can't apply to it
/// is refused before the first tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueKind {
    Bool,
    Int,
    Float,
    String,
}

impl ValueKind {
    /// Whether an ordering, a range or a number reducer reads it.
    pub fn is_number(self) -> bool {
        matches!(self, ValueKind::Int | ValueKind::Float)
    }
}

impl std::fmt::Display for ValueKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ValueKind::Bool => "bool",
            ValueKind::Int => "int",
            ValueKind::Float => "float",
            ValueKind::String => "string",
        })
    }
}

/// _Shared (pre + post)._ The comparison and its operand, parsed from a run
/// file (`condition = { less_than = 25.0 }`) and echoed into the report.
///
/// Each condition carries the operand it needs, so neither "a range without
/// bounds" nor "bounds on `less_than`" can be written. Ordering conditions
/// take an `f64` (a TOML int reads into it, so `{ less_than = 25 }` means
/// 25.0); only equality takes an [`AssertionValue`], since only it compares
/// bools and strings. Operands serde can read but no comparison can use (a
/// NaN bound, a reversed range, a float under `equals`) are refused by
/// [`check_operand`](Self::check_operand) when the run file is validated.
#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Condition {
    Equals(AssertionValue),
    NotEquals(AssertionValue),
    LessThan(f64),
    GreaterThan(f64),
    /// Inclusive bounds, `[low, high]`.
    WithinRange([f64; 2]),
}

impl Condition {
    /// The condition's name as a run file spells it.
    pub fn verb(&self) -> &'static str {
        match self {
            Condition::Equals(_) => "equals",
            Condition::NotEquals(_) => "not_equals",
            Condition::LessThan(_) => "less_than",
            Condition::GreaterThan(_) => "greater_than",
            Condition::WithinRange(_) => "within_range",
        }
    }

    /// Why the operand can never be compared against, if it can't.
    pub fn check_operand(&self) -> Result<(), OperandProblem> {
        match self {
            Condition::Equals(AssertionValue::Float(_))
            | Condition::NotEquals(AssertionValue::Float(_)) => Err(OperandProblem::FloatEquality),
            Condition::Equals(_) | Condition::NotEquals(_) => Ok(()),
            Condition::LessThan(bound) | Condition::GreaterThan(bound) if !bound.is_finite() => {
                Err(OperandProblem::NonFinite)
            }
            Condition::LessThan(_) | Condition::GreaterThan(_) => Ok(()),
            Condition::WithinRange([low, high]) if !(low.is_finite() && high.is_finite()) => {
                Err(OperandProblem::NonFinite)
            }
            Condition::WithinRange([low, high]) if low > high => {
                Err(OperandProblem::ReversedRange {
                    low: *low,
                    high: *high,
                })
            }
            Condition::WithinRange(_) => Ok(()),
        }
    }

    /// Why the condition can never judge a value of kind `judged`, the
    /// series' kind or its reducer's, if it can't: checked at setup, so
    /// what `apply` would fail on every sample stops the run before it
    /// starts. Equality needs an operand of the judged kind and refuses a
    /// float, which is exact float equality; an ordering or a range needs a
    /// number.
    pub fn check_kind(&self, judged: ValueKind) -> Result<(), KindProblem> {
        let condition = self.verb();
        match self {
            Condition::Equals(_) | Condition::NotEquals(_) if judged == ValueKind::Float => {
                Err(KindProblem::FloatEquality { condition })
            }
            Condition::Equals(operand) | Condition::NotEquals(operand)
                if operand.kind() != judged =>
            {
                Err(KindProblem::Mismatch {
                    condition,
                    judged,
                    operand: operand.kind(),
                })
            }
            Condition::Equals(_) | Condition::NotEquals(_) => Ok(()),
            Condition::LessThan(_) | Condition::GreaterThan(_) | Condition::WithinRange(_)
                if !judged.is_number() =>
            {
                Err(KindProblem::NotANumber { condition, judged })
            }
            Condition::LessThan(_) | Condition::GreaterThan(_) | Condition::WithinRange(_) => {
                Ok(())
            }
        }
    }

    /// Whether `actual` meets the condition. `Err` when the comparison can't
    /// be made: values of different kinds under equality, or a value that
    /// isn't a number under an ordering. A NaN reading fails every ordering
    /// and range, since every comparison with NaN is false.
    pub fn apply(&self, actual: &AssertionValue) -> Result<bool, ConditionError> {
        match self {
            Condition::Equals(expected) => same_kind(actual, expected).map(|()| actual == expected),
            Condition::NotEquals(expected) => {
                same_kind(actual, expected).map(|()| actual != expected)
            }
            Condition::LessThan(bound) => self.number(actual).map(|x| x < *bound),
            Condition::GreaterThan(bound) => self.number(actual).map(|x| x > *bound),
            Condition::WithinRange([low, high]) => {
                self.number(actual).map(|x| x >= *low && x <= *high)
            }
        }
    }

    /// `actual` as a number, for an ordering or a range.
    fn number(&self, actual: &AssertionValue) -> Result<f64, ConditionError> {
        actual.as_number().ok_or(ConditionError::NotComparable {
            condition: self.verb(),
            value_kind: actual.kind(),
        })
    }
}

/// The gate equality passes first: an int never equals a float, nor a
/// string a bool.
fn same_kind(actual: &AssertionValue, expected: &AssertionValue) -> Result<(), ConditionError> {
    if actual.kind() == expected.kind() {
        Ok(())
    } else {
        Err(ConditionError::TypeMismatch {
            expected_kind: expected.kind(),
            actual_kind: actual.kind(),
        })
    }
}

/// _Pre-step._ Why a condition's operand can never be compared against.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum OperandProblem {
    #[error("its bound is not a finite number")]
    NonFinite,
    #[error("its range is reversed: low {low} is above high {high}")]
    ReversedRange { low: f64, high: f64 },
    #[error("it compares a float for exact equality; use within_range")]
    FloatEquality,
}

/// _Pre-step._ Why a condition can never judge a series' values, found at
/// setup from the kind they have.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum KindProblem {
    #[error("{condition} compares values of kind {judged} with an operand of kind {operand}")]
    Mismatch {
        condition: &'static str,
        judged: ValueKind,
        operand: ValueKind,
    },
    #[error("{condition} needs a number, not a value of kind {judged}")]
    NotANumber {
        condition: &'static str,
        judged: ValueKind,
    },
    #[error("{condition} on a float compares for exact equality (use within_range)")]
    FloatEquality { condition: &'static str },
}

/// _Per-step._ A comparison that couldn't be carried out, produced during evaluation.
// Two distinct author errors: comparing different types (`TypeMismatch`) vs
// applying a condition a type can't support, e.g. `LessThan` on a bool
// (`NotComparable`). Kept separate so reports can phrase each one usefully.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ConditionError {
    #[error("type mismatch: expected {expected_kind}, got {actual_kind}")]
    TypeMismatch {
        expected_kind: ValueKind,
        actual_kind: ValueKind,
    },
    #[error("condition {condition} not applicable to {value_kind}")]
    NotComparable {
        condition: &'static str,
        value_kind: ValueKind,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Deserialize)]
    struct Wrap {
        condition: Condition,
    }

    fn parse(condition: &str) -> Result<Condition, toml::de::Error> {
        toml::from_str::<Wrap>(&format!("condition = {condition}")).map(|wrap| wrap.condition)
    }

    #[test]
    fn assertion_value_deserializes_bare_scalars() {
        #[derive(Deserialize)]
        struct Wrap {
            v: AssertionValue,
        }
        let cases = [
            ("v = true", AssertionValue::Bool(true)),
            ("v = 0", AssertionValue::Int(0)),
            ("v = 42", AssertionValue::Int(42)),
            ("v = 0.0", AssertionValue::Float(0.0)),
            ("v = 2.5", AssertionValue::Float(2.5)),
            ("v = \"hello\"", AssertionValue::String("hello".into())),
        ];
        for (src, expected) in cases {
            let got: Wrap = toml::from_str(src).unwrap();
            assert_eq!(got.v, expected, "src: {src}");
        }
    }

    #[test]
    fn each_condition_parses_with_its_operand() {
        assert_eq!(
            parse("{ less_than = 25.0 }").unwrap(),
            Condition::LessThan(25.0)
        );
        assert_eq!(
            parse("{ greater_than = 0.0 }").unwrap(),
            Condition::GreaterThan(0.0)
        );
        assert_eq!(
            parse("{ equals = 0 }").unwrap(),
            Condition::Equals(AssertionValue::Int(0))
        );
        assert_eq!(
            parse("{ not_equals = \"idle\" }").unwrap(),
            Condition::NotEquals(AssertionValue::String("idle".into()))
        );
        assert_eq!(
            parse("{ within_range = [0.3, 0.7] }").unwrap(),
            Condition::WithinRange([0.3, 0.7])
        );
    }

    #[test]
    fn an_int_ordering_operand_reads_as_a_float() {
        assert_eq!(
            parse("{ less_than = 25 }").unwrap(),
            Condition::LessThan(25.0)
        );
    }

    #[test]
    fn a_misspelled_or_doubled_or_flat_condition_is_refused_at_parse() {
        for condition in [
            "{ les_than = 1.0 }",
            "{ less_than = 1.0, greater_than = 0.0 }",
            "\"less_than\"",
            "{ within_range = [0.3] }",
        ] {
            assert!(parse(condition).is_err(), "{condition}");
        }
    }

    #[test]
    fn unusable_operands_are_found() {
        assert_eq!(
            Condition::LessThan(f64::NAN).check_operand(),
            Err(OperandProblem::NonFinite)
        );
        assert_eq!(
            Condition::GreaterThan(f64::INFINITY).check_operand(),
            Err(OperandProblem::NonFinite)
        );
        assert_eq!(
            Condition::WithinRange([0.0, f64::NAN]).check_operand(),
            Err(OperandProblem::NonFinite)
        );
        assert_eq!(
            Condition::WithinRange([0.7, 0.3]).check_operand(),
            Err(OperandProblem::ReversedRange {
                low: 0.7,
                high: 0.3
            })
        );
        assert_eq!(
            Condition::Equals(AssertionValue::Float(0.0)).check_operand(),
            Err(OperandProblem::FloatEquality)
        );
        assert_eq!(
            Condition::NotEquals(AssertionValue::Float(1.0)).check_operand(),
            Err(OperandProblem::FloatEquality)
        );
    }

    #[test]
    fn usable_operands_pass() {
        for condition in [
            Condition::LessThan(25.0),
            Condition::WithinRange([0.3, 0.3]),
            Condition::Equals(AssertionValue::Int(0)),
            Condition::NotEquals(AssertionValue::String("idle".into())),
        ] {
            assert_eq!(condition.check_operand(), Ok(()), "{condition:?}");
        }
    }

    #[test]
    fn a_condition_that_fits_the_judged_kind_passes_the_kind_check() {
        for (condition, judged) in [
            (Condition::Equals(AssertionValue::Int(0)), ValueKind::Int),
            (
                Condition::NotEquals(AssertionValue::Bool(true)),
                ValueKind::Bool,
            ),
            (
                Condition::Equals(AssertionValue::String("idle".into())),
                ValueKind::String,
            ),
            (Condition::LessThan(1.0), ValueKind::Float),
            (Condition::GreaterThan(1.0), ValueKind::Int),
            (Condition::WithinRange([0.0, 1.0]), ValueKind::Float),
        ] {
            assert_eq!(condition.check_kind(judged), Ok(()), "{condition:?}");
        }
    }

    #[test]
    fn a_condition_that_can_never_judge_the_kind_is_found() {
        assert_eq!(
            Condition::Equals(AssertionValue::Int(1)).check_kind(ValueKind::Float),
            Err(KindProblem::FloatEquality {
                condition: "equals"
            })
        );
        assert_eq!(
            Condition::NotEquals(AssertionValue::String("x".into())).check_kind(ValueKind::Int),
            Err(KindProblem::Mismatch {
                condition: "not_equals",
                judged: ValueKind::Int,
                operand: ValueKind::String,
            })
        );
        assert_eq!(
            Condition::WithinRange([0.0, 1.0]).check_kind(ValueKind::Bool),
            Err(KindProblem::NotANumber {
                condition: "within_range",
                judged: ValueKind::Bool,
            })
        );
    }

    #[test]
    fn equals_same_kind() {
        let five = Condition::Equals(AssertionValue::Int(5));
        assert_eq!(five.apply(&AssertionValue::Int(5)), Ok(true));
        assert_eq!(five.apply(&AssertionValue::Int(6)), Ok(false));
        assert_eq!(
            Condition::Equals(AssertionValue::String("a".into()))
                .apply(&AssertionValue::String("a".into())),
            Ok(true)
        );
    }

    #[test]
    fn equals_strict_rejects_int_vs_float() {
        let result = Condition::Equals(AssertionValue::Int(5)).apply(&AssertionValue::Float(5.0));
        assert!(matches!(result, Err(ConditionError::TypeMismatch { .. })));
    }

    #[test]
    fn not_equals_returns_negation() {
        let five = Condition::NotEquals(AssertionValue::Int(5));
        assert_eq!(five.apply(&AssertionValue::Int(5)), Ok(false));
        assert_eq!(five.apply(&AssertionValue::Int(6)), Ok(true));
    }

    #[test]
    fn orderings_read_an_int_or_a_float_reading() {
        assert_eq!(
            Condition::LessThan(2.0).apply(&AssertionValue::Float(1.0)),
            Ok(true)
        );
        assert_eq!(
            Condition::GreaterThan(2.5).apply(&AssertionValue::Int(3)),
            Ok(true)
        );
        assert_eq!(
            Condition::LessThan(2.0).apply(&AssertionValue::Int(2)),
            Ok(false)
        );
    }

    #[test]
    fn an_ordering_on_a_non_number_is_not_comparable() {
        let result = Condition::LessThan(1.0).apply(&AssertionValue::Bool(true));
        assert!(matches!(result, Err(ConditionError::NotComparable { .. })));
    }

    #[test]
    fn within_range_is_inclusive() {
        let range = Condition::WithinRange([1.0, 2.0]);
        assert_eq!(range.apply(&AssertionValue::Float(1.0)), Ok(true));
        assert_eq!(range.apply(&AssertionValue::Float(2.0)), Ok(true));
        assert_eq!(range.apply(&AssertionValue::Float(2.5)), Ok(false));
        assert_eq!(
            Condition::WithinRange([1.0, 3.0]).apply(&AssertionValue::Int(2)),
            Ok(true)
        );
    }

    #[test]
    fn a_nan_reading_fails_every_ordering_and_range() {
        let nan = AssertionValue::Float(f64::NAN);
        assert_eq!(Condition::LessThan(25.0).apply(&nan), Ok(false));
        assert_eq!(Condition::GreaterThan(0.0).apply(&nan), Ok(false));
        assert_eq!(Condition::WithinRange([0.0, 1.0]).apply(&nan), Ok(false));
    }

    #[test]
    fn a_condition_serializes_as_it_is_written() {
        #[derive(Serialize)]
        struct Wrap {
            condition: Condition,
        }
        let text = toml::to_string(&Wrap {
            condition: Condition::WithinRange([0.3, 0.7]),
        })
        .unwrap();
        assert_eq!(parse_back(&text), Condition::WithinRange([0.3, 0.7]));
    }

    fn parse_back(text: &str) -> Condition {
        toml::from_str::<Wrap>(text).unwrap().condition
    }
}
