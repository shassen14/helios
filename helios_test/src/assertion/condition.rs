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
    // The type gate for strict comparison: `apply` compares `kind()` before
    // comparing values, so `Int(5)` vs `Float(5.0)` is a `TypeMismatch` error,
    // never a silent coercion. A mistyped TOML value should fail loud, not pass.
    pub fn kind(&self) -> &'static str {
        match self {
            AssertionValue::Bool(_) => "bool",
            AssertionValue::Int(_) => "int",
            AssertionValue::Float(_) => "float",
            AssertionValue::String(_) => "string",
        }
    }
}

/// _Shared (pre + post)._ The comparison verb: parsed from a run file and
/// echoed into the report entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Condition {
    Equals,
    NotEquals,
    LessThan,
    GreaterThan,
    WithinRange,
}

impl Condition {
    pub fn apply(
        &self,
        actual: &AssertionValue,
        expected: &AssertionValue,
    ) -> Result<bool, ConditionError> {
        match self {
            Condition::Equals => {
                if actual.kind() != expected.kind() {
                    return Err(ConditionError::TypeMismatch {
                        expected_kind: expected.kind(),
                        actual_kind: actual.kind(),
                    });
                }
                Ok(actual == expected)
            }
            Condition::NotEquals => {
                if actual.kind() != expected.kind() {
                    return Err(ConditionError::TypeMismatch {
                        expected_kind: expected.kind(),
                        actual_kind: actual.kind(),
                    });
                }
                Ok(actual != expected)
            }
            Condition::LessThan | Condition::GreaterThan => {
                compare_ordered(*self, actual, expected)
            }
            // `WithinRange` carries bounds on the assertion struct, not the value pair;
            // the evaluator must dispatch to `apply_range` before calling `apply`.
            Condition::WithinRange => Err(ConditionError::NotComparable {
                condition: *self,
                value_kind: actual.kind(),
            }),
        }
    }

    pub fn apply_range(
        actual: &AssertionValue,
        low: f64,
        high: f64,
    ) -> Result<bool, ConditionError> {
        match actual {
            AssertionValue::Float(x) => Ok(*x >= low && *x <= high),
            AssertionValue::Int(x) => {
                let x = *x as f64;
                Ok(x >= low && x <= high)
            }
            other => Err(ConditionError::NotComparable {
                condition: Condition::WithinRange,
                value_kind: other.kind(),
            }),
        }
    }
}

fn compare_ordered(
    condition: Condition,
    actual: &AssertionValue,
    expected: &AssertionValue,
) -> Result<bool, ConditionError> {
    let (a, b) = match (actual, expected) {
        (AssertionValue::Float(a), AssertionValue::Float(b)) => (*a, *b),
        // Same-kind Int pairs promote to f64 only to share one comparison path;
        // mixed Int/Float still falls through to the `TypeMismatch` arm below.
        (AssertionValue::Int(a), AssertionValue::Int(b)) => (*a as f64, *b as f64),
        _ if actual.kind() != expected.kind() => {
            return Err(ConditionError::TypeMismatch {
                expected_kind: expected.kind(),
                actual_kind: actual.kind(),
            });
        }
        _ => {
            return Err(ConditionError::NotComparable {
                condition,
                value_kind: actual.kind(),
            });
        }
    };

    Ok(match condition {
        Condition::LessThan => a < b,
        Condition::GreaterThan => a > b,
        _ => unreachable!("compare_ordered called with non-ordering condition"),
    })
}

/// _Per-step._ A comparison that couldn't be carried out, produced during evaluation.
// Two distinct author errors: comparing different types (`TypeMismatch`) vs
// applying a condition a type can't support, e.g. `LessThan` on a bool
// (`NotComparable`). Kept separate so reports can phrase each one usefully.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ConditionError {
    #[error("type mismatch: expected {expected_kind}, got {actual_kind}")]
    TypeMismatch {
        expected_kind: &'static str,
        actual_kind: &'static str,
    },
    #[error("condition {condition:?} not applicable to {value_kind}")]
    NotComparable {
        condition: Condition,
        value_kind: &'static str,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn equals_same_kind() {
        assert_eq!(
            Condition::Equals.apply(&AssertionValue::Int(5), &AssertionValue::Int(5)),
            Ok(true)
        );
        assert_eq!(
            Condition::Equals.apply(&AssertionValue::Int(5), &AssertionValue::Int(6)),
            Ok(false)
        );
        assert_eq!(
            Condition::Equals.apply(
                &AssertionValue::String("a".into()),
                &AssertionValue::String("a".into())
            ),
            Ok(true)
        );
    }

    #[test]
    fn equals_strict_rejects_int_vs_float() {
        let result = Condition::Equals.apply(&AssertionValue::Int(5), &AssertionValue::Float(5.0));
        assert!(matches!(result, Err(ConditionError::TypeMismatch { .. })));
    }

    #[test]
    fn not_equals_returns_negation() {
        assert_eq!(
            Condition::NotEquals.apply(&AssertionValue::Int(5), &AssertionValue::Int(5)),
            Ok(false)
        );
        assert_eq!(
            Condition::NotEquals.apply(&AssertionValue::Int(5), &AssertionValue::Int(6)),
            Ok(true)
        );
    }

    #[test]
    fn less_than_and_greater_than_on_numbers() {
        assert_eq!(
            Condition::LessThan.apply(&AssertionValue::Float(1.0), &AssertionValue::Float(2.0)),
            Ok(true)
        );
        assert_eq!(
            Condition::GreaterThan.apply(&AssertionValue::Int(3), &AssertionValue::Int(2)),
            Ok(true)
        );
        assert_eq!(
            Condition::LessThan.apply(&AssertionValue::Int(2), &AssertionValue::Int(2)),
            Ok(false)
        );
    }

    #[test]
    fn ordered_rejects_non_numeric() {
        let result =
            Condition::LessThan.apply(&AssertionValue::Bool(true), &AssertionValue::Bool(false));
        assert!(matches!(result, Err(ConditionError::NotComparable { .. })));
    }

    #[test]
    fn ordered_rejects_int_vs_float_strict() {
        let result =
            Condition::LessThan.apply(&AssertionValue::Int(1), &AssertionValue::Float(2.0));
        assert!(matches!(result, Err(ConditionError::TypeMismatch { .. })));
    }

    #[test]
    fn within_range_via_apply_errors() {
        let result =
            Condition::WithinRange.apply(&AssertionValue::Float(1.0), &AssertionValue::Float(2.0));
        assert!(matches!(result, Err(ConditionError::NotComparable { .. })));
    }

    #[test]
    fn apply_range_inclusive_bounds() {
        assert_eq!(
            Condition::apply_range(&AssertionValue::Float(1.0), 1.0, 2.0),
            Ok(true)
        );
        assert_eq!(
            Condition::apply_range(&AssertionValue::Float(2.0), 1.0, 2.0),
            Ok(true)
        );
        assert_eq!(
            Condition::apply_range(&AssertionValue::Float(2.5), 1.0, 2.0),
            Ok(false)
        );
        assert_eq!(
            Condition::apply_range(&AssertionValue::Int(2), 1.0, 3.0),
            Ok(true)
        );
    }

    #[test]
    fn apply_range_rejects_non_numeric() {
        let result = Condition::apply_range(&AssertionValue::Bool(true), 0.0, 1.0);
        assert!(matches!(result, Err(ConditionError::NotComparable { .. })));
    }
}
