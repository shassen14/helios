use crate::assertion::condition::{AssertionValue, ValueKind};

use helios_core::kernel::si_unit::SiUnit;
use helios_runtime::ErasedStamped;

use std::{
    any::{Any, TypeId},
    collections::HashMap,
};

/// _Per-step._ Projects one payload type into a comparable value of type
/// `V`. Total: every value of the type has one, so an extractor never
/// declines.
pub type Extractor<T, V> = fn(&T) -> V;

/// _Per-step._ A type an extractor may return: one variant of
/// [`AssertionValue`], whose kind it declares. The kind comes from the
/// return type, so an extractor can't declare one kind and return another.
pub trait ExtractedValue: 'static {
    const KIND: ValueKind;

    fn into_value(self) -> AssertionValue;
}

impl ExtractedValue for f64 {
    const KIND: ValueKind = ValueKind::Float;

    fn into_value(self) -> AssertionValue {
        AssertionValue::Float(self)
    }
}

impl ExtractedValue for i64 {
    const KIND: ValueKind = ValueKind::Int;

    fn into_value(self) -> AssertionValue {
        AssertionValue::Int(self)
    }
}

impl ExtractedValue for bool {
    const KIND: ValueKind = ValueKind::Bool;

    fn into_value(self) -> AssertionValue {
        AssertionValue::Bool(self)
    }
}

impl ExtractedValue for String {
    const KIND: ValueKind = ValueKind::String;

    fn into_value(self) -> AssertionValue {
        AssertionValue::String(self)
    }
}

/// An [`Extractor`] behind the downcast to its payload type. `None` only when
/// handed a payload of another type.
type ErasedExtractor = Box<dyn Fn(&dyn Any) -> Option<AssertionValue> + Send + Sync>;

/// _Pre-step._ What every value an extractor returns is: its kind, and its
/// unit where the payload type fixes one (a wheel's speed is in m/s; a bare
/// `f64` could be anything).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Extracted {
    pub kind: ValueKind,
    pub si_unit: Option<SiUnit>,
}

/// An erased extractor with what its values are.
struct Registered {
    gives: Extracted,
    extract: ErasedExtractor,
}

/// _Per-step._ The `TypeId`-keyed table of payload extractors, each with the
/// kind and unit of value it gives.
// Bridges typed channel values to the untyped `AssertionValue` space assertions
// compare against TOML. Keyed by the *payload* type, never `Stamped<T>`: the
// table reads the payload out of the stamped value itself.
pub struct ExtractorTable {
    by_type: HashMap<TypeId, Registered>,
}

impl ExtractorTable {
    pub fn new() -> Self {
        Self {
            by_type: HashMap::new(),
        }
    }

    /// Registers `extractor` for payloads of type `T`, its values measured
    /// in `si_unit`: `None` when the payload type doesn't fix a unit, and
    /// always for a bool or a string, which has none (a debug build panics,
    /// since the unit would be dropped unseen).
    // Last-writer-wins: registering twice for the same `T` silently overwrites.
    // Intentional — lets tests install fixture extractors over the standard set
    // without a separate "override" entry point.
    pub fn register<T: 'static, V: ExtractedValue>(
        &mut self,
        extractor: Extractor<T, V>,
        si_unit: Option<SiUnit>,
    ) {
        debug_assert!(
            V::KIND.is_number() || si_unit.is_none(),
            "an extractor of {} values can't have a unit",
            V::KIND
        );
        self.by_type.insert(
            TypeId::of::<T>(),
            Registered {
                gives: Extracted {
                    kind: V::KIND,
                    si_unit,
                },
                extract: Box::new(move |payload| {
                    payload
                        .downcast_ref::<T>()
                        .map(|payload| extractor(payload).into_value())
                }),
            },
        );
    }

    /// What a payload of type `payload` gives, or `None` when it has no
    /// extractor.
    pub fn extracted(&self, payload: TypeId) -> Option<Extracted> {
        self.by_type
            .get(&payload)
            .map(|registered| registered.gives)
    }

    /// The comparable value of `stamped`'s payload.
    pub fn extract(&self, stamped: &dyn ErasedStamped) -> Result<AssertionValue, ExtractError> {
        let extractor =
            self.by_type
                .get(&stamped.payload_type_id())
                .ok_or(ExtractError::NoExtractor {
                    payload: stamped.payload_type_name(),
                })?;

        (extractor.extract)(stamped.payload()).ok_or(ExtractError::PayloadMismatch {
            payload: stamped.payload_type_name(),
        })
    }
}

impl Default for ExtractorTable {
    fn default() -> Self {
        Self::new()
    }
}

/// _Per-step._ Why a payload has no comparable value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ExtractError {
    #[error("no extractor for payload type {payload}")]
    NoExtractor { payload: &'static str },
    /// The value's `payload_type_id` names a type its `payload` isn't: an
    /// `ErasedStamped` that disagrees with itself. The impl for `Stamped<T>`
    /// can't.
    #[error("payload type {payload} disagrees with its own type id")]
    PayloadMismatch { payload: &'static str },
}

/// Extractors for the bare payload types. A bare number fixes no unit.
pub fn standard_extractors() -> ExtractorTable {
    let mut table = ExtractorTable::new();
    table.register::<f64, f64>(|x| *x, None);
    table.register::<i64, i64>(|x| *x, None);
    table.register::<bool, bool>(|x| *x, None);
    table
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::MonotonicTime;
    use helios_runtime::prelude::{Health, Stamped};

    /// A payload as a channel carries it.
    fn stamped<T: Send + Sync + 'static>(value: T) -> Stamped<T> {
        Stamped {
            value,
            timestamp: MonotonicTime(0.0),
            health: Health::Ok,
            producer: 0,
        }
    }

    /// Claims to carry an `f64` but holds an `i64`.
    struct Disagreeing(Stamped<i64>);

    impl ErasedStamped for Disagreeing {
        fn payload(&self) -> &dyn Any {
            &self.0.value
        }

        fn payload_type_id(&self) -> TypeId {
            TypeId::of::<f64>()
        }

        fn payload_type_name(&self) -> &'static str {
            std::any::type_name::<f64>()
        }

        fn timestamp(&self) -> MonotonicTime {
            self.0.timestamp
        }

        fn into_any(self: std::sync::Arc<Self>) -> std::sync::Arc<dyn Any + Send + Sync> {
            self
        }
    }

    #[test]
    fn standard_extractors_handles_f64() {
        assert_eq!(
            standard_extractors().extract(&stamped(2.5_f64)),
            Ok(AssertionValue::Float(2.5))
        );
    }

    #[test]
    fn standard_extractors_handles_i64() {
        assert_eq!(
            standard_extractors().extract(&stamped(42_i64)),
            Ok(AssertionValue::Int(42))
        );
    }

    #[test]
    fn standard_extractors_handles_bool() {
        assert_eq!(
            standard_extractors().extract(&stamped(true)),
            Ok(AssertionValue::Bool(true))
        );
    }

    #[test]
    fn an_unregistered_payload_has_no_extractor() {
        assert_eq!(
            standard_extractors().extract(&stamped(String::from("hello"))),
            Err(ExtractError::NoExtractor {
                payload: std::any::type_name::<String>()
            })
        );
    }

    #[test]
    fn a_payload_unlike_its_type_id_is_a_mismatch_not_a_missing_extractor() {
        let disagreeing = Disagreeing(stamped(7_i64));

        assert_eq!(
            standard_extractors().extract(&disagreeing),
            Err(ExtractError::PayloadMismatch {
                payload: std::any::type_name::<f64>()
            })
        );
    }

    #[test]
    fn each_payload_type_gives_the_kind_its_extractor_returns_in_its_unit() {
        let mut table = standard_extractors();
        table.register::<u32, i64>(|x| i64::from(*x), Some(SiUnit::Dimensionless));
        let kind = |payload| table.extracted(payload).map(|gives| gives.kind);

        assert_eq!(kind(TypeId::of::<f64>()), Some(ValueKind::Float));
        assert_eq!(kind(TypeId::of::<bool>()), Some(ValueKind::Bool));
        assert_eq!(kind(TypeId::of::<u32>()), Some(ValueKind::Int));
        assert_eq!(kind(TypeId::of::<String>()), None);

        let unit = |payload| table.extracted(payload).and_then(|gives| gives.si_unit);
        assert_eq!(unit(TypeId::of::<f64>()), None);
        assert_eq!(unit(TypeId::of::<u32>()), Some(SiUnit::Dimensionless));
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "an extractor of bool values can't have a unit")]
    fn a_bool_extractor_with_a_unit_is_refused() {
        standard_extractors().register::<u32, bool>(|x| *x > 0, Some(SiUnit::Meters));
    }

    #[test]
    fn register_overrides_previous_entry() {
        let mut table = standard_extractors();
        table.register::<f64, f64>(|_| 99.0, None);

        assert_eq!(
            table.extract(&stamped(1.0_f64)),
            Ok(AssertionValue::Float(99.0))
        );
    }

    #[test]
    fn default_is_empty() {
        assert!(matches!(
            ExtractorTable::default().extract(&stamped(1.0_f64)),
            Err(ExtractError::NoExtractor { .. })
        ));
    }
}
