use crate::assertion::condition::AssertionValue;

use helios_runtime::ErasedStamped;

use std::{
    any::{Any, TypeId},
    collections::HashMap,
};

/// _Per-step._ Projects one payload type into an `AssertionValue`. Total:
/// every value of the type has one, so an extractor never declines.
pub type Extractor<T> = fn(&T) -> AssertionValue;

/// An [`Extractor`] behind the downcast to its payload type. `None` only when
/// handed a payload of another type.
type ErasedExtractor = Box<dyn Fn(&dyn Any) -> Option<AssertionValue> + Send + Sync>;

/// _Per-step._ The `TypeId`-keyed table of payload extractors.
// Bridges typed channel values to the untyped `AssertionValue` space assertions
// compare against TOML. Keyed by the *payload* type, never `Stamped<T>`: the
// table reads the payload out of the stamped value itself.
pub struct ExtractorTable {
    by_type: HashMap<TypeId, ErasedExtractor>,
}

impl ExtractorTable {
    pub fn new() -> Self {
        Self {
            by_type: HashMap::new(),
        }
    }

    // Last-writer-wins: registering twice for the same `T` silently overwrites.
    // Intentional — lets tests install fixture extractors over the standard set
    // without a separate "override" entry point.
    pub fn register<T: 'static>(&mut self, extractor: Extractor<T>) {
        self.by_type.insert(
            TypeId::of::<T>(),
            Box::new(move |payload| payload.downcast_ref::<T>().map(extractor)),
        );
    }

    /// The comparable value of `stamped`'s payload.
    pub fn extract(&self, stamped: &dyn ErasedStamped) -> Result<AssertionValue, ExtractError> {
        let extractor =
            self.by_type
                .get(&stamped.payload_type_id())
                .ok_or(ExtractError::NoExtractor {
                    payload: stamped.payload_type_name(),
                })?;

        extractor(stamped.payload()).ok_or(ExtractError::PayloadMismatch {
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

pub fn standard_extractors() -> ExtractorTable {
    let mut table = ExtractorTable::new();
    table.register::<f64>(|x| AssertionValue::Float(*x));
    table.register::<i64>(|x| AssertionValue::Int(*x));
    table.register::<bool>(|x| AssertionValue::Bool(*x));
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
    fn register_overrides_previous_entry() {
        let mut table = standard_extractors();
        table.register::<f64>(|_| AssertionValue::Float(99.0));

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
