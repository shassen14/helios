use crate::assertion::condition::AssertionValue;

use std::{
    any::{Any, TypeId},
    collections::HashMap,
};

// `fn` (not `Fn`) means only captureless closures coerce in. That's deliberate —
// the table stores function pointers, not boxed closures, so registrations stay
// cheap to copy and free of per-entry allocation. If you ever need a registration
// that closes over state, switch the alias to `Box<dyn Fn(...)>` and pay the
// allocation; until then, every extractor is a pure type projection.
pub type Extractor = fn(&dyn Any) -> Option<AssertionValue>;

// Bridges typed bus payloads to the untyped `AssertionValue` space the evaluator
// compares against TOML. Keyed by the *payload* `TypeId` — the evaluator unwraps
// `Stamped<T>` before calling `extract`, so registrations target `T`, never
// `Stamped<T>`.
pub struct ExtractorTable {
    by_type: HashMap<TypeId, Extractor>,
}

impl ExtractorTable {
    pub fn new() -> Self {
        ExtractorTable {
            by_type: HashMap::new(),
        }
    }

    // Last-writer-wins: registering twice for the same `T` silently overwrites.
    // Intentional — lets tests install fixture extractors over the standard set
    // without a separate "override" entry point.
    pub fn register<T: 'static>(&mut self, extractor: Extractor) {
        self.by_type.insert(TypeId::of::<T>(), extractor);
    }

    // Returns `None` for two distinct cases collapsed for now: (1) no extractor
    // registered for `type_id`, (2) extractor returned `None` (downcast failed,
    // which would indicate the caller passed a `TypeId` that doesn't match the
    // `&dyn Any`'s real type). Split into a richer error when a test needs to
    // distinguish them.
    pub fn extract(&self, type_id: TypeId, value: &dyn Any) -> Option<AssertionValue> {
        let extractor = self.by_type.get(&type_id)?;
        extractor(value)
    }
}

impl Default for ExtractorTable {
    fn default() -> Self {
        Self::new()
    }
}

pub fn standard_extractors() -> ExtractorTable {
    let mut table = ExtractorTable::new();
    table.register::<f64>(|x| x.downcast_ref::<f64>().copied().map(AssertionValue::Float));
    table.register::<i64>(|x| x.downcast_ref::<i64>().copied().map(AssertionValue::Int));
    table.register::<bool>(|x| x.downcast_ref::<bool>().copied().map(AssertionValue::Bool));
    table
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_extractors_handles_f64() {
        let table = standard_extractors();
        let v: f64 = 3.14;
        assert_eq!(
            table.extract(TypeId::of::<f64>(), &v as &dyn Any),
            Some(AssertionValue::Float(3.14))
        );
    }

    #[test]
    fn standard_extractors_handles_i64() {
        let table = standard_extractors();
        let v: i64 = 42;
        assert_eq!(
            table.extract(TypeId::of::<i64>(), &v as &dyn Any),
            Some(AssertionValue::Int(42))
        );
    }

    #[test]
    fn standard_extractors_handles_bool() {
        let table = standard_extractors();
        let v: bool = true;
        assert_eq!(
            table.extract(TypeId::of::<bool>(), &v as &dyn Any),
            Some(AssertionValue::Bool(true))
        );
    }

    #[test]
    fn extract_returns_none_for_unregistered_type() {
        let table = standard_extractors();
        let v: String = "hello".to_string();
        assert_eq!(
            table.extract(TypeId::of::<String>(), &v as &dyn Any),
            None
        );
    }

    #[test]
    fn extract_returns_none_when_type_id_mismatches_payload() {
        // Caller passed `TypeId::of::<f64>()` but the payload is actually `i64`.
        // The downcast inside the extractor fails and we get `None`.
        let table = standard_extractors();
        let wrong: i64 = 7;
        assert_eq!(table.extract(TypeId::of::<f64>(), &wrong as &dyn Any), None);
    }

    #[test]
    fn register_overrides_previous_entry() {
        let mut table = standard_extractors();
        table.register::<f64>(|_| Some(AssertionValue::Float(99.0)));
        let v: f64 = 1.0;
        assert_eq!(
            table.extract(TypeId::of::<f64>(), &v as &dyn Any),
            Some(AssertionValue::Float(99.0))
        );
    }

    #[test]
    fn empty_table_returns_none() {
        let table = ExtractorTable::new();
        let v: f64 = 1.0;
        assert_eq!(table.extract(TypeId::of::<f64>(), &v as &dyn Any), None);
    }

    #[test]
    fn default_is_empty() {
        let table = ExtractorTable::default();
        let v: f64 = 1.0;
        assert_eq!(table.extract(TypeId::of::<f64>(), &v as &dyn Any), None);
    }
}
