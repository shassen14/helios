//! Attribute tables: the attribute columns a payload carries, readable by key.
//!
//! A table pairs a schema with one column per attribute, in the schema's order.
//! It is built only by core's builders, which validate columns before handing
//! them over, and never changes afterwards; filtering rows makes a new table.

use super::column::AttributeColumn;
use super::key::{AttributeKey, Element};
use super::schema::AttributeSchema;

/// Attribute columns with their definitions: `columns[i]` holds the values of
/// the schema's `i`-th attribute.
///
/// The schema is stored rather than rebuilt from the columns, so reading it
/// cannot fail, and its unique-name rule holds for the table by construction.
/// Empty (the `Default`) is a geometry-only payload. Cloning shares the values.
#[derive(Clone, Debug, Default)]
pub struct AttributeTable {
    schema: AttributeSchema,
    columns: Vec<AttributeColumn>,
}

impl AttributeTable {
    /// A table of `columns`, one per attribute of `schema`, in its order.
    ///
    /// The caller has already checked that the columns match the schema and
    /// each other in length; this only asserts the count in debug builds.
    pub(crate) fn new(schema: AttributeSchema, columns: Vec<AttributeColumn>) -> Self {
        debug_assert_eq!(schema.len(), columns.len());
        Self { schema, columns }
    }

    /// The attributes this table carries.
    pub fn schema(&self) -> &AttributeSchema {
        &self.schema
    }

    /// The values of the column `key` names, typed as `T`.
    ///
    /// `None` unless the table holds an attribute with `key`'s whole
    /// definition: a same-named attribute with another element type, marker,
    /// or blank is a different attribute, and reading it as this key would
    /// misread it.
    pub fn get<T: Element>(&self, key: AttributeKey<T>) -> Option<&[T]> {
        self.schema
            .iter()
            .position(|d| d == key.descriptor())
            .and_then(|i| self.columns.get(i))
            .and_then(T::view)
    }

    /// A new table keeping the rows whose `keep` flag is set, in every column.
    ///
    /// `keep` has one flag per row, not per column: every attribute survives,
    /// each shortened by the same mask, so the columns stay aligned with each
    /// other and with the geometry the caller filters alongside.
    pub fn select(&self, keep: &[bool]) -> Self {
        let schema = self.schema.clone();
        let columns = self
            .columns
            .iter()
            .map(|column| column.select(keep))
            .collect();

        Self { schema, columns }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interchange::measurement::attribute::canonical::{INTENSITY, RING};
    use crate::interchange::measurement::attribute::key::TransformMarker;

    /// `intensity` redefined as an integer: same name, different definition.
    const INTEGER_INTENSITY: AttributeKey<u16> =
        AttributeKey::no_blank("intensity", TransformMarker::Scalar);

    /// `intensity` with the right element type but no blank.
    const UNBLANKED_INTENSITY: AttributeKey<f32> =
        AttributeKey::no_blank("intensity", TransformMarker::Scalar);

    /// A lidar table: intensity `[0.1, 0.2, 0.3]`, ring `[0, 1, 2]`.
    fn lidar() -> AttributeTable {
        let schema = AttributeSchema::new([INTENSITY.descriptor(), RING.descriptor()])
            .expect("distinct core keys");
        AttributeTable::new(
            schema,
            vec![
                AttributeColumn::F32([0.1, 0.2, 0.3].into()),
                AttributeColumn::U16([0, 1, 2].into()),
            ],
        )
    }

    #[test]
    fn get_reads_each_column_by_its_key() {
        let table = lidar();
        assert_eq!(table.get(INTENSITY), Some(&[0.1, 0.2, 0.3][..]));
        assert_eq!(table.get(RING), Some(&[0, 1, 2][..]));
    }

    #[test]
    fn get_refuses_a_same_named_key_of_another_element_type() {
        assert_eq!(lidar().get(INTEGER_INTENSITY), None);
    }

    #[test]
    fn get_refuses_a_same_named_key_with_another_blank() {
        assert_eq!(lidar().get(UNBLANKED_INTENSITY), None);
    }

    #[test]
    fn get_misses_an_absent_key() {
        let table = AttributeTable::default();
        assert!(table.schema().is_empty());
        assert_eq!(table.get(INTENSITY), None);
    }

    /// The mask filters rows, never columns: dropping row 1 must not drop the
    /// second column.
    #[test]
    fn select_filters_rows_in_every_column() {
        let kept = lidar().select(&[true, false, true]);
        assert_eq!(kept.schema().len(), 2);
        assert_eq!(kept.get(INTENSITY), Some(&[0.1, 0.3][..]));
        assert_eq!(kept.get(RING), Some(&[0, 2][..]));
    }

    #[test]
    fn select_keeps_the_schema_even_when_every_row_is_dropped() {
        let kept = lidar().select(&[false; 3]);
        assert_eq!(kept.schema().len(), 2);
        assert_eq!(kept.get(RING), Some(&[][..]));
    }
}
