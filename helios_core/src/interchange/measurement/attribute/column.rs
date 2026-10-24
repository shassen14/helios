//! Attribute columns: one attribute's values, one per point (or per cell),
//! stored contiguously and shared by `Arc`.
//!
//! A column is erased: its element type is a variant, not a type parameter, so
//! columns of different element types sit in one table. Typed access goes back
//! through a key, which checks the variant before handing out a slice.

use super::key::{AttributeDescriptor, BlankPolicy, ElementType};

use std::sync::Arc;

/// The values of one attribute column, one variant per element type.
///
/// The variant set mirrors [`ElementType`] and is
/// closed for the same reason: serialization and grid pre-fill must handle
/// every member. Every `match` on it names each variant (no wildcard arm), so
/// adding an element type fails to compile until each operation handles it.
///
/// Cloning is cheap: it shares the values and bumps a reference count.
#[derive(Clone, Debug)]
pub enum AttributeColumn {
    F32(Arc<[f32]>),
    U16(Arc<[u16]>),
}

impl AttributeColumn {
    /// A column of `len` values, each the blank of the attribute `descriptor`
    /// defines, or `None` if that attribute has no blank its element type can
    /// hold. This is how a grid starts out before any cell is written.
    pub(crate) fn blank(descriptor: AttributeDescriptor, len: usize) -> Option<Self> {
        match (descriptor.blank(), descriptor.element_type()) {
            (BlankPolicy::Nan, ElementType::F32) => Some(Self::F32(vec![f32::NAN; len].into())),
            // Unreachable, since only `f32` keys can be NaN-blank, but a
            // descriptor is data, so the pair is still handled.
            (BlankPolicy::Nan, ElementType::U16) => None,
            (BlankPolicy::NoBlank, _) => None,
        }
    }

    /// The number of values: one per point, or one per cell of a grid.
    pub fn len(&self) -> usize {
        match self {
            Self::F32(a) => a.len(),
            Self::U16(a) => a.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        match self {
            Self::F32(a) => a.is_empty(),
            Self::U16(a) => a.is_empty(),
        }
    }

    /// A new column of the values whose `keep` flag is set, in their original
    /// order and with the same element type.
    ///
    /// `keep` has one flag per value; the cloud applies the same mask to its
    /// geometry, time, and every column, which is what keeps them aligned.
    pub fn select(&self, keep: &[bool]) -> Self {
        debug_assert_eq!(keep.len(), self.len());
        match self {
            Self::F32(values) => Self::F32(keep_rows(values, keep)),
            Self::U16(values) => Self::U16(keep_rows(values, keep)),
        }
    }
}

/// The values whose `keep` flag is set, in order. Shared by every variant of
/// [`AttributeColumn::select`], since keeping rows does not depend on the
/// element type.
fn keep_rows<T: Copy>(values: &[T], keep: &[bool]) -> Arc<[T]> {
    values
        .iter()
        .zip(keep)
        .filter_map(|(v, k)| k.then_some(*v))
        .collect::<Vec<T>>()
        .into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interchange::measurement::attribute::canonical::{INTENSITY, RING};

    fn floats(values: &[f32]) -> AttributeColumn {
        AttributeColumn::F32(values.into())
    }

    fn integers(values: &[u16]) -> AttributeColumn {
        AttributeColumn::U16(values.into())
    }

    #[test]
    fn a_blank_nan_column_is_len_nans() {
        let AttributeColumn::F32(values) =
            AttributeColumn::blank(INTENSITY.descriptor(), 6).expect("intensity has a blank")
        else {
            panic!("an f32 key blanked into another element type");
        };
        assert_eq!(values.len(), 6);
        assert!(values.iter().all(|v| v.is_nan()));
    }

    #[test]
    fn a_key_with_no_blank_has_no_blank_column() {
        assert!(AttributeColumn::blank(RING.descriptor(), 6).is_none());
    }

    #[test]
    fn len_and_is_empty_agree_for_every_variant() {
        assert_eq!(floats(&[0.1, 0.2]).len(), 2);
        assert!(!floats(&[0.1]).is_empty());
        assert!(floats(&[]).is_empty());

        assert_eq!(integers(&[3, 4, 5]).len(), 3);
        assert!(integers(&[]).is_empty());
    }

    #[test]
    fn select_keeps_flagged_values_in_order() {
        let AttributeColumn::U16(kept) =
            integers(&[10, 11, 12, 13]).select(&[true, false, false, true])
        else {
            panic!("a u16 column selected into another element type");
        };
        assert_eq!(&*kept, &[10, 13]);
    }

    /// Selection copies values, never compares them, so a NaN blank survives
    /// as a blank rather than being dropped or altered.
    #[test]
    fn select_keeps_nan_blanks() {
        let AttributeColumn::F32(kept) =
            floats(&[f32::NAN, 0.5, f32::NAN]).select(&[true, true, false])
        else {
            panic!("an f32 column selected into another element type");
        };
        assert_eq!(kept.len(), 2);
        assert!(kept[0].is_nan());
        assert_eq!(kept[1], 0.5);
    }

    #[test]
    fn select_all_or_none() {
        let column = floats(&[0.1, 0.2, 0.3]);
        assert_eq!(column.select(&[true; 3]).len(), 3);
        assert!(column.select(&[false; 3]).is_empty());
    }

    #[test]
    fn clone_shares_values() {
        let column = floats(&[0.1, 0.2]);
        let (AttributeColumn::F32(original), AttributeColumn::F32(copy)) =
            (&column, &column.clone())
        else {
            panic!("clone changed the element type");
        };
        assert!(Arc::ptr_eq(original, copy));
    }

    /// A mask of the wrong length would otherwise silently drop the tail
    /// through `zip`.
    #[test]
    #[cfg(debug_assertions)]
    #[should_panic]
    fn select_rejects_a_mask_of_the_wrong_length() {
        integers(&[1, 2, 3]).select(&[true, false]);
    }
}
