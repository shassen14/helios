//! Attribute keys: the typed name of one attribute column, and its erased form
//! for schemas and build-time checks.
//!
//! A key is a `const` shared by producer and consumer, so neither side types the
//! column's name as a string. [`AttributeKey<T>`] carries the element type in
//! `T`, so a typed read cannot misread the column; [`AttributeDescriptor`] is the
//! same definition with `T` recorded as an [`ElementType`] value, for places that
//! hold keys of mixed element types (schemas, build-time agreement checks).

use std::marker::PhantomData;

/// The typed name of one attribute column.
///
/// Built only through the constructors below, one per blank policy; each is
/// bounded so an element type can only take a blank it can represent (a `u16`
/// key cannot blank to NaN — that constructor does not exist for it). Keys are
/// meant to be `const` items shared by producer and consumer.
///
/// Not `PartialEq`: the derive would bound on `T`, and `f32` is not `Eq`.
/// Compare definitions through [`descriptor`](Self::descriptor).
#[derive(Clone, Copy, Debug)]
pub struct AttributeKey<T: Element> {
    name: &'static str,
    marker: TransformMarker,
    blank: BlankPolicy,
    element: PhantomData<T>,
}

impl<T: NanBlankable> AttributeKey<T> {
    /// A key whose unwritten cells read as NaN.
    pub const fn nan_blank(name: &'static str, marker: TransformMarker) -> Self {
        Self {
            name,
            marker,
            blank: BlankPolicy::Nan,
            element: PhantomData,
        }
    }
}

impl<T: Element> AttributeKey<T> {
    /// A key with no blank: legal only for a column that can never be missing —
    /// one derived from position (a ring index) or measured on every cell,
    /// misses included (passive ambient light). A grid pre-fills every other
    /// column with its blank, so a grid holding a no-blank column must have
    /// every cell of it written before it can be finalized.
    pub const fn no_blank(name: &'static str, marker: TransformMarker) -> Self {
        Self {
            name,
            marker,
            blank: BlankPolicy::NoBlank,
            element: PhantomData,
        }
    }

    pub const fn name(&self) -> &'static str {
        self.name
    }

    pub const fn marker(&self) -> TransformMarker {
        self.marker
    }

    pub const fn blank(&self) -> BlankPolicy {
        self.blank
    }

    /// This key's definition with `T` recorded as a runtime [`ElementType`].
    pub const fn descriptor(&self) -> AttributeDescriptor {
        AttributeDescriptor {
            name: self.name,
            element_type: T::ELEMENT_TYPE,
            marker: self.marker,
            blank: self.blank,
        }
    }
}

/// The runtime tag for an attribute column's element type — the erased
/// counterpart of the `T` in [`AttributeKey<T>`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ElementType {
    F32,
    U16,
}

mod sealed {
    pub trait Sealed {}
    impl Sealed for f32 {}
    impl Sealed for u16 {}
}

/// A Rust type that may be an attribute column's element.
///
/// Sealed, unlike `Frame`: serialization and grid pre-fill must handle every
/// element type, so third parties may define new *keys* but never new
/// *element types*. Adding one is a core change: a variant on [`ElementType`],
/// an impl here and on `sealed::Sealed`, and the blank capabilities it supports.
pub trait Element: sealed::Sealed + Copy + Send + Sync + 'static {
    const ELEMENT_TYPE: ElementType;
}

impl Element for f32 {
    const ELEMENT_TYPE: ElementType = ElementType::F32;
}

impl Element for u16 {
    const ELEMENT_TYPE: ElementType = ElementType::U16;
}

/// Element types that can blank to NaN.
///
/// A capability, not a numeric category: it names which blank a type can
/// represent, so a type with no spare value to mean "empty" simply implements
/// no blank capability and can only take [`AttributeKey::no_blank`].
pub trait NanBlankable: Element {}

impl NanBlankable for f32 {}

/// How a column behaves when its cloud is re-expressed in another frame.
///
/// Every `match` on this is exhaustive with no wildcard arm, so adding a
/// variant fails to compile until re-expression handles it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TransformMarker {
    /// Unchanged by a change of frame (intensity, ring).
    Scalar,
}

/// What an unwritten cell of a column reads as.
///
/// No variant holds a float, so equality is exact: two NaN-blank keys compare
/// equal by variant, never by comparing NaN to NaN.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BlankPolicy {
    /// Unwritten cells read as NaN, which no real value can be.
    Nan,
    /// The column is never missing, so it has no blank.
    NoBlank,
}

/// An [`AttributeKey`]'s definition with its element type erased to a value.
///
/// Equality is "same definition": the check that two keys sharing a name
/// agree. Constructed only from a key, so it can never pair an element type
/// with a blank that type cannot represent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct AttributeDescriptor {
    name: &'static str,
    element_type: ElementType,
    marker: TransformMarker,
    blank: BlankPolicy,
}

impl AttributeDescriptor {
    pub const fn name(&self) -> &'static str {
        self.name
    }

    pub const fn element_type(&self) -> ElementType {
        self.element_type
    }

    pub const fn marker(&self) -> TransformMarker {
        self.marker
    }

    pub const fn blank(&self) -> BlankPolicy {
        self.blank
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FLOAT_KEY: AttributeKey<f32> =
        AttributeKey::nan_blank("test_float", TransformMarker::Scalar);

    #[test]
    fn descriptor_carries_every_field() {
        let descriptor = FLOAT_KEY.descriptor();
        assert_eq!(descriptor.name(), "test_float");
        assert_eq!(descriptor.element_type(), ElementType::F32);
        assert_eq!(descriptor.marker(), TransformMarker::Scalar);
        assert_eq!(descriptor.blank(), BlankPolicy::Nan);
    }

    /// NaN is not equal to itself, so a definition comparison that compared
    /// blank values would report a NaN-blank key as disagreeing with itself.
    #[test]
    fn nan_blank_key_equals_itself() {
        assert_eq!(FLOAT_KEY.descriptor(), FLOAT_KEY.descriptor());
    }

    #[test]
    fn element_type_alone_distinguishes_definitions() {
        let float = AttributeKey::<f32>::no_blank("x", TransformMarker::Scalar);
        let integer = AttributeKey::<u16>::no_blank("x", TransformMarker::Scalar);
        assert_ne!(float.descriptor(), integer.descriptor());
    }

    #[test]
    fn blank_alone_distinguishes_definitions() {
        let nan = AttributeKey::<f32>::nan_blank("x", TransformMarker::Scalar);
        let none = AttributeKey::<f32>::no_blank("x", TransformMarker::Scalar);
        assert_ne!(nan.descriptor(), none.descriptor());
    }
}
