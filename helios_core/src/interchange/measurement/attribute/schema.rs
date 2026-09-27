//! Attribute schemas: which attribute columns a payload carries, and the two
//! pure checks the build runs over them.
//!
//! A producer's schema says what it provides; a consumer's says what it
//! requires. [`AttributeSchema::provides`] checks one against the other, and
//! [`check_agreement`] checks that every schema in a pipeline means the same
//! thing by each attribute name. Both report every problem at once, so one
//! failed build lists everything to fix.

use super::key::AttributeDescriptor;

/// The attribute columns a payload carries, in declaration order.
///
/// Names are unique within a schema. Order is kept for a deterministic dump
/// and serialization; the checks ignore it, so two schemas listing the same
/// attributes in different orders satisfy each other. That is also why this is
/// not `PartialEq`: a derived equality would call such schemas unequal. Empty
/// (the `Default`) is valid: a geometry-only producer, or a consumer that
/// requires nothing.
#[derive(Clone, Debug, Default)]
pub struct AttributeSchema {
    attributes: Vec<AttributeDescriptor>,
}

impl AttributeSchema {
    /// A schema of `attributes` in the given order, rejecting any name that
    /// appears more than once. Each duplicate is reported once, in first-seen
    /// order.
    pub fn new(
        attributes: impl IntoIterator<Item = AttributeDescriptor>,
    ) -> Result<Self, SchemaError> {
        let attributes: Vec<AttributeDescriptor> = attributes.into_iter().collect();

        let mut duplicates: Vec<&'static str> = Vec::new();

        for (index, attribute) in attributes.iter().enumerate() {
            let name = attribute.name();
            // A linear scan rather than a set: a schema holds a handful of
            // attributes and is checked once at build time, and a scan keeps the
            // duplicates in first-seen order for a deterministic error.
            let seen_before = attributes[..index]
                .iter()
                .any(|earlier| earlier.name() == name);

            if seen_before && !duplicates.contains(&name) {
                duplicates.push(name);
            }
        }

        if duplicates.is_empty() {
            Ok(Self { attributes })
        } else {
            Err(SchemaError::DuplicateNames { names: duplicates })
        }
    }

    /// The attributes in declaration order, yielded by value (descriptors are
    /// `Copy`); the iterator itself borrows the schema.
    pub fn iter(&self) -> impl Iterator<Item = AttributeDescriptor> + '_ {
        self.attributes.iter().copied()
    }

    /// The attribute named `name`, if this schema carries it.
    pub fn get(&self, name: &str) -> Option<AttributeDescriptor> {
        self.iter().find(|attribute| attribute.name() == name)
    }

    pub fn len(&self) -> usize {
        self.attributes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.attributes.is_empty()
    }

    /// Whether this (provided) schema satisfies `required`: every required
    /// attribute is present under the same definition. Extra attributes are
    /// fine — a consumer never asks for what it does not need.
    pub fn provides(&self, required: &AttributeSchema) -> Result<(), Vec<RequirementError>> {
        let problems: Vec<RequirementError> = required
            .iter()
            .filter_map(|wanted| match self.get(wanted.name()) {
                None => Some(RequirementError::Missing {
                    name: wanted.name(),
                }),
                Some(offered) if offered != wanted => Some(RequirementError::Mismatched {
                    required: wanted,
                    provided: offered,
                }),
                Some(_) => None,
            })
            .collect();

        if problems.is_empty() {
            Ok(())
        } else {
            Err(problems)
        }
    }
}

/// Why a schema could not be built.
#[derive(Debug, PartialEq, Eq)]
pub enum SchemaError {
    /// These names each appeared more than once, in first-seen order.
    DuplicateNames { names: Vec<&'static str> },
}

/// One way a provided schema falls short of a required one.
#[derive(Debug, PartialEq, Eq)]
pub enum RequirementError {
    /// The provider carries no attribute of this name.
    Missing { name: &'static str },
    /// The provider carries the name under a different definition (element
    /// type, marker, or blank).
    Mismatched {
        required: AttributeDescriptor,
        provided: AttributeDescriptor,
    },
}

/// One attribute name used with more than one definition across a set of
/// schemas.
#[derive(Debug, PartialEq, Eq)]
pub struct DefinitionConflict {
    pub name: &'static str,
    /// Every distinct definition seen for `name`, in first-seen order.
    pub definitions: Vec<AttributeDescriptor>,
}

/// Checks that every schema in `schemas` means the same thing by each
/// attribute name, reporting each name used with more than one definition.
///
/// Catches a drifted duplicate (the same name redefined with another element
/// type) and a third-party key that reuses a name under a different
/// definition. It cannot catch two keys with identical definitions but
/// different documented meanings: that is what namespacing non-core names
/// prevents. Output is in first-seen order, so the same pipeline always
/// reports the same way.
///
/// Takes the schemas by reference: the caller keeps owning them, and checking
/// never consumes what it checks.
pub fn check_agreement<'a>(
    schemas: impl IntoIterator<Item = &'a AttributeSchema>,
) -> Result<(), Vec<DefinitionConflict>> {
    let mut seen: Vec<DefinitionConflict> = Vec::new();
    for attribute in schemas.into_iter().flat_map(AttributeSchema::iter) {
        match seen.iter_mut().find(|entry| entry.name == attribute.name()) {
            Some(entry) if !entry.definitions.contains(&attribute) => {
                entry.definitions.push(attribute);
            }
            Some(_) => {}
            None => seen.push(DefinitionConflict {
                name: attribute.name(),
                definitions: vec![attribute],
            }),
        }
    }
    let conflicts: Vec<DefinitionConflict> = seen
        .into_iter()
        .filter(|entry| entry.definitions.len() > 1)
        .collect();
    if conflicts.is_empty() {
        Ok(())
    } else {
        Err(conflicts)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interchange::measurement::attribute::canonical::{INTENSITY, RING};
    use crate::interchange::measurement::attribute::key::{AttributeKey, TransformMarker};

    /// `intensity` redefined as an integer: the drifted duplicate the checks
    /// exist to catch.
    const INTEGER_INTENSITY: AttributeKey<u16> =
        AttributeKey::no_blank("intensity", TransformMarker::Scalar);

    fn schema(attributes: &[AttributeDescriptor]) -> AttributeSchema {
        AttributeSchema::new(attributes.iter().copied()).expect("test schema has unique names")
    }

    #[test]
    fn new_keeps_declaration_order() {
        let names: Vec<&str> = schema(&[RING.descriptor(), INTENSITY.descriptor()])
            .iter()
            .map(|attribute| attribute.name())
            .collect();
        assert_eq!(names, ["ring", "intensity"]);
    }

    #[test]
    fn new_reports_every_duplicate_name_once() {
        let result = AttributeSchema::new([
            INTENSITY.descriptor(),
            RING.descriptor(),
            INTENSITY.descriptor(),
            RING.descriptor(),
            INTENSITY.descriptor(),
        ]);
        assert_eq!(
            result.unwrap_err(),
            SchemaError::DuplicateNames {
                names: vec!["intensity", "ring"]
            }
        );
    }

    #[test]
    fn an_empty_schema_is_valid() {
        let empty = AttributeSchema::new([]).expect("empty schema");
        assert!(empty.is_empty());
        assert_eq!(empty.len(), 0);
    }

    #[test]
    fn get_finds_by_name_and_misses_cleanly() {
        let lidar = schema(&[INTENSITY.descriptor()]);
        assert_eq!(lidar.get("intensity"), Some(INTENSITY.descriptor()));
        assert_eq!(lidar.get("ring"), None);
    }

    /// The superset promise: a producer with more than a consumer needs still
    /// satisfies it, and a consumer requiring nothing is always met.
    #[test]
    fn a_superset_or_empty_requirement_is_provided() {
        let lidar = schema(&[INTENSITY.descriptor(), RING.descriptor()]);
        assert!(lidar.provides(&schema(&[INTENSITY.descriptor()])).is_ok());
        assert!(lidar.provides(&AttributeSchema::default()).is_ok());
    }

    #[test]
    fn order_does_not_affect_provides() {
        let lidar = schema(&[RING.descriptor(), INTENSITY.descriptor()]);
        let required = schema(&[INTENSITY.descriptor(), RING.descriptor()]);
        assert!(lidar.provides(&required).is_ok());
    }

    #[test]
    fn provides_reports_missing_and_mismatched_separately_and_all_at_once() {
        let provider = schema(&[INTEGER_INTENSITY.descriptor()]);
        let required = schema(&[INTENSITY.descriptor(), RING.descriptor()]);
        assert_eq!(
            provider.provides(&required).unwrap_err(),
            vec![
                RequirementError::Mismatched {
                    required: INTENSITY.descriptor(),
                    provided: INTEGER_INTENSITY.descriptor(),
                },
                RequirementError::Missing { name: "ring" },
            ]
        );
    }

    /// Two lidars declaring the same core key agree, including its NaN blank,
    /// which a value comparison would call unequal to itself.
    #[test]
    fn the_same_key_across_schemas_agrees() {
        let front = schema(&[INTENSITY.descriptor(), RING.descriptor()]);
        let rear = schema(&[INTENSITY.descriptor()]);
        assert!(check_agreement([&front, &rear]).is_ok());
    }

    #[test]
    fn disjoint_schemas_agree() {
        let a = schema(&[INTENSITY.descriptor()]);
        let b = schema(&[RING.descriptor()]);
        assert!(check_agreement([&a, &b]).is_ok());
    }

    #[test]
    fn a_name_redefined_across_schemas_is_one_conflict_listing_each_definition() {
        let a = schema(&[INTENSITY.descriptor()]);
        let b = schema(&[INTEGER_INTENSITY.descriptor()]);
        let c = schema(&[INTENSITY.descriptor(), RING.descriptor()]);
        assert_eq!(
            check_agreement([&a, &b, &c]).unwrap_err(),
            vec![DefinitionConflict {
                name: "intensity",
                definitions: vec![INTENSITY.descriptor(), INTEGER_INTENSITY.descriptor()],
            }]
        );
    }
}
