//! Semantic classes: what an object *is* (`car`, `tree`, `traffic_cone`),
//! as a catalog loaded from config rather than a fixed enum, so the class
//! list can follow the dataset or model in use. The same catalog labels
//! sim ground truth and hardware perception output.
//!
//! A class says what a thing is, never how to treat it; groupings such as
//! "vehicle" or "drivable" belong to each consumer's own mapping.

use crate::kernel::identifier::is_snake_case;

use std::collections::BTreeMap;
use std::fmt;

use serde::Deserialize;

/// A validated class catalog: every name is unique and `snake_case`, every
/// ID is unique, and `unlabeled` has ID 0. Built only by deserializing, so
/// an invalid catalog never exists.
///
/// Names are used at load time and for display; at runtime a class is its
/// [`SemanticClass`] ID.
#[derive(Debug, Clone, Deserialize)]
#[serde(try_from = "RawCatalog")]
pub struct SemanticTaxonomy {
    name_to_class: BTreeMap<String, SemanticClass>,
    class_to_name: BTreeMap<SemanticClass, String>,
}

impl SemanticTaxonomy {
    /// The class called `name`, or an error listing every valid name.
    pub fn class(&self, name: &str) -> Result<SemanticClass, UnknownClass> {
        self.name_to_class
            .get(name)
            .copied()
            .ok_or_else(|| UnknownClass {
                name: name.to_string(),
                valid: self.name_to_class.keys().cloned().collect(),
            })
    }

    /// The name of `class`, or `None` if it came from a different catalog.
    pub fn name(&self, class: SemanticClass) -> Option<&str> {
        self.class_to_name.get(&class).map(|s| s.as_str())
    }
}

impl TryFrom<RawCatalog> for SemanticTaxonomy {
    type Error = TaxonomyError;

    /// Checks the entries in file order and stops at the first problem.
    fn try_from(raw: RawCatalog) -> Result<Self, Self::Error> {
        let mut name_to_class: BTreeMap<String, SemanticClass> = BTreeMap::new();
        let mut class_to_name: BTreeMap<SemanticClass, String> = BTreeMap::new();

        for entry in raw.class {
            let class = SemanticClass(entry.id);

            if !is_snake_case(&entry.name) {
                return Err(TaxonomyError::NotSnakeCase { name: entry.name });
            }

            if name_to_class.contains_key(&entry.name) {
                return Err(TaxonomyError::DuplicateName { name: entry.name });
            }

            if let Some(first) = class_to_name.get(&class) {
                return Err(TaxonomyError::DuplicateId {
                    id: entry.id,
                    first: first.clone(),
                    second: entry.name,
                });
            }

            name_to_class.insert(entry.name.clone(), class);
            class_to_name.insert(class, entry.name);
        }

        match name_to_class.get(SemanticClass::UNLABELED_NAME) {
            None => return Err(TaxonomyError::MissingUnlabeled),
            Some(class) if *class != SemanticClass::UNLABELED => {
                return Err(TaxonomyError::UnlabeledNotZero { id: class.id() });
            }
            Some(_) => {}
        }

        Ok(Self {
            name_to_class,
            class_to_name,
        })
    }
}

/// One semantic class, as its catalog ID: the integer written into label
/// images and per-point labels. Obtained from a [`SemanticTaxonomy`], so it
/// always names a class in some catalog.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SemanticClass(u16);

impl SemanticClass {
    /// What geometry with no class, and a ray that hits nothing labelled,
    /// reports. Every catalog has it.
    pub const UNLABELED: Self = Self(0);
    const UNLABELED_NAME: &str = "unlabeled";

    pub fn id(&self) -> u16 {
        self.0
    }
}

/// Why a catalog was rejected at load.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TaxonomyError {
    NotSnakeCase {
        name: String,
    },
    DuplicateName {
        name: String,
    },
    DuplicateId {
        id: u16,
        first: String,
        second: String,
    },
    MissingUnlabeled,
    UnlabeledNotZero {
        id: u16,
    },
}

impl fmt::Display for TaxonomyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotSnakeCase { name } => write!(
                f,
                "semantic class `{name}` is not snake_case; use lowercase letters, \
                 digits and single underscores, starting with a letter"
            ),
            Self::DuplicateName { name } => write!(
                f,
                "semantic class `{name}` is listed twice; each class appears once"
            ),
            Self::DuplicateId { id, first, second } => write!(
                f,
                "semantic classes `{first}` and `{second}` both have id {id}; give \
                 `{second}` the next unused id (never change an existing one)"
            ),
            Self::MissingUnlabeled => write!(
                f,
                "semantic class catalog has no `{}` class; add it with id {}",
                SemanticClass::UNLABELED_NAME,
                SemanticClass::UNLABELED.id()
            ),
            Self::UnlabeledNotZero { id } => write!(
                f,
                "semantic class `{}` has id {id}, but must have id {}",
                SemanticClass::UNLABELED_NAME,
                SemanticClass::UNLABELED.id()
            ),
        }
    }
}

impl std::error::Error for TaxonomyError {}

/// A class name the catalog does not contain.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownClass {
    name: String,
    /// Every name the catalog does contain, sorted.
    valid: Vec<String>,
}

impl fmt::Display for UnknownClass {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "semantic class `{}` is not in the catalog; valid classes are: {}",
            self.name,
            self.valid.join(", ")
        )
    }
}

impl std::error::Error for UnknownClass {}

/// The catalog file as written, before validation.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawCatalog {
    class: Vec<RawClass>,
}

/// One `[[class]]` entry as written.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawClass {
    name: String,
    id: u16,
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEFAULT_CATALOG: &str =
        include_str!("../../../../configs/runtime/catalog/semantic_classes/default.toml");

    const SMALL_CATALOG: &str = r#"
        [[class]]
        name = "unlabeled"
        id = 0

        [[class]]
        name = "car"
        id = 9

        [[class]]
        name = "traffic_cone"
        id = 15
    "#;

    /// Parses and validates, returning the validation error itself rather
    /// than serde's wrapped message, so tests can match on the variant.
    fn build(src: &str) -> Result<SemanticTaxonomy, TaxonomyError> {
        let raw: RawCatalog = toml::from_str(src).expect("test catalog is valid TOML");
        SemanticTaxonomy::try_from(raw)
    }

    #[test]
    fn default_catalog_is_valid() {
        let taxonomy: SemanticTaxonomy =
            toml::from_str(DEFAULT_CATALOG).expect("default catalog must load");

        assert_eq!(taxonomy.class("unlabeled"), Ok(SemanticClass::UNLABELED));
        assert_eq!(taxonomy.class("traffic_cone").map(|c| c.id()), Ok(15));
    }

    #[test]
    fn lookups_go_both_ways() {
        let taxonomy = build(SMALL_CATALOG).unwrap();

        let car = taxonomy.class("car").unwrap();
        assert_eq!(car.id(), 9);
        assert_eq!(taxonomy.name(car), Some("car"));
        assert_eq!(taxonomy.name(SemanticClass::UNLABELED), Some("unlabeled"));
    }

    #[test]
    fn file_order_does_not_assign_ids() {
        let reordered = r#"
            [[class]]
            name = "car"
            id = 9

            [[class]]
            name = "unlabeled"
            id = 0
        "#;
        let taxonomy = build(reordered).unwrap();

        assert_eq!(taxonomy.class("car").map(|c| c.id()), Ok(9));
    }

    #[test]
    fn class_from_another_catalog_has_no_name() {
        let taxonomy = build(SMALL_CATALOG).unwrap();

        assert_eq!(taxonomy.name(SemanticClass(4)), None);
    }

    #[test]
    fn unknown_class_lists_valid_names_sorted() {
        let taxonomy = build(SMALL_CATALOG).unwrap();

        let err = taxonomy.class("crate").unwrap_err();
        assert_eq!(err.name, "crate");
        assert_eq!(err.valid, ["car", "traffic_cone", "unlabeled"]);
        assert!(err.to_string().contains("car, traffic_cone, unlabeled"));
    }

    #[test]
    fn rejects_name_not_snake_case() {
        let src = r#"
            [[class]]
            name = "unlabeled"
            id = 0

            [[class]]
            name = "TrafficCone"
            id = 15
        "#;

        assert_eq!(
            build(src).unwrap_err(),
            TaxonomyError::NotSnakeCase {
                name: "TrafficCone".to_string()
            }
        );
    }

    #[test]
    fn rejects_duplicate_name() {
        let src = r#"
            [[class]]
            name = "unlabeled"
            id = 0

            [[class]]
            name = "car"
            id = 9

            [[class]]
            name = "car"
            id = 10
        "#;

        assert_eq!(
            build(src).unwrap_err(),
            TaxonomyError::DuplicateName {
                name: "car".to_string()
            }
        );
    }

    #[test]
    fn rejects_duplicate_id_naming_both_classes() {
        let src = r#"
            [[class]]
            name = "unlabeled"
            id = 0

            [[class]]
            name = "car"
            id = 9

            [[class]]
            name = "truck"
            id = 9
        "#;

        assert_eq!(
            build(src).unwrap_err(),
            TaxonomyError::DuplicateId {
                id: 9,
                first: "car".to_string(),
                second: "truck".to_string(),
            }
        );
    }

    #[test]
    fn rejects_missing_unlabeled() {
        let src = r#"
            [[class]]
            name = "car"
            id = 9
        "#;

        assert_eq!(build(src).unwrap_err(), TaxonomyError::MissingUnlabeled);
    }

    #[test]
    fn rejects_unlabeled_with_nonzero_id() {
        let src = r#"
            [[class]]
            name = "unlabeled"
            id = 5
        "#;

        assert_eq!(
            build(src).unwrap_err(),
            TaxonomyError::UnlabeledNotZero { id: 5 }
        );
    }

    #[test]
    fn validation_error_reaches_the_deserializer() {
        let src = r#"
            [[class]]
            name = "car"
            id = 9
        "#;

        let err = toml::from_str::<SemanticTaxonomy>(src).unwrap_err();
        assert!(err.to_string().contains("no `unlabeled` class"));
    }

    #[test]
    fn rejects_unknown_field() {
        let src = r#"
            [[class]]
            name = "unlabeled"
            id = 0
            colour = "black"
        "#;

        assert!(toml::from_str::<SemanticTaxonomy>(src).is_err());
    }

    #[test]
    fn rejects_id_outside_u16() {
        for id in ["-1", "65536"] {
            let src = format!("[[class]]\nname = \"unlabeled\"\nid = {id}\n");

            assert!(
                toml::from_str::<SemanticTaxonomy>(&src).is_err(),
                "id {id} should be rejected"
            );
        }
    }
}
