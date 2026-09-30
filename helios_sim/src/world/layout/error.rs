//! Why a world layout is rejected, one variant per load-time check.

use super::resolved::MESH_EXTENSION;

use helios_core::interchange::perception::semantic_class::UnknownClass;

use std::fmt;
use std::path::PathBuf;

/// Why a layout was rejected at load.
#[derive(Debug, Clone, PartialEq)]
pub enum LayoutError {
    LayoutNameNotSnakeCase {
        name: String,
    },
    PlacementNameNotSnakeCase {
        placement: String,
    },
    DuplicatePlacement {
        placement: String,
    },
    UnknownPrefab {
        placement: String,
        prefab: String,
    },
    NotFinite {
        placement: String,
        field: &'static str,
    },
    NonPositiveScale {
        placement: String,
        scale: [f64; 3],
    },
    MeshNotRelativeGlb {
        prefab: String,
        mesh: PathBuf,
    },
    InvalidMass {
        prefab: String,
        mass_kg: f64,
    },
    UnknownClass {
        prefab: String,
        source: UnknownClass,
    },
    DynamicWithoutMass {
        placement: String,
        prefab: String,
    },
    DynamicWithoutCollider {
        placement: String,
        prefab: String,
    },
}

impl fmt::Display for LayoutError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::LayoutNameNotSnakeCase { name } => write!(
                f,
                "layout name `{name}` is not snake_case; use lowercase letters, digits \
                 and single underscores, starting with a letter"
            ),
            Self::PlacementNameNotSnakeCase { placement } => write!(
                f,
                "placement `{placement}`: name is not snake_case; use lowercase \
                 letters, digits and single underscores, starting with a letter"
            ),
            Self::DuplicatePlacement { placement } => write!(
                f,
                "placement `{placement}`: name is used twice; each placement's name \
                 is its instance ID and must be unique"
            ),
            Self::UnknownPrefab { placement, prefab } => write!(
                f,
                "placement `{placement}`: prefab `{prefab}` was not loaded"
            ),
            Self::NotFinite { placement, field } => write!(
                f,
                "placement `{placement}`: `{field}` has a value that is not finite"
            ),
            Self::NonPositiveScale { placement, scale } => write!(
                f,
                "placement `{placement}`: scale {scale:?} must be positive on every \
                 axis (zero flattens the object, negative mirrors it)"
            ),
            Self::MeshNotRelativeGlb { prefab, mesh } => write!(
                f,
                "prefab `{prefab}`: mesh {mesh:?} must be a path to a \
                 .{MESH_EXTENSION} relative to the asset folder"
            ),
            Self::InvalidMass { prefab, mass_kg } => write!(
                f,
                "prefab `{prefab}`: mass_kg {mass_kg} must be a positive number"
            ),
            Self::UnknownClass { prefab, source } => write!(f, "prefab `{prefab}`: {source}"),
            Self::DynamicWithoutMass { placement, prefab } => write!(
                f,
                "placement `{placement}`: dynamic, but prefab `{prefab}` has no \
                 mass_kg; add one to the prefab or make the placement static"
            ),
            Self::DynamicWithoutCollider { placement, prefab } => write!(
                f,
                "placement `{placement}`: dynamic, but prefab `{prefab}` has \
                 collides = false, so it would fall through the ground"
            ),
        }
    }
}

impl std::error::Error for LayoutError {}
