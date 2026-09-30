//! Why a world layout is rejected, one variant per load-time check, and the
//! capped report a failed run prints.

use super::resolved::MESH_EXTENSION;

use helios_core::interchange::perception::semantic_class::UnknownClass;

use std::fmt;
use std::path::PathBuf;

/// How many errors a failed load lists before summarising the rest. A broken
/// generator can produce one error per placement.
const MAX_REPORTED_ERRORS: usize = 50;

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

/// The failure message: `heading` with the error count, every error up to
/// [`MAX_REPORTED_ERRORS`], then a count of the rest.
pub(super) fn error_report(heading: &str, errors: &[impl fmt::Display]) -> String {
    let mut report = format!("{heading}. Errors: {}", errors.len());
    for error in errors.iter().take(MAX_REPORTED_ERRORS) {
        report.push_str(&format!("\n{error}"));
    }
    if errors.len() > MAX_REPORTED_ERRORS {
        report.push_str(&format!(
            "\n... and {} more",
            errors.len() - MAX_REPORTED_ERRORS
        ));
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_report_caps_the_list_and_counts_the_rest() {
        let errors: Vec<LayoutError> = (0..MAX_REPORTED_ERRORS + 7)
            .map(|i| LayoutError::DuplicatePlacement {
                placement: format!("crate_{i}"),
            })
            .collect();

        let report = error_report("world layout `yard` is invalid", &errors);

        assert_eq!(report.lines().count(), 1 + MAX_REPORTED_ERRORS + 1);
        assert!(report.ends_with("... and 7 more"), "{report}");
    }
}
