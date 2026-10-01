//! The checked, runtime form of a layout, and the pure function that builds
//! it from the loaded config.

use super::error::LayoutError;
use crate::config::structs::{BodyKind, ObjectPlacement, ObjectPrefab};
use crate::config::LoadedWorldLayout;

use helios_core::interchange::perception::semantic_class::{SemanticClass, SemanticTaxonomy};
use helios_core::kernel::identifier::is_snake_case;

use bevy::prelude::Resource;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;

/// Separates the layout name from the placement name in an instance ID.
/// Neither name can contain it, since both are `snake_case`.
const INSTANCE_ID_SEPARATOR: char = '/';

/// The only geometry format the object loader reads.
pub(super) const MESH_EXTENSION: &str = "glb";

/// The only scale a dynamic placement may have. A prefab's mass is one
/// number, so a scaled dynamic copy would keep the mass of the unscaled one.
const DYNAMIC_SCALE: [f64; 3] = [1.0; 3];

/// A checked layout, ready to spawn. Built only by [`resolve`](Self::resolve),
/// so every placement's prefab exists and every value is valid.
#[derive(Resource, Debug, Clone)]
pub struct ResolvedWorldLayout {
    name: String,
    /// One entry per prefab key, shared by every placement of it.
    prefabs: Vec<ResolvedPrefab>,
    placements: Vec<ResolvedPlacement>,
    taxonomy: SemanticTaxonomy,
}

impl ResolvedWorldLayout {
    /// Checks `loaded` and builds the runtime layout, or returns every
    /// problem found.
    pub fn resolve(loaded: &LoadedWorldLayout) -> Result<Self, Vec<LayoutError>> {
        let layout = &loaded.layout;
        let mut errors = Vec::new();

        if !is_snake_case(&layout.name) {
            errors.push(LayoutError::LayoutNameNotSnakeCase {
                name: layout.name.clone(),
            });
        }

        let mut prefabs = Vec::with_capacity(loaded.prefabs.len());
        let mut prefab_index: HashMap<&str, usize> = HashMap::new();
        for (key, prefab) in &loaded.prefabs {
            if let Some(resolved) = resolve_prefab(key, prefab, &loaded.taxonomy, &mut errors) {
                prefab_index.insert(key, prefabs.len());
                prefabs.push(resolved);
            }
        }

        let mut names: HashSet<&str> = HashSet::with_capacity(layout.objects.len());
        let mut placements = Vec::with_capacity(layout.objects.len());
        for placement in &layout.objects {
            if !names.insert(&placement.name) {
                errors.push(LayoutError::DuplicatePlacement {
                    placement: placement.name.clone(),
                });
            }
            let Some(prefab) = loaded.prefabs.get(&placement.prefab) else {
                errors.push(LayoutError::UnknownPrefab {
                    placement: placement.name.clone(),
                    prefab: placement.prefab.clone(),
                });
                continue;
            };
            let index = prefab_index.get(placement.prefab.as_str()).copied();
            if let Some(resolved) = resolve_placement(placement, prefab, index, &mut errors) {
                placements.push(resolved);
            }
        }

        if !errors.is_empty() {
            return Err(errors);
        }
        Ok(Self {
            name: layout.name.clone(),
            prefabs,
            placements,
            taxonomy: loaded.taxonomy.clone(),
        })
    }

    /// The layout's declared name: the prefix of every instance ID in it.
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn placements(&self) -> &[ResolvedPlacement] {
        &self.placements
    }

    /// Every prefab the layout uses, once each: what to load before
    /// spawning.
    pub fn prefabs(&self) -> &[ResolvedPrefab] {
        &self.prefabs
    }

    /// Every placement with the prefab it places, in file order: what to
    /// spawn. Paired here, by the type that built the indices, so no caller
    /// can pair a placement with the wrong layout's prefabs.
    pub fn objects(&self) -> impl Iterator<Item = (&ResolvedPlacement, &ResolvedPrefab)> {
        self.placements
            .iter()
            .map(|placement| (placement, &self.prefabs[placement.prefab]))
    }

    /// The object's identity in ground truth: `<layout>/<placement>`, e.g.
    /// `yard/crate_a`. Independent of list order and of the seed.
    pub fn instance_id(&self, placement: &ResolvedPlacement) -> String {
        format!("{}{INSTANCE_ID_SEPARATOR}{}", self.name, placement.name)
    }

    /// The class catalog every label in this world comes from.
    pub fn taxonomy(&self) -> &SemanticTaxonomy {
        &self.taxonomy
    }
}

/// A prefab with its class resolved against the scenario's catalog.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPrefab {
    /// The catalog key, e.g. `entities.objects.crate_1m`.
    pub key: String,
    /// The `.glb`, relative to the Bevy asset root.
    pub mesh: PathBuf,
    pub class: SemanticClass,
    pub collides: bool,
}

/// Checks one prefab and resolves its class. Mass is checked here, whatever
/// the body, since a non-positive mass is wrong wherever it is written.
fn resolve_prefab(
    key: &str,
    prefab: &ObjectPrefab,
    taxonomy: &SemanticTaxonomy,
    errors: &mut Vec<LayoutError>,
) -> Option<ResolvedPrefab> {
    let before = errors.len();

    if prefab.mesh.is_absolute()
        || prefab.mesh.extension().and_then(|e| e.to_str()) != Some(MESH_EXTENSION)
    {
        errors.push(LayoutError::MeshNotRelativeGlb {
            prefab: key.to_string(),
            mesh: prefab.mesh.clone(),
        });
    }
    if let Some(mass_kg) = prefab.mass_kg {
        if !(mass_kg.is_finite() && mass_kg > 0.0) {
            errors.push(LayoutError::InvalidMass {
                prefab: key.to_string(),
                mass_kg,
            });
        }
    }
    let class = match taxonomy.class(&prefab.class) {
        Ok(class) => Some(class),
        Err(source) => {
            errors.push(LayoutError::UnknownClass {
                prefab: key.to_string(),
                source,
            });
            None
        }
    };

    if errors.len() > before {
        return None;
    }
    Some(ResolvedPrefab {
        key: key.to_string(),
        mesh: prefab.mesh.clone(),
        class: class?,
        collides: prefab.collides,
    })
}

/// One placed object, in SI units and ENU.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPlacement {
    /// Unique within the layout.
    pub name: String,
    /// Index into the layout's prefabs; read through
    /// [`ResolvedWorldLayout::objects`].
    prefab: usize,
    /// The object's frame in the ENU world frame.
    pub pose: Isometry3<f64>,
    /// Per-axis scale along the object's own axes; every component positive.
    pub scale: Vector3<f64>,
    pub body: ResolvedBody,
}

impl ResolvedPlacement {
    /// The position of this placement's prefab in the resolving layout's
    /// [`prefabs`](ResolvedWorldLayout::prefabs), for data kept per prefab
    /// in the same order (loaded assets, derived geometry). Meaningful only
    /// against that one layout.
    pub fn prefab_index(&self) -> usize {
        self.prefab
    }
}

/// How physics treats a placed object, with what that needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ResolvedBody {
    Static,
    /// Always has a positive mass and a collider.
    Dynamic {
        mass_kg: f64,
    },
}

/// Checks one placement against its prefab and converts it. `prefab_index`
/// is `None` when the prefab itself failed; the placement is still checked,
/// so its own errors are reported too, but not built.
fn resolve_placement(
    placement: &ObjectPlacement,
    prefab: &ObjectPrefab,
    prefab_index: Option<usize>,
    errors: &mut Vec<LayoutError>,
) -> Option<ResolvedPlacement> {
    let before = errors.len();
    let name = || placement.name.clone();

    if !is_snake_case(&placement.name) {
        errors.push(LayoutError::PlacementNameNotSnakeCase { placement: name() });
    }
    for (field, values) in [
        ("position", placement.position),
        ("orientation_degrees", placement.orientation_degrees),
        ("scale", placement.scale),
    ] {
        if values.iter().any(|v| !v.is_finite()) {
            errors.push(LayoutError::NotFinite {
                placement: name(),
                field,
            });
        }
    }
    if placement.scale.iter().any(|&s| s <= 0.0) {
        errors.push(LayoutError::NonPositiveScale {
            placement: name(),
            scale: placement.scale,
        });
    }

    let body = match placement.body {
        BodyKind::Static => Some(ResolvedBody::Static),
        BodyKind::Dynamic => {
            if placement.scale != DYNAMIC_SCALE {
                errors.push(LayoutError::ScaledDynamic {
                    placement: name(),
                    scale: placement.scale,
                });
            }
            if !prefab.collides {
                errors.push(LayoutError::DynamicWithoutCollider {
                    placement: name(),
                    prefab: placement.prefab.clone(),
                });
            }
            match prefab.mass_kg {
                Some(mass_kg) => Some(ResolvedBody::Dynamic { mass_kg }),
                None => {
                    errors.push(LayoutError::DynamicWithoutMass {
                        placement: name(),
                        prefab: placement.prefab.clone(),
                    });
                    None
                }
            }
        }
    };

    if errors.len() > before {
        return None;
    }
    let [roll, pitch, yaw] = placement.orientation_degrees.map(f64::to_radians);
    Some(ResolvedPlacement {
        name: name(),
        prefab: prefab_index?,
        pose: Isometry3::from_parts(
            Translation3::from(Vector3::from(placement.position)),
            UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        ),
        scale: Vector3::from(placement.scale),
        body: body?,
    })
}

#[cfg(test)]
#[path = "resolved_tests.rs"]
mod tests;
