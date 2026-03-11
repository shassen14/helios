use serde::Deserialize;
use std::path::PathBuf;

/// Placement of a single world object instance, declared in a scenario TOML
/// under `[[world.objects]]`.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct WorldObjectPlacement {
    /// Key into the prefab catalog, e.g. `"objects.stop_sign"`.
    pub prefab: String,
    /// Position in ENU world frame: [x_east, y_north, z_up], meters.
    pub position: [f64; 3],
    /// Orientation as [roll, pitch, yaw] in **degrees** (converted to radians on spawn).
    /// Defaults to [0, 0, 0].
    #[serde(default)]
    pub orientation_degrees: [f64; 3],
    /// Per-axis scale applied to the visual mesh only. Defaults to [1, 1, 1].
    #[serde(default = "default_scale")]
    pub scale: [f32; 3],
}

fn default_scale() -> [f32; 3] {
    [1.0, 1.0, 1.0]
}

/// Object type definition stored in `configs/catalog/objects/<name>.toml`.
/// Describes geometry, physics, and semantic metadata for an object type.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct WorldObjectPrefab {
    /// Human-readable semantic label, e.g. `"stop_sign"`.
    pub label: String,
    /// Integer class ID used by perception algorithms and dataset labelers.
    pub class_id: u32,
    /// Path to the visual GLB/GLTF asset, relative to the Bevy asset root.
    /// If absent, no visual mesh is spawned (collider-only object).
    #[serde(default)]
    pub visual_mesh: Option<PathBuf>,
    /// Axis-aligned bounding box in object-local space: [width, height, depth] meters.
    /// Used for sensor queries and debug visualization. Optional.
    #[serde(default)]
    pub bounding_box: Option<[f32; 3]>,
    /// Physics collider shape. If absent, object has no physics collider.
    #[serde(default)]
    pub collider: Option<WorldObjectCollider>,
}

/// Collider shape descriptor for a world object.
/// Only the fields relevant to the chosen `shape` need to be present.
#[derive(Debug, Deserialize, Clone)]
pub struct WorldObjectCollider {
    /// Shape type: `"box"`, `"sphere"`, `"capsule"`, or `"cylinder"`.
    pub shape: String,
    /// Half-extents [hx, hy, hz] for `"box"` shape. In meters.
    #[serde(default)]
    pub half_extents: Option<[f32; 3]>,
    /// Radius for `"sphere"`, `"capsule"`, or `"cylinder"` shapes. In meters.
    #[serde(default)]
    pub radius: Option<f32>,
    /// Half-height of the cylindrical section for `"capsule"` or full half-height
    /// for `"cylinder"`. In meters.
    #[serde(default)]
    pub half_height: Option<f32>,
}
