//! The world layout as the simulation uses it: every placement checked, its
//! class resolved to a `SemanticClass`, its orientation turned into a
//! rotation, and its body into a static or dynamic body with a mass; and
//! each prefab's `.glb` turned into a bounding box and a collider; and the
//! objects spawned from both.
//!
//! Resolution and geometry are pure functions over plain data, so a layout
//! written by hand and one produced by a generator pass the same checks.

mod assets;
mod error;
mod geometry;
mod gltf_nodes;
mod plugin;
mod resolved;
mod spawn;

pub use assets::ObjectAssets;
pub use error::LayoutError;
pub use geometry::{AssetNode, GeometryError, ObjectBounds, PrefabGeometry};
pub use gltf_nodes::read_scene;
pub use plugin::WorldLayoutPlugin;
pub use resolved::{ResolvedBody, ResolvedPlacement, ResolvedPrefab, ResolvedWorldLayout};
