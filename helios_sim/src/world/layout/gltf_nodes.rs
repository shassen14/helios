//! Reads a `.glb`'s scene as plain [`AssetNode`]s: node names, local
//! transforms, vertex positions and triangles, nothing else.
//!
//! Reads the glTF document itself, not Bevy's processed meshes, so the
//! geometry is exactly what Blender exported whatever Bevy does to render
//! it.

use super::geometry::{AssetNode, GeometryError};

use gltf::buffer::Source;
use gltf::mesh::Mode;
use nalgebra::{Matrix4, Point3};

/// The scene's root nodes: the default scene, or the first when none is
/// marked default.
pub fn read_scene(gltf: &gltf::Gltf) -> Result<Vec<AssetNode>, GeometryError> {
    if gltf
        .document
        .buffers()
        .any(|buffer| matches!(buffer.source(), Source::Uri(_)))
    {
        return Err(GeometryError::ExternalBuffer);
    }
    let scene = gltf
        .document
        .default_scene()
        .or_else(|| gltf.document.scenes().next())
        .ok_or(GeometryError::NoScene)?;
    let blob = gltf.blob.as_deref();

    Ok(scene.nodes().map(|node| read_node(&node, blob)).collect())
}

fn read_node(node: &gltf::Node, blob: Option<&[u8]>) -> AssetNode {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    if let Some(mesh) = node.mesh() {
        for primitive in mesh.primitives() {
            let reader = primitive.reader(|_| blob);
            let Some(read) = reader.read_positions() else {
                continue;
            };
            // Each primitive's indices count from its own first vertex.
            let first = positions.len();
            positions.extend(read.map(|p| Point3::from(p).cast::<f64>()));
            if primitive.mode() != Mode::Triangles {
                continue;
            }
            // Without indices, every three vertices in order are a triangle.
            let indices: Vec<usize> = match reader.read_indices() {
                Some(read) => read.into_u32().map(|i| first + i as usize).collect(),
                None => (first..positions.len()).collect(),
            };
            triangles.extend(indices.chunks_exact(3).map(|t| [t[0], t[1], t[2]]));
        }
    }

    AssetNode {
        name: node.name().unwrap_or_default().to_string(),
        local: Matrix4::from(node.transform().matrix()).cast::<f64>(),
        positions,
        triangles,
        children: node
            .children()
            .map(|child| read_node(&child, blob))
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::layout::geometry::PrefabGeometry;

    use approx::assert_abs_diff_eq;
    use nalgebra::Vector3;

    const TOLERANCE_M: f64 = 1e-5;

    /// Reads and derives one of the repository's exported assets.
    fn derive_asset(relative: &str) -> PrefabGeometry {
        let path = crate::asset_root().join(relative);
        let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        let gltf = gltf::Gltf::from_slice(&bytes).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        let roots = read_scene(&gltf).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        PrefabGeometry::derive(&roots).unwrap_or_else(|e| panic!("{path:?}: {e:?}"))
    }

    #[test]
    fn exported_crate_is_a_one_metre_cube_standing_on_its_origin() {
        // Exported with its scale unapplied (a 2 m mesh under a 0.5 scale),
        // so this also checks node transforms are composed. Its box is its
        // collider, and must be exact: the box is a ground-truth label.
        let crate_1m = derive_asset("objects/crate_1m.glb");

        let bounds = crate_1m.bounds();
        assert_abs_diff_eq!(
            bounds.half_extents,
            Vector3::new(0.5, 0.5, 0.5),
            epsilon = TOLERANCE_M
        );
        assert_abs_diff_eq!(
            bounds.centre.coords,
            Vector3::new(0.0, 0.0, 0.5),
            epsilon = TOLERANCE_M
        );
    }

    #[test]
    fn exported_cone_stands_on_its_origin_with_a_hull_collider() {
        let cone = derive_asset("objects/traffic_cone.glb");

        let bounds = cone.bounds();
        let base = bounds.centre.z - bounds.half_extents.z;
        assert_abs_diff_eq!(base, 0.0, epsilon = TOLERANCE_M);
        assert_abs_diff_eq!(bounds.centre.x, 0.0, epsilon = TOLERANCE_M);
        assert_abs_diff_eq!(bounds.centre.y, 0.0, epsilon = TOLERANCE_M);
        assert!(
            bounds.half_extents.z > bounds.half_extents.x,
            "a cone is taller than it is wide: {bounds:?}"
        );
        assert!(cone.collider(&Vector3::new(1.0, 1.0, 1.0)).is_ok());
        // The plate and the cone are separate parts: one hull over both
        // would fill the gap round the cone.
        assert!(cone.to_string().ends_with("2 hulls"), "{cone}");
    }

    #[test]
    fn exported_meshes_carry_their_triangles() {
        // The concave-part check reads triangles; without them an extruded
        // concave part would pass.
        let path = crate::asset_root().join("objects/traffic_cone.glb");
        let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        let gltf = gltf::Gltf::from_slice(&bytes).unwrap_or_else(|e| panic!("{path:?}: {e}"));

        let roots = read_scene(&gltf).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        for node in &roots {
            assert!(
                !node.triangles.is_empty(),
                "`{}` has no triangles",
                node.name
            );
            assert!(
                node.triangles
                    .iter()
                    .flatten()
                    .all(|&i| i < node.positions.len()),
                "`{}` indexes past its vertices",
                node.name
            );
        }
    }
}
