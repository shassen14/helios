use super::*;

use approx::assert_abs_diff_eq;
use avian3d::prelude::SimpleCollider;
use nalgebra::{Translation3, UnitQuaternion};
use std::f64::consts::FRAC_PI_4;

const TOLERANCE_M: f64 = 1e-9;
const COLLIDER_TOLERANCE_M: f32 = 1e-5;

// Meshes here are written in glTF axes, as a file stores them: +Y is up, so
// an object's forward, left, up (x, y, z) is glTF (x, z, −y) the other way
// round, glTF (x, y, z) is object (x, −z, y).

/// A leaf node with a mesh.
fn node(name: &str, local: Matrix4<f64>, positions: Vec<Point3<f64>>) -> AssetNode {
    AssetNode {
        name: name.to_string(),
        local,
        positions,
        children: Vec::new(),
    }
}

/// A node with no mesh, only children.
fn group(local: Matrix4<f64>, children: Vec<AssetNode>) -> AssetNode {
    AssetNode {
        name: String::new(),
        local,
        positions: Vec::new(),
        children,
    }
}

/// The eight corners of a box, in glTF axes.
fn cuboid(min: [f64; 3], max: [f64; 3]) -> Vec<Point3<f64>> {
    let mut corners = Vec::with_capacity(8);
    for x in [min[0], max[0]] {
        for y in [min[1], max[1]] {
            for z in [min[2], max[2]] {
                corners.push(Point3::new(x, y, z));
            }
        }
    }
    corners
}

/// A 1 m cube standing on its origin, in glTF axes (y is up).
fn cube_on_origin() -> Vec<Point3<f64>> {
    cuboid([-0.5, 0.0, -0.5], [0.5, 1.0, 0.5])
}

fn translation(x: f64, y: f64, z: f64) -> Matrix4<f64> {
    Translation3::new(x, y, z).to_homogeneous()
}

/// A rotation about glTF +Y, which is the object's up axis.
fn turn_about_up(angle: f64) -> Matrix4<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle).to_homogeneous()
}

fn derive(roots: &[AssetNode]) -> PrefabGeometry {
    PrefabGeometry::derive(roots).unwrap_or_else(|e| panic!("geometry should derive: {e:?}"))
}

fn derive_errors(roots: &[AssetNode]) -> Vec<GeometryError> {
    match PrefabGeometry::derive(roots) {
        Ok(geometry) => panic!("geometry should be rejected: {geometry:?}"),
        Err(errors) => errors,
    }
}

/// The collider's box in the entity's Bevy frame, as (min, max).
fn collider_box(geometry: &PrefabGeometry, scale: [f64; 3]) -> (Vec3, Vec3) {
    let collider = geometry
        .collider(&Vector3::from(scale))
        .expect("collider builds");
    let aabb = collider.aabb(Vec3::ZERO, Quat::IDENTITY);
    (aabb.min, aabb.max)
}

fn assert_vec3_eq(actual: Vec3, expected: [f32; 3]) {
    let expected = Vec3::from(expected);
    assert!(
        (actual - expected).abs().max_element() < COLLIDER_TOLERANCE_M,
        "{actual} != {expected}"
    );
}

#[test]
fn cube_on_its_origin_has_its_centre_half_up() {
    let geometry = derive(&[node("crate", Matrix4::identity(), cube_on_origin())]);

    assert_abs_diff_eq!(
        geometry.bounds().centre.coords,
        Vector3::new(0.0, 0.0, 0.5),
        epsilon = TOLERANCE_M
    );
    assert_abs_diff_eq!(
        geometry.bounds().half_extents,
        Vector3::new(0.5, 0.5, 0.5),
        epsilon = TOLERANCE_M
    );
}

#[test]
fn node_transform_is_applied() {
    // The exported crate's shape: a 2 m mesh scaled by half and lifted half
    // a metre, with the transform left unapplied in Blender. The old loader
    // read the mesh alone and got a 2 m cube through the ground.
    let local = translation(0.0, 0.5, 0.0) * Matrix4::new_scaling(0.5);
    let mesh = cuboid([-1.0; 3], [1.0; 3]);
    let geometry = derive(&[node("crate", local, mesh)]);

    assert_abs_diff_eq!(
        geometry.bounds().centre.coords,
        Vector3::new(0.0, 0.0, 0.5),
        epsilon = TOLERANCE_M
    );
    assert_abs_diff_eq!(
        geometry.bounds().half_extents,
        Vector3::new(0.5, 0.5, 0.5),
        epsilon = TOLERANCE_M
    );
}

#[test]
fn node_rotation_turns_the_box() {
    // A plank 2 m along forward, turned 90° left about up, lies along left.
    let plank = cuboid([-1.0, -0.1, -0.1], [1.0, 0.1, 0.1]);
    let geometry = derive(&[node("plank", turn_about_up(FRAC_PI_4 * 2.0), plank)]);

    assert_abs_diff_eq!(
        geometry.bounds().half_extents,
        Vector3::new(0.1, 1.0, 0.1),
        epsilon = TOLERANCE_M
    );
}

#[test]
fn nested_transforms_compose() {
    // One metre forward from a parent one metre forward: two metres.
    let child = node("crate", translation(1.0, 0.0, 0.0), cube_on_origin());
    let geometry = derive(&[group(translation(1.0, 0.0, 0.0), vec![child])]);

    assert_abs_diff_eq!(
        geometry.bounds().centre.coords,
        Vector3::new(2.0, 0.0, 0.5),
        epsilon = TOLERANCE_M
    );
}

#[test]
fn rotated_mesh_keeps_a_tight_box() {
    // An octagon turned 45° lands on itself, so its box stays ±1. Boxing
    // the mesh first and turning the box would give ±√2, and the box is a
    // ground-truth label.
    let octagon: Vec<Point3<f64>> = (0..8)
        .flat_map(|k| {
            let angle = f64::from(k) * FRAC_PI_4;
            [0.0, 1.0].map(|height| Point3::new(angle.cos(), height, angle.sin()))
        })
        .collect();
    let geometry = derive(&[node("pillar", turn_about_up(FRAC_PI_4), octagon)]);

    assert_abs_diff_eq!(
        geometry.bounds().half_extents,
        Vector3::new(1.0, 1.0, 0.5),
        epsilon = TOLERANCE_M
    );
}

#[test]
fn asset_without_parts_collides_as_its_box() {
    // The box collider sits on the box centre, not on the origin: a crate's
    // collider spans 0 to 1 m up (Bevy's y), not −0.5 to 0.5.
    let geometry = derive(&[node("crate", Matrix4::identity(), cube_on_origin())]);

    let (min, max) = collider_box(&geometry, [1.0; 3]);
    assert_vec3_eq(min, [-0.5, 0.0, -0.5]);
    assert_vec3_eq(max, [0.5, 1.0, 0.5]);
}

#[test]
fn box_collider_scales_along_the_objects_axes() {
    // Object up (z) is Bevy's y; object left (y) is Bevy's −z.
    let geometry = derive(&[node("crate", Matrix4::identity(), cube_on_origin())]);

    let (min, max) = collider_box(&geometry, [1.0, 2.0, 3.0]);
    assert_vec3_eq(min, [-0.5, 0.0, -1.0]);
    assert_vec3_eq(max, [0.5, 3.0, 1.0]);
}

#[test]
fn collider_parts_are_hulled_in_place() {
    // A part moved by its node is hulled where the node puts it; linked
    // duplicates (one mesh, two nodes) give one hull each.
    let part = cuboid([-0.5, 0.0, -0.5], [0.5, 0.2, 0.5]);
    let geometry = derive(&[
        node("frame", Matrix4::identity(), cube_on_origin()),
        node("col_foot", translation(-2.0, 0.0, 0.0), part.clone()),
        node("col_foot.001", translation(2.0, 0.0, 0.0), part),
    ]);

    let (min, max) = collider_box(&geometry, [1.0; 3]);
    assert_vec3_eq(min, [-2.5, 0.0, -0.5]);
    assert_vec3_eq(max, [2.5, 0.2, 0.5]);
}

#[test]
fn rotated_part_scales_exactly() {
    // A cube part turned 45° about up, stretched 2× along forward. Scaling
    // its vertices gives a box ±√2 forward, ±√½ left; scaling the part in
    // its own turned frame, as Avian would, gives a different shape.
    let part = cuboid([-0.5, 0.0, -0.5], [0.5, 1.0, 0.5]);
    let geometry = derive(&[
        node("post", Matrix4::identity(), cube_on_origin()),
        node("col_post", turn_about_up(FRAC_PI_4), part),
    ]);

    let half_diagonal = std::f32::consts::FRAC_1_SQRT_2;
    let (min, max) = collider_box(&geometry, [2.0, 1.0, 1.0]);
    assert_vec3_eq(min, [-2.0 * half_diagonal, 0.0, -half_diagonal]);
    assert_vec3_eq(max, [2.0 * half_diagonal, 1.0, half_diagonal]);
}

#[test]
fn bounds_scale_about_the_origin() {
    let geometry = derive(&[node("crate", Matrix4::identity(), cube_on_origin())]);

    let scaled = geometry.bounds().scaled(&Vector3::new(1.0, 2.0, 3.0));
    assert_abs_diff_eq!(
        scaled.centre.coords,
        Vector3::new(0.0, 0.0, 1.5),
        epsilon = TOLERANCE_M
    );
    assert_abs_diff_eq!(
        scaled.half_extents,
        Vector3::new(0.5, 1.0, 1.5),
        epsilon = TOLERANCE_M
    );
}

#[test]
fn flat_asset_without_parts_is_rejected() {
    // A plane's box has no height, so it cannot stand in as a collider.
    let plane = cuboid([-1.0, 0.0, -1.0], [1.0, 0.0, 1.0]);

    let errors = derive_errors(&[node("decal", Matrix4::identity(), plane)]);
    let [GeometryError::FlatWithoutColliderParts { size }] = errors.as_slice() else {
        panic!("expected a flat-asset error: {errors:?}");
    };
    assert_abs_diff_eq!(
        Vector3::from(*size),
        Vector3::new(2.0, 2.0, 0.0),
        epsilon = TOLERANCE_M
    );
}

#[test]
fn flat_visual_with_parts_is_accepted() {
    // A sign plate is flat, but its collider part is not.
    let plate = cuboid([-0.5, 0.0, 0.0], [0.5, 1.0, 0.0]);
    let geometry = derive(&[
        node("plate", Matrix4::identity(), plate),
        node("col_plate", Matrix4::identity(), cube_on_origin()),
    ]);

    assert_abs_diff_eq!(geometry.bounds().half_extents.y, 0.0, epsilon = TOLERANCE_M);
}

#[test]
fn flat_collider_part_is_rejected_by_name() {
    let flat = cuboid([-0.5, 0.0, -0.5], [0.5, 0.0, 0.5]);

    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("col_base", Matrix4::identity(), flat),
    ]);
    assert_eq!(
        errors,
        [GeometryError::FlatColliderPart {
            part: "col_base".to_string()
        }]
    );
}

#[test]
fn collinear_collider_part_is_rejected() {
    let line = vec![
        Point3::origin(),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];

    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("col_rail", Matrix4::identity(), line),
    ]);
    assert_eq!(
        errors,
        [GeometryError::FlatColliderPart {
            part: "col_rail".to_string()
        }]
    );
}

#[test]
fn mirrored_node_is_rejected() {
    let mirror = Matrix4::new_nonuniform_scaling(&Vector3::new(-1.0, 1.0, 1.0));

    let errors = derive_errors(&[node("crate", mirror, cube_on_origin())]);
    assert_eq!(
        errors,
        [
            GeometryError::MirroredNode {
                node: "crate".to_string()
            },
            GeometryError::NoVisualGeometry,
        ]
    );
}

#[test]
fn mirror_inherited_from_a_parent_is_rejected() {
    let mirror = Matrix4::new_nonuniform_scaling(&Vector3::new(1.0, 1.0, -1.0));
    let child = node("crate", Matrix4::identity(), cube_on_origin());

    let errors = derive_errors(&[group(mirror, vec![child])]);
    assert!(
        errors.contains(&GeometryError::MirroredNode {
            node: "crate".to_string()
        }),
        "{errors:?}"
    );
}

#[test]
fn miscased_collider_prefix_is_rejected() {
    // `Col_cone` would otherwise be drawn as a second cone and never collide.
    let errors = derive_errors(&[
        node("cone", Matrix4::identity(), cube_on_origin()),
        node("Col_cone", Matrix4::identity(), cube_on_origin()),
        node("COL_base", Matrix4::identity(), cube_on_origin()),
    ]);
    assert_eq!(
        errors,
        [
            GeometryError::MiscasedColliderPrefix {
                node: "Col_cone".to_string()
            },
            GeometryError::MiscasedColliderPrefix {
                node: "COL_base".to_string()
            },
        ]
    );
}

#[test]
fn names_merely_starting_with_col_are_visual() {
    let geometry = derive(&[node("collar", Matrix4::identity(), cube_on_origin())]);

    assert_abs_diff_eq!(
        geometry.bounds().half_extents,
        Vector3::new(0.5, 0.5, 0.5),
        epsilon = TOLERANCE_M
    );
}

#[test]
fn asset_of_only_collider_parts_is_rejected() {
    let errors = derive_errors(&[node("col_box", Matrix4::identity(), cube_on_origin())]);

    assert_eq!(errors, [GeometryError::NoVisualGeometry]);
}

#[test]
fn empty_scene_is_rejected() {
    let errors = derive_errors(&[group(Matrix4::identity(), Vec::new())]);

    assert_eq!(errors, [GeometryError::NoVisualGeometry]);
}

#[test]
fn non_finite_vertex_is_rejected() {
    let mut mesh = cube_on_origin();
    mesh[0].x = f64::NAN;

    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("broken", Matrix4::identity(), mesh),
    ]);
    assert_eq!(
        errors,
        [GeometryError::NotFinite {
            node: "broken".to_string()
        }]
    );
}

#[test]
fn every_problem_is_reported_at_once() {
    let flat = cuboid([-0.5, 0.0, -0.5], [0.5, 0.0, 0.5]);
    let mirror = Matrix4::new_nonuniform_scaling(&Vector3::new(-1.0, 1.0, 1.0));

    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("mirrored", mirror, cube_on_origin()),
        node("Col_typo", Matrix4::identity(), cube_on_origin()),
        node("col_flat", Matrix4::identity(), flat),
    ]);
    assert_eq!(errors.len(), 3, "{errors:?}");
}
