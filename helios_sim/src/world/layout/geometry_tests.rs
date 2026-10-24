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
        triangles: Vec::new(),
        children: Vec::new(),
    }
}

/// A node with no mesh, only children.
fn group(local: Matrix4<f64>, children: Vec<AssetNode>) -> AssetNode {
    AssetNode {
        name: String::new(),
        local,
        positions: Vec::new(),
        triangles: Vec::new(),
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
    // ground-truth label. An octagon fills too little of its box to collide
    // as it, so it carries its own collider part.
    let octagon: Vec<Point3<f64>> = (0..8)
        .flat_map(|k| {
            let angle = f64::from(k) * FRAC_PI_4;
            [0.0, 1.0].map(|height| Point3::new(angle.cos(), height, angle.sin()))
        })
        .collect();
    let geometry = derive(&[
        node("pillar", turn_about_up(FRAC_PI_4), octagon.clone()),
        node("col_pillar", turn_about_up(FRAC_PI_4), octagon),
    ]);

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
    // duplicates (one mesh, two nodes) give one hull each. The frame spans
    // both feet, so they lie within the visual.
    let part = cuboid([-0.5, 0.0, -0.5], [0.5, 0.2, 0.5]);
    let geometry = derive(&[
        node(
            "frame",
            Matrix4::identity(),
            cuboid([-2.5, 0.0, -0.5], [2.5, 1.0, 0.5]),
        ),
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
    // its own turned frame, as Avian would, gives a different shape. The
    // post is turned with its part, so the part lies within the visual.
    let part = cuboid([-0.5, 0.0, -0.5], [0.5, 1.0, 0.5]);
    let geometry = derive(&[
        node("post", turn_about_up(FRAC_PI_4), part.clone()),
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
    // A sign plate is flat, but its collider part is not: a slab thin
    // enough to stay within the overhang tolerance either side of it.
    let half_thickness = MAX_PART_OVERHANG_M / 2.0;
    let plate = cuboid([-0.5, 0.0, 0.0], [0.5, 1.0, 0.0]);
    let slab = cuboid([-0.5, 0.0, -half_thickness], [0.5, 1.0, half_thickness]);
    let geometry = derive(&[
        node("plate", Matrix4::identity(), plate),
        node("col_plate", Matrix4::identity(), slab),
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
        Point3::new(-0.5, 0.0, 0.0),
        Point3::origin(),
        Point3::new(0.5, 0.0, 0.0),
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

/// The 1 m cube on its origin grown by `margin` on every face.
fn cube_grown_by(margin: f64) -> Vec<Point3<f64>> {
    cuboid(
        [-0.5 - margin, -margin, -0.5 - margin],
        [0.5 + margin, 1.0 + margin, 0.5 + margin],
    )
}

/// The one error expected, as the part it names and how far it reaches out.
fn only_overhang(errors: &[GeometryError]) -> (&str, f64) {
    let [GeometryError::ColliderPartOutsideVisual { part, overhang_m }] = errors else {
        panic!("expected one collider-part-outside-visual error: {errors:?}");
    };
    (part, *overhang_m)
}

#[test]
fn moved_collider_part_is_rejected_by_name() {
    // A part dragged 1 m forward in Blender would collide in empty air,
    // and parts are never drawn, so nothing else would show it.
    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("col_box", translation(1.0, 0.0, 0.0), cube_on_origin()),
    ]);

    let (part, overhang) = only_overhang(&errors);
    assert_eq!(part, "col_box");
    assert_abs_diff_eq!(overhang, 1.0, epsilon = TOLERANCE_M);
}

#[test]
fn part_sunk_below_the_visual_is_rejected() {
    // Down is glTF −y: the bottom face, checked from the low side.
    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("col_box", translation(0.0, -0.5, 0.0), cube_on_origin()),
    ]);

    let (_, overhang) = only_overhang(&errors);
    assert_abs_diff_eq!(overhang, 0.5, epsilon = TOLERANCE_M);
}

#[test]
fn part_within_the_overhang_tolerance_is_accepted() {
    // Slop from placing a part by hand.
    derive(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node(
            "col_box",
            Matrix4::identity(),
            cube_grown_by(MAX_PART_OVERHANG_M / 2.0),
        ),
    ]);
}

#[test]
fn part_past_the_overhang_tolerance_is_rejected() {
    let margin = MAX_PART_OVERHANG_M * 2.0;

    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("col_box", Matrix4::identity(), cube_grown_by(margin)),
    ]);

    let (_, overhang) = only_overhang(&errors);
    assert_abs_diff_eq!(overhang, margin, epsilon = TOLERANCE_M);
}

#[test]
fn only_the_moved_part_is_named() {
    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("col_kept", Matrix4::identity(), cube_on_origin()),
        node("col_moved", translation(0.0, 0.0, 2.0), cube_on_origin()),
    ]);

    let (part, _) = only_overhang(&errors);
    assert_eq!(part, "col_moved");
}

/// A cone 1 m tall on a 16-sided base of radius 0.5, standing on its origin,
/// in glTF axes, and the fraction of its 1 m box it fills.
fn cone() -> (Vec<Point3<f64>>, f64) {
    const SIDES: u32 = 16;
    const RADIUS: f64 = 0.5;
    let mut points: Vec<Point3<f64>> = (0..SIDES)
        .map(|k| {
            let angle = f64::from(k) * std::f64::consts::TAU / f64::from(SIDES);
            Point3::new(RADIUS * angle.cos(), 0.0, RADIUS * angle.sin())
        })
        .collect();
    points.push(Point3::new(0.0, 1.0, 0.0));
    // Base polygon area × height ÷ 3, over a 1 × 1 × 1 box.
    let sides = f64::from(SIDES);
    let base_area = sides / 2.0 * RADIUS * RADIUS * (std::f64::consts::TAU / sides).sin();
    (points, base_area / 3.0)
}

/// The hull is built in f32, so its volume is good to about this.
const FILL_TOLERANCE: f64 = 1e-4;

#[test]
fn cone_without_parts_is_rejected_with_its_fill() {
    // About π/12 of its box: a box collider would be mostly empty air.
    let (cone, expected) = cone();

    let errors = derive_errors(&[node("cone", Matrix4::identity(), cone)]);
    let [GeometryError::NotBoxShaped { fill }] = errors.as_slice() else {
        panic!("expected a not-box-shaped error: {errors:?}");
    };
    assert_abs_diff_eq!(*fill, expected, epsilon = FILL_TOLERANCE);
}

#[test]
fn cone_with_a_collider_part_is_accepted() {
    // Parts give the real shape, so how much of the box it fills is moot.
    let (cone, _) = cone();

    derive(&[
        node("cone", Matrix4::identity(), cone.clone()),
        node("col_cone", Matrix4::identity(), cone),
    ]);
}

#[test]
fn l_shape_without_parts_is_rejected() {
    // A 2 × 1 × 1 base with a 1 m cube on one end fills ¾ of its 2 × 2 × 1
    // box; its hull adds the slope between them, filling 7/8. Close to a
    // box, still not one.
    let mut l_shape = cuboid([0.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    l_shape.extend(cuboid([0.0, 1.0, 0.0], [1.0, 2.0, 1.0]));

    let errors = derive_errors(&[node("step", Matrix4::identity(), l_shape)]);
    let [GeometryError::NotBoxShaped { fill }] = errors.as_slice() else {
        panic!("expected a not-box-shaped error: {errors:?}");
    };
    assert_abs_diff_eq!(*fill, 0.875, epsilon = FILL_TOLERANCE);
}

#[test]
fn crate_with_bevelled_edges_is_accepted() {
    // A 1 m cube with every edge cut 5 cm in, as a modelled crate's are:
    // about 98% of its box, comfortably a box.
    const BEVEL_M: f64 = 0.05;
    let inset = |v: f64| if v == 0.0 { BEVEL_M } else { 1.0 - BEVEL_M };
    let mut bevelled = Vec::new();
    for corner in cuboid([0.0; 3], [1.0; 3]) {
        // Each corner becomes three points, each moved in along two axes,
        // which cuts the corner's three edges.
        let [x, y, z] = [corner.x, corner.y, corner.z];
        bevelled.push(Point3::new(x, inset(y), inset(z)));
        bevelled.push(Point3::new(inset(x), y, inset(z)));
        bevelled.push(Point3::new(inset(x), inset(y), z));
    }

    derive(&[node("crate", Matrix4::identity(), bevelled)]);
}

/// A leaf node whose mesh has triangles.
fn solid(name: &str, (positions, triangles): (Vec<Point3<f64>>, Vec<[usize; 3]>)) -> AssetNode {
    AssetNode {
        triangles,
        ..node(name, Matrix4::identity(), positions)
    }
}

/// A closed mesh: `profile`, a polygon on the ground in glTF (x, z), pulled
/// straight up to `height`. Each cap is a fan from the profile's first
/// corner, so every corner must be visible from it (an L seen from its
/// outer corner is). Each side is two triangles.
fn prism(profile: &[[f64; 2]], height: f64) -> (Vec<Point3<f64>>, Vec<[usize; 3]>) {
    let n = profile.len();
    let positions = [0.0, height]
        .iter()
        .flat_map(|&y| profile.iter().map(move |&[x, z]| Point3::new(x, y, z)))
        .collect();
    let mut triangles = Vec::new();
    for k in 1..n - 1 {
        triangles.push([0, k, k + 1]);
        triangles.push([n, n + k + 1, n + k]);
    }
    for k in 0..n {
        let next = (k + 1) % n;
        triangles.push([k, next, n + next]);
        triangles.push([k, n + next, n + k]);
    }
    (positions, triangles)
}

/// A 2 × 2 m L, 1 m arm width, as seen from above: its outer corner at the
/// origin.
const L_PROFILE: [[f64; 2]; 6] = [
    [0.0, 0.0],
    [2.0, 0.0],
    [2.0, 1.0],
    [1.0, 1.0],
    [1.0, 2.0],
    [0.0, 2.0],
];

/// The one error expected, as the part it names and how deep its dent is.
fn only_dent(errors: &[GeometryError]) -> (&str, f64) {
    let [GeometryError::ConcaveColliderPart { part, dent_m }] = errors else {
        panic!("expected one concave-collider-part error: {errors:?}");
    };
    (part, *dent_m)
}

#[test]
fn concave_part_is_rejected_by_name() {
    // One L-shaped part is hulled into a full slab with one corner cut off,
    // colliding across the empty inside of the L.
    let errors = derive_errors(&[
        solid("step", prism(&L_PROFILE, 1.0)),
        solid("col_step", prism(&L_PROFILE, 1.0)),
    ]);

    let (part, dent) = only_dent(&errors);
    assert_eq!(part, "col_step");
    assert!(dent > MAX_PART_DENT_M, "{dent}");
}

#[test]
fn concave_prism_is_caught_by_its_walls_not_its_vertices() {
    // Every vertex of an extruded L lies on its top or bottom face, so a
    // vertex-only measure reads no dent. The inner walls' triangles do not.
    let (positions, triangles) = prism(&L_PROFILE, 1.0);

    assert_abs_diff_eq!(deepest_dent(&positions, &[]), 0.0, epsilon = 1e-6);
    assert!(deepest_dent(&positions, &triangles) > MAX_PART_DENT_M);
}

#[test]
fn concave_part_split_into_convex_parts_is_accepted() {
    // The same L as two boxes, each convex, overlapping where they meet.
    derive(&[
        solid("step", prism(&L_PROFILE, 1.0)),
        solid(
            "col_long",
            prism(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]], 1.0),
        ),
        solid(
            "col_short",
            prism(&[[0.0, 0.0], [1.0, 0.0], [1.0, 2.0], [0.0, 2.0]], 1.0),
        ),
    ]);
}

#[test]
fn convex_part_with_triangles_is_accepted() {
    // A hexagonal column: every triangle, caps and sides, lies in a face of
    // its hull.
    let hexagon: Vec<[f64; 2]> = (0..6)
        .map(|k| {
            let angle = f64::from(k) * std::f64::consts::TAU / 6.0;
            [0.5 * angle.cos(), 0.5 * angle.sin()]
        })
        .collect();

    derive(&[
        solid("column", prism(&hexagon, 2.0)),
        solid("col_column", prism(&hexagon, 2.0)),
    ]);
}

#[test]
fn vertex_inside_a_part_is_rejected_with_its_depth() {
    // A cube with a stray vertex at its centre, half a metre from every
    // face.
    let mut part = cube_on_origin();
    part.push(Point3::new(0.0, 0.5, 0.0));

    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("col_crate", Matrix4::identity(), part),
    ]);

    let (_, dent) = only_dent(&errors);
    assert_abs_diff_eq!(dent, 0.5, epsilon = 1e-6);
}

#[test]
fn vertices_on_a_parts_faces_are_not_dents() {
    // A cube subdivided once: its face and edge midpoints lie on its hull.
    let mut part = cube_on_origin();
    part.extend([
        Point3::new(0.5, 0.5, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.5, -0.5),
        Point3::new(0.5, 1.0, 0.5),
    ]);

    derive(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        node("col_crate", Matrix4::identity(), part),
    ]);
}

/// The shipped traffic cone's shape: a cone of radius 0.16 m on a 16-sided
/// base, its tip 0.7 m up, standing on a 0.4 m square plate `PLATE_M`
/// thick. Returns (plate, cone), in glTF axes.
fn cone_on_a_plate() -> (Vec<Point3<f64>>, Vec<Point3<f64>>) {
    const SIDES: u32 = 16;
    let plate = cuboid([-0.2, 0.0, -0.2], [0.2, PLATE_M, 0.2]);
    let mut cone: Vec<Point3<f64>> = (0..SIDES)
        .map(|k| {
            let angle = f64::from(k) * std::f64::consts::TAU / f64::from(SIDES);
            Point3::new(0.16 * angle.cos(), PLATE_M, 0.16 * angle.sin())
        })
        .collect();
    cone.push(Point3::new(0.0, 0.7, 0.0));
    (plate, cone)
}

const PLATE_M: f64 = 0.03;

#[test]
fn cone_and_plate_as_one_part_is_rejected() {
    // One hull over both is a pyramid from the plate's corners to the tip;
    // the cone's base ring lies inside it, a plate's thickness above its
    // floor, and the gap round the cone collides.
    let (plate, cone) = cone_on_a_plate();
    let both: Vec<_> = plate.iter().chain(&cone).copied().collect();

    let errors = derive_errors(&[
        node("traffic_cone", Matrix4::identity(), both.clone()),
        node("col_cone", Matrix4::identity(), both),
    ]);

    let (_, dent) = only_dent(&errors);
    assert_abs_diff_eq!(dent, PLATE_M, epsilon = 1e-5);
}

#[test]
fn cone_and_plate_as_two_parts_is_accepted() {
    let (plate, cone) = cone_on_a_plate();
    let both: Vec<_> = plate.iter().chain(&cone).copied().collect();

    derive(&[
        node("traffic_cone", Matrix4::identity(), both),
        node("col_base", Matrix4::identity(), plate),
        node("col_cone", Matrix4::identity(), cone),
    ]);
}

#[test]
fn triangle_naming_a_missing_vertex_is_rejected() {
    // A cube has vertices 0 to 7.
    let errors = derive_errors(&[
        node("crate", Matrix4::identity(), cube_on_origin()),
        AssetNode {
            triangles: vec![[0, 1, 8]],
            ..node("col_crate", Matrix4::identity(), cube_on_origin())
        },
    ]);

    assert_eq!(
        errors,
        [GeometryError::TriangleOutOfRange {
            node: "col_crate".to_string()
        }]
    );
}

#[test]
fn summary_gives_size_base_and_collider() {
    let boxed = derive(&[node("crate", Matrix4::identity(), cube_on_origin())]);
    assert_eq!(
        boxed.to_string(),
        "1.00 x 1.00 x 1.00 m, base at z 0 mm, box collider"
    );

    let (plate, cone) = cone_on_a_plate();
    let both: Vec<_> = plate.iter().chain(&cone).copied().collect();
    let hulled = derive(&[
        node("traffic_cone", Matrix4::identity(), both),
        node("col_base", Matrix4::identity(), plate),
        node("col_cone", Matrix4::identity(), cone),
    ]);
    assert_eq!(
        hulled.to_string(),
        "0.40 x 0.40 x 0.70 m, base at z 0 mm, 2 hulls"
    );
}
