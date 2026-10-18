//! What a prefab's `.glb` means physically: its bounding box (a ground-truth
//! label) and its collider, derived once per prefab from the glTF node tree.
//!
//! Every node's transform is composed down from the scene root, so where a
//! part was left in Blender (unapplied transforms, origins, parenting) does
//! not matter. Nodes named `col_*` are collider parts, each convex, hulled
//! on its own and kept within the visual geometry's box; every other node is
//! visual. An asset without parts collides as its bounding box.
//!
//! Geometry is kept in the **object's axes**: forward +X, left +Y, up +Z, as
//! authored in Blender. glTF's axes are Bevy's (Y up), and Blender's +Y-up
//! export maps object axes onto them with the same relabel as ENU onto Bevy,
//! so object-axis values cross the bridge as ENU-framed ones. The spawner
//! poses objects with an ENU-to-ENU transform for the same reason.

use crate::core::components::BoundingBox3D;
use crate::core::transforms::{per_axis_to_bevy, point_bevy_to_vec3, Bevy, FromBevy, ToBevy};

use helios_core::spatial::conventions::Enu;
use helios_core::spatial::quantities::Point;

use avian3d::prelude::{Collider, ComputeMassProperties3d};
use bevy::prelude::{Quat, Vec3};
use nalgebra::{Matrix4, Point3, Vector3};
use std::fmt;

/// Name prefix marking a node as a collider part.
const COLLIDER_PREFIX: &str = "col_";

/// Smallest size, in meters, a collider may have in any direction; anything
/// thinner counts as flat and cannot be a solid.
const MIN_EXTENT_M: f64 = 1e-4;

/// Least fraction of its box an asset without collider parts must fill (its
/// convex hull's volume over the box's) for the box to stand in as its
/// collider. Boxes fill 1.0 and a bevelled crate about 0.98; an L-shape fills
/// 0.875, a cylinder 0.785 and a cone 0.26, so 0.9 passes every box and
/// rejects the rest.
const MIN_BOX_FILL: f64 = 0.9;

/// Farthest, in meters, a collider part may reach past the visual geometry's
/// box. A part sticking out is wrong for every agent, so this clears only
/// snapping slop. Absolute rather than a fraction of the box, which would
/// let a part on a 100 m slab wander a metre.
const MAX_PART_OVERHANG_M: f64 = 0.001;

/// Deepest, in meters, a collider part's vertex may sit inside the part's
/// own hull. A part is hulled, so a concave one silently fills its dents;
/// this clears only float noise on vertices lying on a face.
const MAX_PART_DENT_M: f64 = 0.001;

const MM_PER_M: f64 = 1000.0;

/// A prefab's derived geometry, shared by every placement of it.
#[derive(Debug, Clone)]
pub struct PrefabGeometry {
    bounds: ObjectBounds,
    collision: CollisionShape,
}

impl PrefabGeometry {
    /// Derives the geometry from a scene's root nodes, or returns every
    /// problem found.
    pub fn derive(roots: &[AssetNode]) -> Result<Self, Vec<GeometryError>> {
        let mut errors = Vec::new();
        let mut visual = Vec::new();
        let mut parts = Vec::new();

        let mut stack: Vec<(&AssetNode, Matrix4<f64>)> = roots
            .iter()
            .rev()
            .map(|node| (node, Matrix4::identity()))
            .collect();
        while let Some((node, parent_to_root)) = stack.pop() {
            let to_root = parent_to_root * node.local;
            stack.extend(node.children.iter().rev().map(|child| (child, to_root)));
            if node.positions.is_empty() {
                continue;
            }
            match node_points(node, &to_root) {
                Ok(points) => match is_collider_part(&node.name) {
                    Ok(true) => parts.push(ColliderPart {
                        name: node.name.clone(),
                        points,
                        triangles: node.triangles.clone(),
                    }),
                    Ok(false) => visual.extend(points),
                    Err(e) => errors.push(e),
                },
                Err(e) => errors.push(e),
            }
        }

        let bounds = ObjectBounds::around(&visual);
        if bounds.is_none() {
            errors.push(GeometryError::NoVisualGeometry);
        }
        for part in &parts {
            if !spans_volume(&part.points) {
                errors.push(GeometryError::FlatColliderPart {
                    part: part.name.clone(),
                });
                continue;
            }
            let dent = deepest_dent(&part.points, &part.triangles);
            if dent > MAX_PART_DENT_M {
                errors.push(GeometryError::ConcaveColliderPart {
                    part: part.name.clone(),
                    dent_m: dent,
                });
            }
        }
        if let Some(bounds) = &bounds {
            let size = bounds.half_extents * 2.0;
            if parts.is_empty() {
                if size.min() < MIN_EXTENT_M {
                    errors.push(GeometryError::FlatWithoutColliderParts { size: size.into() });
                } else {
                    let fill = box_fill(&visual, &size);
                    if fill < MIN_BOX_FILL {
                        errors.push(GeometryError::NotBoxShaped { fill });
                    }
                }
            } else {
                for part in &parts {
                    if let Some(part_box) = ObjectBounds::around(&part.points) {
                        let overhang = part_box.overhang_past(bounds);
                        if overhang > MAX_PART_OVERHANG_M {
                            errors.push(GeometryError::ColliderPartOutsideVisual {
                                part: part.name.clone(),
                                overhang_m: overhang,
                            });
                        }
                    }
                }
            }
        }

        match bounds {
            Some(bounds) if errors.is_empty() => Ok(Self {
                bounds,
                collision: if parts.is_empty() {
                    CollisionShape::Box
                } else {
                    CollisionShape::Hulls(parts)
                },
            }),
            _ => Err(errors),
        }
    }

    /// The tight box around the visual geometry, at unit scale.
    pub fn bounds(&self) -> &ObjectBounds {
        &self.bounds
    }

    /// Builds the collider for one placement scale (per axis of the object,
    /// every component positive), expressed in the placed entity's Bevy
    /// frame.
    ///
    /// Scale is applied to the vertices here and the hulls rebuilt, so the
    /// entity itself must not be scaled: Avian's own scaling of a hull is
    /// wrong for a non-uniform scale. Build once per distinct scale and
    /// share the result.
    pub fn collider(&self, scale: &Vector3<f64>) -> Result<Collider, GeometryError> {
        match &self.collision {
            CollisionShape::Box => {
                let bounds = self.bounds.scaled(scale);
                let size = per_axis_to_bevy::<Enu>(bounds.half_extents * 2.0);
                Ok(Collider::compound(vec![(
                    object_to_bevy(bounds.centre),
                    Quat::IDENTITY,
                    Collider::cuboid(size.x as f32, size.y as f32, size.z as f32),
                )]))
            }
            CollisionShape::Hulls(parts) => parts
                .iter()
                .map(|part| {
                    let points = part
                        .points
                        .iter()
                        .map(|p| object_to_bevy(Point3::from(p.coords.component_mul(scale))))
                        .collect();
                    Collider::convex_hull(points)
                        .map(|hull| (Vec3::ZERO, Quat::IDENTITY, hull))
                        .ok_or_else(|| GeometryError::FlatColliderPart {
                            part: part.name.clone(),
                        })
                })
                .collect::<Result<Vec<_>, _>>()
                .map(Collider::compound),
        }
    }
}

/// One line for the load log: size, base height and collider, e.g.
/// `0.38 x 0.38 x 0.72 m, base at z 0 mm, 2 hulls`. A wrong size or a
/// floating base shows here before a run.
impl fmt::Display for PrefabGeometry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let size = self.bounds.half_extents * 2.0;
        let base_m = self.bounds.centre.z - self.bounds.half_extents.z;
        // Adding zero turns the -0 that float noise rounds to into 0.
        let base_mm = (base_m * MM_PER_M).round() + 0.0;
        write!(
            f,
            "{:.2} x {:.2} x {:.2} m, base at z {base_mm} mm, ",
            size.x, size.y, size.z
        )?;
        match &self.collision {
            CollisionShape::Box => write!(f, "box collider"),
            CollisionShape::Hulls(parts) if parts.len() == 1 => write!(f, "1 hull"),
            CollisionShape::Hulls(parts) => write!(f, "{} hulls", parts.len()),
        }
    }
}

/// An axis-aligned box in the object's axes: the object's ground-truth
/// extent. The centre is not the origin in general: a crate standing on its
/// origin has its centre half its height up.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObjectBounds {
    /// Box centre in the object's axes, meters.
    pub centre: Point3<f64>,
    /// Half the box's size along each of the object's axes, meters.
    pub half_extents: Vector3<f64>,
}

impl ObjectBounds {
    /// The tightest box containing `points`; `None` when there are none.
    fn around(points: &[Point3<f64>]) -> Option<Self> {
        let (first, rest) = points.split_first()?;

        let (min, max) = rest
            .iter()
            .fold((first.coords, first.coords), |(min, max), p| {
                (min.inf(&p.coords), max.sup(&p.coords))
            });

        Some(Self {
            centre: Point3::from((min + max) / 2.0),
            half_extents: (max - min) / 2.0,
        })
    }

    /// How far this box reaches past `outer`, in meters: the largest
    /// overshoot over its six faces. Each face's overshoot is positive when
    /// this box sticks out through it, so zero or less means this box lies
    /// inside `outer`.
    fn overhang_past(&self, outer: &Self) -> f64 {
        let self_min = self.centre - self.half_extents;
        let self_max = self.centre + self.half_extents;
        let outer_min = outer.centre - outer.half_extents;
        let outer_max = outer.centre + outer.half_extents;

        let below = outer_min - self_min;
        let above = self_max - outer_max;

        below.max().max(above.max())
    }

    /// The box of the object scaled per axis about its origin.
    pub fn scaled(&self, scale: &Vector3<f64>) -> Self {
        Self {
            centre: Point3::from(self.centre.coords.component_mul(scale)),
            half_extents: self.half_extents.component_mul(scale),
        }
    }

    /// The same box in the placed entity's Bevy frame.
    pub fn in_bevy_frame(&self) -> BoundingBox3D {
        let half_extents = per_axis_to_bevy::<Enu>(self.half_extents).cast::<f32>();
        BoundingBox3D {
            centre: object_to_bevy(self.centre),
            half_extents: Vec3::new(half_extents.x, half_extents.y, half_extents.z),
        }
    }
}

/// How a prefab collides.
#[derive(Debug, Clone)]
enum CollisionShape {
    /// No collider parts: the bounding box itself.
    Box,
    /// One convex hull per `col_` node.
    Hulls(Vec<ColliderPart>),
}

/// One `col_` node's vertices, in the object's axes, and its triangles.
#[derive(Debug, Clone)]
struct ColliderPart {
    name: String,
    points: Vec<Point3<f64>>,
    /// Indices into `points`, checked in range.
    triangles: Vec<[usize; 3]>,
}

/// One node of an asset's scene, as read from its glTF: plain data, so the
/// geometry can be derived and tested without Bevy's asset server.
#[derive(Debug, Clone)]
pub struct AssetNode {
    /// The Blender object's name; unnamed nodes are empty.
    pub name: String,
    /// This node's frame in its parent's, in glTF axes; may scale.
    pub local: Matrix4<f64>,
    /// Vertex positions of the node's mesh in its own frame; empty for a
    /// node without one.
    pub positions: Vec<Point3<f64>>,
    /// The mesh's triangles, as three indices into `positions` each; empty
    /// for a node without a mesh, or whose mesh is only points or lines.
    pub triangles: Vec<[usize; 3]>,
    pub children: Vec<AssetNode>,
}

/// Why an asset's geometry cannot be used.
#[derive(Debug, Clone, PartialEq)]
pub enum GeometryError {
    /// The file holds no scene to place.
    NoScene,
    /// Geometry is stored outside the file; only self-contained `.glb`
    /// files are read.
    ExternalBuffer,
    /// A node's composed transform mirrors it, turning its mesh inside out.
    MirroredNode { node: String },
    /// A vertex position is NaN or infinite.
    NotFinite { node: String },
    /// A triangle names a vertex the node does not have: a malformed file.
    TriangleOutOfRange { node: String },
    /// A node looks like a collider part but its prefix is not lowercase
    /// `col_`, so it would silently be treated as visual.
    MiscasedColliderPrefix { node: String },
    /// Every mesh is a collider part, so there is nothing to see or label.
    NoVisualGeometry,
    /// A collider part lies in a plane (or a line), so it has no volume.
    FlatColliderPart { part: String },
    /// The asset has no collider parts and its box is flat, so the box
    /// cannot stand in for them.
    FlatWithoutColliderParts { size: [f64; 3] },
    /// The asset has no collider parts and fills too little of its box (a
    /// cone, a cylinder, a stray mesh) for the box to stand in for them.
    NotBoxShaped { fill: f64 },
    /// A collider part reaches past the visual geometry's box (a part moved
    /// or scaled away from its mesh in Blender), so it collides where
    /// nothing is drawn.
    ColliderPartOutsideVisual { part: String, overhang_m: f64 },
    /// A collider part is concave (one mesh where several convex parts were
    /// needed), so its hull fills the dent with collider where nothing is
    /// drawn.
    ConcaveColliderPart { part: String, dent_m: f64 },
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoScene => write!(f, "the file holds no scene"),
            Self::ExternalBuffer => write!(
                f,
                "geometry is stored outside the file; export a self-contained .glb"
            ),
            Self::MirroredNode { node } => write!(
                f,
                "node `{node}` is mirrored (negative scale); apply the scale in Blender"
            ),
            Self::NotFinite { node } => {
                write!(f, "node `{node}` has a vertex that is NaN or infinite")
            }
            Self::TriangleOutOfRange { node } => write!(
                f,
                "node `{node}` has a triangle naming a vertex it does not have; \
                 the file is malformed, re-export it"
            ),
            Self::MiscasedColliderPrefix { node } => write!(
                f,
                "node `{node}` would be a collider part, but the prefix must be \
                 lowercase `{COLLIDER_PREFIX}`"
            ),
            Self::NoVisualGeometry => write!(
                f,
                "every mesh is a `{COLLIDER_PREFIX}` collider part; nothing is visible"
            ),
            Self::FlatColliderPart { part } => {
                write!(f, "collider part `{part}` is flat and has no volume")
            }
            Self::FlatWithoutColliderParts { size } => write!(
                f,
                "the asset is flat ({:.4} x {:.4} x {:.4} m) and has no \
                 `{COLLIDER_PREFIX}` parts to collide with",
                size[0], size[1], size[2]
            ),
            Self::NotBoxShaped { fill } => write!(
                f,
                "the asset has no `{COLLIDER_PREFIX}` parts and fills {:.0}% of its box \
                 (at least {:.0}% needed for the box to be its collider); add \
                 `{COLLIDER_PREFIX}` parts for its real shape",
                fill * 100.0,
                MIN_BOX_FILL * 100.0
            ),
            Self::ColliderPartOutsideVisual { part, overhang_m } => write!(
                f,
                "collider part `{part}` sticks out {overhang_m:.3} m past the visible \
                 mesh; move it back over the mesh, or check it wasn't scaled"
            ),
            Self::ConcaveColliderPart { part, dent_m } => write!(
                f,
                "collider part `{part}` is concave: a vertex sits {dent_m:.3} m inside \
                 its hull, which fills that gap; split it into convex \
                 `{COLLIDER_PREFIX}` parts, or apply Mesh > Convex Hull if the hull is \
                 close enough"
            ),
        }
    }
}

impl std::error::Error for GeometryError {}

/// A mesh node's vertices, moved to the scene root and into object axes.
fn node_points(
    node: &AssetNode,
    to_root: &Matrix4<f64>,
) -> Result<Vec<Point3<f64>>, GeometryError> {
    if to_root.fixed_view::<3, 3>(0, 0).determinant() < 0.0 {
        return Err(GeometryError::MirroredNode {
            node: node.name.clone(),
        });
    }
    if node
        .positions
        .iter()
        .any(|p| p.coords.iter().any(|v| !v.is_finite()))
    {
        return Err(GeometryError::NotFinite {
            node: node.name.clone(),
        });
    }
    if node
        .triangles
        .iter()
        .flatten()
        .any(|&i| i >= node.positions.len())
    {
        return Err(GeometryError::TriangleOutOfRange {
            node: node.name.clone(),
        });
    }
    Ok(node
        .positions
        .iter()
        .map(|p| object_from_gltf(to_root.transform_point(p)))
        .collect())
}

/// Whether a node is a collider part. A name whose prefix matches only when
/// case is ignored is an error rather than a visual node.
fn is_collider_part(name: &str) -> Result<bool, GeometryError> {
    if is_collider_node(name) {
        return Ok(true);
    }
    let miscased = name
        .get(..COLLIDER_PREFIX.len())
        .is_some_and(|prefix| prefix.eq_ignore_ascii_case(COLLIDER_PREFIX));
    if miscased {
        return Err(GeometryError::MiscasedColliderPrefix {
            node: name.to_string(),
        });
    }
    Ok(false)
}

/// Whether a node named `name` is a collider part rather than something to
/// render.
pub(super) fn is_collider_node(name: &str) -> bool {
    name.starts_with(COLLIDER_PREFIX)
}

/// The fraction of a box of `size` that the convex hull of `points` fills.
/// Zero when the points have no hull, so degenerate geometry fails the
/// check rather than passing it.
fn box_fill(points: &[Point3<f64>], size: &Vector3<f64>) -> f64 {
    // At unit density a body's mass is its volume.
    let hull_volume = hull_of(points)
        .map(|hull| f64::from(hull.mass(1.0)))
        .unwrap_or(0.0);
    hull_volume / (size.x * size.y * size.z)
}

/// How deep, in meters, a mesh reaches inside its own convex hull: the
/// largest distance to the hull's surface of any vertex or any triangle's
/// centroid. Zero for a convex mesh, whose every triangle lies flat in one
/// of the hull's faces. Zero too when the points have no hull, since a flat
/// part is reported as flat.
///
/// Vertices alone are not enough: an extruded L or U has every vertex on
/// its top or bottom face, and only the triangles of its inner walls sit
/// inside. `triangles` must index `points`.
fn deepest_dent(points: &[Point3<f64>], triangles: &[[usize; 3]]) -> f64 {
    let Some(planes) = hull_planes(points) else {
        return 0.0;
    };
    let centroids = triangles.iter().map(|t| {
        Point3::from((points[t[0]].coords + points[t[1]].coords + points[t[2]].coords) / 3.0)
    });
    points
        .iter()
        .copied()
        .chain(centroids)
        .map(|p| {
            // Inside a convex hull, the nearest surface is the nearest face
            // plane; a point on the surface measures zero, give or take.
            planes
                .iter()
                .map(|(normal, offset)| offset - normal.dot(&p.coords))
                .fold(f64::INFINITY, f64::min)
                .max(0.0)
        })
        .fold(0.0, f64::max)
}

/// The face planes of the convex hull of `points`, each as an outward unit
/// normal `n` and offset `d`: `n · x = d` on the plane, less inside. `None`
/// when the points have no hull.
///
/// Measured here rather than with the collider's own point query, which
/// misreports a point lying exactly on one of the hull's corners.
fn hull_planes(points: &[Point3<f64>]) -> Option<Vec<(Vector3<f64>, f64)>> {
    let hull = hull_of(points)?;
    let (corners, faces) = hull.shape().as_convex_polyhedron()?.to_trimesh();
    let corners: Vec<Vector3<f64>> = corners
        .iter()
        .map(|c| Vector3::new(c.x, c.y, c.z).cast::<f64>())
        .collect();
    // Facing away from a point inside settles each normal's sign whatever
    // the triangles' winding.
    let inside = corners.iter().sum::<Vector3<f64>>() / corners.len() as f64;
    Some(
        faces
            .iter()
            .filter_map(|face| {
                let [a, b, c] = face.map(|i| corners[i as usize]);
                // A sliver has no reliable normal; skipping a plane can only
                // hide a dent, never invent one.
                let normal = (b - a)
                    .cross(&(c - a))
                    .try_normalize(MIN_EXTENT_M * MIN_EXTENT_M)?;
                let normal = if normal.dot(&(a - inside)) < 0.0 {
                    -normal
                } else {
                    normal
                };
                Some((normal, normal.dot(&a)))
            })
            .collect(),
    )
}

/// The convex hull of `points` as a collider, in the points' own axes;
/// `None` when they enclose no volume.
fn hull_of(points: &[Point3<f64>]) -> Option<Collider> {
    let points = points
        .iter()
        .map(|p| Vec3::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    Collider::convex_hull(points)
}

/// Whether the points enclose a volume: two far apart, a third off their
/// line, and a fourth off their plane, each by at least [`MIN_EXTENT_M`].
fn spans_volume(points: &[Point3<f64>]) -> bool {
    let Some(&a) = points.first() else {
        return false;
    };
    let farthest = |distance: &dyn Fn(&Point3<f64>) -> f64| {
        points
            .iter()
            .map(|p| (distance(p), *p))
            .fold(
                (0.0, a),
                |best, next| if next.0 > best.0 { next } else { best },
            )
    };

    let (ab, b) = farthest(&|p| (p - a).norm());
    if ab < MIN_EXTENT_M {
        return false;
    }
    let along = (b - a) / ab;
    let off_line = |p: &Point3<f64>| {
        let d = p - a;
        d - along * d.dot(&along)
    };
    let (line_distance, c) = farthest(&|p| off_line(p).norm());
    if line_distance < MIN_EXTENT_M {
        return false;
    }
    let normal = along.cross(&off_line(&c)).normalize();
    let (plane_distance, _) = farthest(&|p| (p - a).dot(&normal).abs());
    plane_distance >= MIN_EXTENT_M
}

/// A position in glTF (Bevy) axes, in the object's axes.
fn object_from_gltf(p: Point3<f64>) -> Point3<f64> {
    let object: Point<Enu> = Point::<Bevy>::from_raw(p.coords).from_bevy();
    Point3::from(object.into_inner())
}

/// A position in the object's axes, in the placed entity's Bevy frame.
fn object_to_bevy(p: Point3<f64>) -> Vec3 {
    point_bevy_to_vec3(Point::<Enu>::from_raw(p.coords).to_bevy())
}

#[cfg(test)]
#[path = "geometry_tests.rs"]
mod tests;
