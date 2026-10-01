//! What a prefab's `.glb` means physically: its bounding box (a ground-truth
//! label) and its collider, derived once per prefab from the glTF node tree.
//!
//! Every node's transform is composed down from the scene root, so where a
//! part was left in Blender (unapplied transforms, origins, parenting) does
//! not matter. Nodes named `col_*` are collider parts, each hulled on its
//! own; every other node is visual. An asset without parts collides as its
//! bounding box.
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

/// One `col_` node's vertices, in the object's axes.
#[derive(Debug, Clone)]
struct ColliderPart {
    name: String,
    points: Vec<Point3<f64>>,
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
    let points: Vec<Vec3> = points
        .iter()
        .map(|p| Vec3::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    // At unit density a body's mass is its volume.
    let hull_volume = Collider::convex_hull(points)
        .map(|hull| f64::from(hull.mass(1.0)))
        .unwrap_or(0.0);
    hull_volume / (size.x * size.y * size.z)
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
