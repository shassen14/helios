// helios_sim/src/simulation/core/transforms/frame_types.rs
//
// Typed coordinate frame newtypes that make illegal frame conversions unrepresentable.
// All newtypes wrap `Isometry3<f64>` or `Vector3<f64>` — no Bevy dependency here.
//
// These live in helios_sim (not helios_runtime) so that standard `From`/`Into` impls
// for Bevy types can be defined without violating Rust's orphan rules.

use nalgebra::{Isometry3, Vector3};

/// A world-frame pose in the ENU coordinate system, for static objects (terrain, buildings).
///
/// Use for static world objects whose meshes are authored in Bevy world space.
/// Uses the pure similarity transform when converting to Bevy: identity ENU → identity Bevy.
///
/// For agent/vehicle body poses use [`EnuBodyPose`] instead.
#[derive(Debug, Clone, Copy)]
pub struct EnuWorldPose(pub Isometry3<f64>);

/// An agent/vehicle body pose in the ENU coordinate system with the FLU body convention.
///
/// Accounts for the FLU body convention (Forward=+X) when converting to Bevy world space.
/// ENU identity (heading East, FLU +X = ENU East) → Bevy R_y(−π/2).
///
/// Use for spawning agents, visualizing agent poses, and reading physics transforms.
#[derive(Debug, Clone, Copy)]
pub struct EnuBodyPose(pub Isometry3<f64>);

/// A sensor child transform in the FLU body-relative coordinate system.
///
/// FLU: Forward=+X, Left=+Y, Up=+Z. Use for all sensor local transforms
/// relative to a vehicle body. FLU identity → Bevy local identity.
#[derive(Debug, Clone, Copy)]
pub struct FluLocalPose(pub Isometry3<f64>);

/// A 3D vector in the ENU world coordinate system (East=+X, North=+Y, Up=+Z).
#[derive(Debug, Clone, Copy)]
pub struct EnuVector(pub Vector3<f64>);

/// A 3D vector in the FLU body-relative coordinate system (Forward=+X, Left=+Y, Up=+Z).
#[derive(Debug, Clone, Copy)]
pub struct FluVector(pub Vector3<f64>);
