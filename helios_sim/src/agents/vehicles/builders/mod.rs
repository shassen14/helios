//! The built-in embodiment axis builders — one `kind` per file, each a plain fn
//! the `EmbodimentRegistry` dispatches on. These are morphology-general (a drone
//! reuses `RigidBodyWithMount`), so they live above any one vehicle family.

pub mod collision;
pub mod plant;
pub mod topology;

pub use collision::build_cuboid;
pub use plant::{build_l0_shim, L0ShimPlantComponent};
pub use topology::build_rigid_body_with_mount;
