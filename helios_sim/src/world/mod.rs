//! The simulated environment: the ground truth the agents' sensors observe and
//! the bodies collide with.
//!
//! Each submodule builds one part of that world — [`terrain`] loads the ground
//! mesh and its collider, [`atmosphere`] sets gravity and ambient conditions,
//! [`objects`] spawns the static scene props. Unlike `agents`, nothing here
//! belongs to any one agent; it is the shared stage they all act on.
//!
//! [`HeliosWorldPlugin`] adds them all.

pub mod atmosphere;
mod object_helpers;
pub mod objects;
pub mod plugin_set;
pub mod terrain;

pub use atmosphere::AtmospherePlugin;
pub use objects::{WorldObjectAssets, WorldObjectPlugin};
pub use plugin_set::HeliosWorldPlugin;
pub use terrain::{TerrainAssets, TerrainPlugin};
