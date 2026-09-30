//! The simulated environment: the ground truth the agents' sensors observe and
//! the bodies collide with.
//!
//! Each submodule builds one part of that world — [`terrain`] loads the ground
//! mesh and its collider, [`atmosphere`] sets gravity and ambient conditions,
//! [`layout`] checks and spawns the scene's objects, and [`asset_gate`] starts
//! the scene build once all of their assets have loaded. Unlike `agents`, nothing here
//! belongs to any one agent; it is the shared stage they all act on.
//!
//! [`HeliosWorldPlugin`] adds them all.

pub mod asset_gate;
pub mod atmosphere;
pub mod layout;
pub mod plugin_set;
pub mod terrain;

pub use asset_gate::AssetGatePlugin;
pub use atmosphere::AtmospherePlugin;
pub use layout::{ResolvedWorldLayout, WorldLayoutPlugin};
pub use plugin_set::HeliosWorldPlugin;
pub use terrain::{TerrainAssets, TerrainPlugin};
