//! Vehicle simulation: turns the pipeline's control output into physical motion.
//!
//! Each submodule is one vehicle family's host plugin. A vehicle plugin
//! translates the brain's control command into the inputs its Avian3D body obeys
//! (forces, target velocities, steering) and reads the resulting state back out.
//! It does not integrate the motion itself — the physics engine does.
//! [`ackermann`] is the one implemented family.
//!
//! [`HeliosVehiclesPlugin`] adds them all.

pub mod ackermann;
pub mod plugin_set;

pub use plugin_set::HeliosVehiclesPlugin;
