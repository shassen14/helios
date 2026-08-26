//! Plant models: the forward map from an actuator command to the forces it
//! produces on a body.
//!
//! A plant is what a controller acts *on* — the counterpart to the laws in
//! [`super::controllers`]. Fidelity is a ladder: [`L0ShimPlant`] is the crudest
//! rung, folding a command straight into one chassis wrench; richer per-wheel
//! dynamic plants replace it without changing this module's role.

pub mod l0_shim;

pub use l0_shim::{L0ShimPlant, L0ShimWrench};
