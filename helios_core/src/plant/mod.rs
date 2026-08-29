//! Plant models: the forward map from an actuator command to the forces it
//! produces on a body.
//!
//! A plant is world-truth — what the body *actually does* when actuated, the
//! motion dual of [`sensors`](crate::sensors) (observation-truth). It is what a
//! control law acts *on*, not part of the law, which is why it sits at the crate
//! root beside `sensors/` rather than inside [`control`](crate::control). The
//! forward force it computes is integrated by the host's physics engine; the
//! plant supplies the forces, the integrator supplies the motion.
//!
//! Fidelity is a ladder: [`L0ShimPlant`] is the crudest rung, folding a command
//! straight into one chassis wrench; richer per-wheel dynamic plants replace it
//! without changing this module's role.

pub mod l0_shim;
pub mod raycast_wheels;

pub use l0_shim::{L0ShimPlant, L0ShimWrench};
pub use raycast_wheels::{Axle, RaycastWheelPlant, SuspensionParams, TireParams, Wheel, WheelRay};
