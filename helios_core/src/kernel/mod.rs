//! Numerical foundations shared across helios_core algorithms and their
//! downstream hosts — the primitives every higher module is built on, named by
//! what they compute rather than by which algorithm consumes them.
//!
//! - [`integrators`] — RK1–RK4 for ODE stepping. Prefer RK4 unless the system
//!   is provably linear.
//! - [`determinism`] — seed derivation, so each subsystem draws from its own
//!   reproducible RNG stream instead of sharing one generator.
//! - [`manifold`] — state-manifold blocks (Euclidean, quaternion) with their
//!   retraction pairs and tangent-space noise.

pub mod determinism;
pub mod integrators;
pub mod manifold;
