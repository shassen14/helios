//! Foundations shared across helios_core algorithms and their
//! downstream hosts — the primitives every higher module is built on, named by
//! what they compute rather than by which algorithm consumes them.
//!
//! - [`integrators`] — RK1–RK4 for ODE stepping. Prefer RK4 unless the system
//!   is provably linear.
//! - [`determinism`] — seed derivation, so each subsystem draws from its own
//!   reproducible RNG stream instead of sharing one generator.
//! - [`manifold`] — state-manifold blocks (Euclidean, quaternion) with their
//!   retraction pairs and tangent-space noise.
//! - [`identifier`] — the rules config-written names follow: `snake_case`, and
//!   levels joined by `.` for names that address something.
//! - [`si_unit`] — [`SiUnit`](si_unit::SiUnit), the SI unit a number is
//!   measured in.

pub mod determinism;
pub mod identifier;
pub mod integrators;
pub mod manifold;
pub mod si_unit;
