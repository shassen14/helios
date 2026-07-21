//! Utilities shared across helios_core algorithms and their downstream hosts.
//!
//! - [`integrators`] — RK1–RK4 for ODE stepping. Prefer RK4 unless the system
//!   is provably linear.
//! - [`metrics`] — error and accuracy measures over estimates.
//! - [`determinism`] — seed derivation, so each subsystem draws from its own
//!   reproducible RNG stream instead of sharing one generator.

pub mod determinism;
pub mod integrators;
pub mod metrics;
