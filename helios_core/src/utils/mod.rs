//! Numerical utilities shared across helios_core algorithms.
//!
//! Currently contains [`integrators`] (RK1–RK4 for ODE stepping). Prefer RK4
//! unless the system is provably linear.

pub mod integrators;
