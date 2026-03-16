//! Estimation model traits and concrete implementations.
//!
//! [`dynamics`] contains `EstimationDynamics` (state propagation, Jacobians, integrator
//! dispatch) and concrete vehicle/IMU models. [`measurement`] contains the `Measurement`
//! trait (predict measurement from state, compute innovation) and concrete sensor models.

pub mod dynamics;
pub mod measurement;
