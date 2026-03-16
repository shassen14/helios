//! Concrete state estimator implementations.
//!
//! Contains the Extended Kalman Filter ([`ekf`]) and Unscented Kalman Filter ([`ukf`]).
//! Both implement the `StateEstimator` trait from the parent `estimation` module.

pub mod ekf;
pub mod ukf;
