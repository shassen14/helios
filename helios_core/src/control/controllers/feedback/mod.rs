//! Feedback laws — drive a command from the error between reference and measured
//! state. These generalize: the model-based family (LQR, H₂, H∞, MPC, iLQR, state
//! feedback) all synthesize `u` from a plant model, with the plant as a parameter,
//! so they share this module regardless of morphology.

pub mod longitudinal_velocity;
