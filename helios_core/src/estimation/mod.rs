// helios_core/src/estimation/mod.rs

use crate::frames::FrameAwareState;
use crate::messages::ModuleInput;
use crate::prelude::{EstimationDynamics, MeasurementMessage};
use crate::types::{Control, TfProvider};
use std::any::Any;

/// Provides the necessary world-context for an estimator to perform its work.
/// This struct is created and passed in by the simulation (`helios_sim`) layer.
#[derive(Default)]
pub struct FilterContext<'a> {
    /// Provides access to the transform tree (TF) for coordinate conversions.
    pub tf: Option<&'a dyn TfProvider>,
}

/// The contract for any algorithm that performs the "State Estimator" role.
/// Its sole responsibility is to estimate the state of an agent.
pub trait StateEstimator: Send + Sync {
    /// Predicts the state forward using a driving control input `u`.
    fn predict(&mut self, dt: f64, u: &Control, context: &FilterContext);

    /// Updates the state with an aiding measurement message.
    fn update(&mut self, message: &MeasurementMessage, context: &FilterContext);

    /// Returns a reference to the current best estimate of the state.
    fn get_state(&self) -> &FrameAwareState;

    /// Allows for dynamic downcasting to access algorithm-specific methods if needed.
    fn as_any_mut(&mut self) -> &mut dyn Any;

    fn get_dynamics_model(&self) -> &dyn EstimationDynamics;
}

pub mod filters;
