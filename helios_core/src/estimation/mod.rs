// helios_core/src/estimation/mod.rs

use crate::frames::FrameAwareState;
use crate::messages::ModuleInput;
use crate::types::TfProvider;
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
    /// The single, unified method for processing all types of input data.
    /// The implementation is responsible for interpreting the `ModuleInput`.
    fn process(&mut self, input: &ModuleInput, context: &FilterContext);

    /// Returns a reference to the current best estimate of the state.
    fn get_state(&self) -> &FrameAwareState;

    /// Allows for dynamic downcasting to access algorithm-specific methods if needed.
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

pub mod filters;
