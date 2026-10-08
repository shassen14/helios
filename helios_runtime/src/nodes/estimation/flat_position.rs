//! A test dynamics model whose state is flat: one `odom` position block that
//! holds still up to a random walk, driven by no input. It gives a filter that
//! refuses a curved state (the UKF) something to build and run on.

use helios_core::estimation::dynamics::EstimationDynamics;
use helios_core::estimation::schema::{InputSchema, StateSchema, StateSchemaBlock};
use helios_core::kernel::integrators::Integrator;
use helios_core::kernel::manifold::TangentNoise;
use helios_core::prelude::AgentId;
use helios_core::spatial::state::Quantity;
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

/// The position's random-walk variance rate, m²/s per axis.
const POSITION_WALK_VAR: f64 = 0.01;
/// The position's starting variance, m² per axis.
const POSITION_PRIOR_VAR: f64 = 4.0;

/// `odom` position in ENU, still up to a random walk.
#[derive(Debug)]
pub(crate) struct FlatPosition {
    schema: Arc<StateSchema>,
}

impl FlatPosition {
    pub(crate) fn new(agent: AgentId) -> Self {
        let noise = TangentNoise::from_variances(DVector::from_element(3, POSITION_WALK_VAR))
            .expect("a positive diagonal has a Cholesky factor");
        let block = StateSchemaBlock::new(
            Quantity::Position(FrameId::odom(agent)),
            Convention::Enu,
            Some(noise),
            DVector::zeros(3),
            DMatrix::identity(3, 3) * POSITION_PRIOR_VAR,
        );
        Self {
            schema: Arc::new(StateSchema::compose(vec![block])),
        }
    }
}

impl EstimationDynamics for FlatPosition {
    fn input_schema(&self) -> Arc<InputSchema> {
        Arc::new(InputSchema::compose(vec![]))
    }

    fn schema(&self) -> Arc<StateSchema> {
        Arc::clone(&self.schema)
    }

    fn propagate(
        &self,
        x: &DVector<f64>,
        _u: &DVector<f64>,
        _t: f64,
        _dt: f64,
        _integrator: &dyn Integrator<f64>,
    ) -> DVector<f64> {
        x.clone()
    }
}
