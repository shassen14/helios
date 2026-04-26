// helios_core/src/control/direct_velocity.rs
//
// DirectVelocityController: passes Vx and Wz from the reference TrajectoryPoint
// directly to BodyVelocity output. Intended as the controller stage when a
// PathFollower (e.g., PurePursuit) has already computed velocity commands.

use nalgebra::Vector3;

use super::{ControlContext, ControlOutput, Controller};
use crate::frames::{FrameAwareState, FrameId, StateVariable};

pub struct DirectVelocityController;

impl DirectVelocityController {
    pub fn new() -> Self {
        Self
    }
}

impl Default for DirectVelocityController {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller for DirectVelocityController {
    fn compute(
        &mut self,
        _state: &FrameAwareState,
        _dt: f64,
        ctx: &ControlContext,
    ) -> ControlOutput {
        let zero = ControlOutput::BodyVelocity {
            linear: Vector3::zeros(),
            angular: Vector3::zeros(),
        };

        let Some(reference) = ctx.reference else {
            return zero;
        };

        let mut vx = 0.0f64;
        let mut wz = 0.0f64;

        for (i, var) in reference.state.layout.iter().enumerate() {
            match var {
                StateVariable::Vx(FrameId::Body(_)) => vx = reference.state.vector[i],
                StateVariable::Wz(FrameId::Body(_)) => wz = reference.state.vector[i],
                _ => {}
            }
        }

        ControlOutput::BodyVelocity {
            linear: Vector3::new(vx, 0.0, 0.0),
            angular: Vector3::new(0.0, 0.0, wz),
        }
    }

    fn reset(&mut self) {}
}
