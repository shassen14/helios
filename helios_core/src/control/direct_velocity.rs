// DirectVelocityController: passes Vx and Wz from the reference TrajectoryPoint
// directly through as a BodyTwist. Intended as the controller stage when a
// PathFollower (e.g., PurePursuit) has already computed velocity commands, so the
// controller only re-frames them into the command bus.
use super::commands::BodyTwist;
use super::{ControlInputs, Controller};
use crate::frames::conventions::FluVector;
use crate::frames::{FrameId, StateVariable};

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
    type Inputs = ControlInputs;
    type Out = BodyTwist;

    fn compute(&mut self, _dt: f64, inputs: &ControlInputs) -> BodyTwist {
        let zero = BodyTwist::zero();

        let Some(reference) = &inputs.reference else {
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

        BodyTwist::new(FluVector::new(vx, 0.0, 0.0), FluVector::new(0.0, 0.0, wz))
    }

    fn reset(&mut self) {}
}
