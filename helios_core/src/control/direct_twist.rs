// DirectTwistController: passes the reference TrajectoryPoint's body-frame
// velocity through as a BodyTwist, applying no control law. Intended as the
// controller stage when a PathFollower (e.g., PurePursuit) has already computed
// velocity commands, so the controller only re-frames them onto the command bus.
// Whichever body-frame velocity DOF the reference carries pass through: a
// nonholonomic car specifies only Vx and Wz; an omnidirectional or aerial body
// specifies more. DOF the reference omits stay zero.
use super::commands::BodyTwist;
use super::{ControlInputs, Controller};
use crate::frames::quantities::FluVector;
use crate::frames::FrameId;
use crate::state::{Component, Quantity};

pub struct DirectTwistController;

impl DirectTwistController {
    pub fn new() -> Self {
        Self
    }
}

impl Default for DirectTwistController {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller for DirectTwistController {
    type Inputs = ControlInputs;
    type Out = BodyTwist;

    fn compute(&mut self, _dt: f64, inputs: &ControlInputs) -> BodyTwist {
        let Some(reference) = &inputs.reference else {
            return BodyTwist::zero();
        };

        let mut vx = 0.0f64;
        let mut vy = 0.0f64;
        let mut vz = 0.0f64;
        let mut wx = 0.0f64;
        let mut wy = 0.0f64;
        let mut wz = 0.0f64;

        for (i, var) in reference.state.layout.iter().enumerate() {
            let value = reference.state.vector[i];
            match (var.quantity(), var.component()) {
                (Quantity::Velocity(FrameId::Body(_)), Component::X) => vx = value,
                (Quantity::Velocity(FrameId::Body(_)), Component::Y) => vy = value,
                (Quantity::Velocity(FrameId::Body(_)), Component::Z) => vz = value,
                (Quantity::AngularVelocity(FrameId::Body(_)), Component::X) => wx = value,
                (Quantity::AngularVelocity(FrameId::Body(_)), Component::Y) => wy = value,
                (Quantity::AngularVelocity(FrameId::Body(_)), Component::Z) => wz = value,
                _ => {}
            }
        }

        BodyTwist::new(FluVector::new(vx, vy, vz), FluVector::new(wx, wy, wz))
    }

    fn reset(&mut self) {}
}
