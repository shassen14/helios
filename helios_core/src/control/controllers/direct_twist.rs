// DirectTwistController: passes the reference BodyTwistRef's body-frame velocity
// through as a BodyTwist, applying no control law. Intended as the controller
// stage when a PathFollower (e.g., PurePursuit) has already computed velocity
// commands, so the controller only re-frames them onto the command bus.
use crate::control::commands::BodyTwist;
use crate::control::{BodyTwistRef, ControlInputs, Controller};

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
    type Inputs = ControlInputs<BodyTwistRef>;
    type Out = BodyTwist;

    fn compute(&mut self, _dt: f64, inputs: &ControlInputs<BodyTwistRef>) -> BodyTwist {
        match &inputs.reference {
            Some(reference) => *reference.twist(),
            None => BodyTwist::zero(),
        }
    }

    fn reset(&mut self) {}
}
