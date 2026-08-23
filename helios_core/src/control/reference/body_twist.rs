use crate::control::commands::BodyTwist;
use crate::control::reference::ControlReference;

/// A body-frame twist reference: the linear/angular velocity a controller should
/// track, already expressed in the body frame (no frame projection needed).
#[derive(Clone)]
pub struct BodyTwistRef(BodyTwist);

impl ControlReference for BodyTwistRef {}

impl BodyTwistRef {
    pub fn new(body_twist: BodyTwist) -> Self {
        Self(body_twist)
    }

    pub fn twist(&self) -> &BodyTwist {
        &self.0
    }
}
