use crate::control::commands::BodyTwist;

use std::any::Any;

pub trait ControlReference: Any + Send + Sync {}

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
