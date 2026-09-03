use crate::{
    frames::FrameId,
    state::{Component, StateVariable},
};

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Quantity {
    Position(FrameId),
    Velocity(FrameId),
    Acceleration(FrameId),
    AngularVelocity(FrameId),
    AngularAcceleration(FrameId),
    Mag(FrameId),
    MagBias(FrameId),
    AccelBias(FrameId),
    GyroBias(FrameId),
    Orientation { from: FrameId, to: FrameId },
}

impl Quantity {
    pub fn variables(&self) -> Vec<StateVariable> {
        self.components()
            .into_iter()
            .map(|component| StateVariable::new(self.clone(), component))
            .collect()
    }

    pub fn components(&self) -> Vec<Component> {
        match self {
            Quantity::Orientation { .. } => {
                vec![Component::X, Component::Y, Component::Z, Component::W]
            }
            _ => {
                vec![Component::X, Component::Y, Component::Z]
            }
        }
    }
}
