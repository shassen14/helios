use crate::{
    frames::FrameId,
    state::{Component, StateVariable},
};

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Quantity {
    Position(FrameId),
    Velocity(FrameId),
    Acceleration(FrameId),
    SpecificForce(FrameId),
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

    pub fn frame(&self) -> Option<&FrameId> {
        match self {
            Quantity::Position(f)
            | Quantity::Velocity(f)
            | Quantity::Acceleration(f)
            | Quantity::SpecificForce(f)
            | Quantity::AngularVelocity(f)
            | Quantity::AngularAcceleration(f)
            | Quantity::Mag(f)
            | Quantity::MagBias(f)
            | Quantity::AccelBias(f)
            | Quantity::GyroBias(f) => Some(f),
            Quantity::Orientation { .. } => None,
        }
    }
}

impl std::fmt::Display for Quantity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Quantity::Position(frame) => write!(f, "position in {frame}"),
            Quantity::Velocity(frame) => write!(f, "velocity in {frame}"),
            Quantity::Acceleration(frame) => write!(f, "acceleration in {frame}"),
            Quantity::SpecificForce(frame) => write!(f, "specific force in {frame}"),
            Quantity::AngularVelocity(frame) => write!(f, "angular velocity in {frame}"),
            Quantity::AngularAcceleration(frame) => {
                write!(f, "angular acceleration in {frame}")
            }
            Quantity::Mag(frame) => write!(f, "magnetic field in {frame}"),
            Quantity::MagBias(frame) => write!(f, "magnetometer bias in {frame}"),
            Quantity::AccelBias(frame) => write!(f, "accelerometer bias in {frame}"),
            Quantity::GyroBias(frame) => write!(f, "gyroscope bias in {frame}"),
            Quantity::Orientation { from, to } => write!(f, "orientation {from}→{to}"),
        }
    }
}
