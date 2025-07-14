// helios_core/src/types.rs

use nalgebra::{DVector, Isometry3};

// --- Core Type Aliases ---
pub type State = DVector<f64>;
pub type Control = DVector<f64>;

// --- Core Identifier ---
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct FrameHandle(pub u64);

impl FrameHandle {
    // A convenience method for use in the Bevy adapter crate.
    #[cfg(feature = "bevy")] // This will only compile if the "bevy" feature is enabled
    pub fn from_entity(entity: bevy_ecs::prelude::Entity) -> Self {
        Self(entity.to_bits())
    }

    #[cfg(feature = "bevy")]
    pub fn to_entity(self) -> bevy_ecs::prelude::Entity {
        bevy_ecs::prelude::Entity::from_bits(self.0)
    }
}

// --- Core Trait for Transform Lookups ---
// This is so fundamental it belongs here.
pub trait TfProvider {
    fn get_transform(&self, from: FrameHandle, to: FrameHandle) -> Option<Isometry3<f64>>;
}
