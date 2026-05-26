use nalgebra::DVector;
use serde::{Deserialize, Serialize};

// --- Core Type Aliases ---
pub type State = DVector<f64>;
pub type Control = DVector<f64>;

/// Monotonically increasing time in seconds (simulation or hardware clock).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct MonotonicTime(pub f64);

/// A lightweight, copyable identifier for a coordinate frame (agent body, sensor, world).
///
/// In simulation, the bits encode a Bevy `Entity` via `from_entity()`/`to_entity()` (behind
/// the `bevy` feature gate). On hardware, bits encode static calibration IDs assigned at startup.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
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
