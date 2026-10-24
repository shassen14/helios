//! Coordinate frame conversion helpers, the frame marker every tracked entity
//! carries, and the world's datum.
//!
//! The vector/point boundary crosses through core's typed frame algebra: the
//! [`Bevy`] frame marker turns each axis swap into a real `Rotation`, and the
//! [`ToBevy`] / [`FromBevy`] traits carry any core frame across — dispatched on
//! the frame type — handing back typed `Point<Bevy>` / `FreeVector<Bevy>` values
//! that the cast helpers ([`point_bevy_to_vec3`] etc.) copy to `bevy::Vec3` at
//! the render edge. Poses cross the same way through [`ToBevy`] / [`FromBevy`] on
//! `Transform<From, To>`, with the [`transform_bevy_to_bevy_transform`] /
//! [`bevy_transform_to_transform_bevy`] casts at the `bevy::Transform` edge.
//!
//! All axis-swap logic is centralized in `bevy_bridge.rs`. Never perform manual axis swaps
//! (e.g. `v.y = physics.z`) anywhere else in the codebase — every such swap is a latent bug.

mod bevy_bridge;
mod body_twist;

pub use bevy_bridge::{
    bevy_transform_to_transform_bevy, frame_triad_to_bevy, freevector_bevy_to_vec3,
    per_axis_to_bevy, point_bevy_to_vec3, transform_bevy_to_bevy_transform,
    vec3_to_freevector_bevy, vec3_to_point_bevy, Bevy, FromBevy, ToBevy,
};
pub use body_twist::enu_twist_to_body_flu;

use bevy::prelude::*;
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

/// Marks an entity that is a coordinate frame, carrying the frame's identity
/// and axis convention as one value.
///
/// Every spawner stamps the same `(agent, leaf)` the runtime builds that
/// frame's `MeasurementModel` for — the leaf is the installation's fit key for
/// a sensor, the mount's name for a topology mount, `base_link` for the body —
/// so a reading, the frame it was taken in, and the filter's model all name
/// one identity.
///
/// The convention comes from the frame's owner (the device kind, the
/// topology) and only names the frame's axes; the pose already carries them,
/// as an installation's rotation is the device's actual axes in the body.
#[derive(Component)]
pub struct TrackedFrame {
    pub id: FrameId,
    pub convention: Convention,
}

impl TrackedFrame {
    pub fn new(id: FrameId, convention: Convention) -> Self {
        Self { id, convention }
    }
}

/// The world's datum: the global frame at the scene origin, which ground
/// truth is published in. `None` until the world layout resolves.
#[derive(Resource, Default, Debug)]
pub struct TfTree {
    datum: Option<FrameId>,
}

impl TfTree {
    /// Roots the world at `datum`.
    pub fn set_datum(&mut self, datum: FrameId) {
        self.datum = Some(datum);
    }

    /// The world's datum, once the world layout has resolved.
    pub fn datum(&self) -> Option<&FrameId> {
        self.datum.as_ref()
    }
}
