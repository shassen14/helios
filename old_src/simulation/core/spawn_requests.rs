// src/simulation/core/spawn_requests.rs
use bevy::prelude::Component;

/// A temporary "tag" component used by the two-pass spawner.
/// It requests that a specialized plugin (e.g., AckermannCarPlugin)
/// attach a physical body and vehicle-specific logic to this entity.
#[derive(Component)]
pub struct SpawnRequestAckermann;

// In the future, you would add other requests here:
// #[derive(Component)]
// pub struct SpawnRequestQuadcopter;
