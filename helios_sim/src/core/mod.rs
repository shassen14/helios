//! Core simulation infrastructure: TF tree and ground-truth publishing.
//!
//! Contains systems that run every tick regardless of profile: ground-truth physics sync
//! (`ground_truth_sync_system`) and the structural/incremental TF tree update systems.
//! All cross-boundary frame conversions delegate to [`transforms`].

pub mod app_state;
pub mod components;
pub mod events;
pub mod ground_truth;
pub mod prng;
pub mod sim_runtime;
pub mod simulation_setup;
pub mod spawn_requests;
pub mod transforms;

pub use ground_truth::{ground_truth_sync_system, publish_oracle_channels_system};
