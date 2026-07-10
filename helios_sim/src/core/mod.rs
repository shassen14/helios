//! Core simulation infrastructure: everything that exists independently of any
//! one domain plugin.
//!
//! App assembly ([`host`]) and the state machine and system-set graph it drives
//! ([`app_state`], [`simulation_setup`]); the shared ECS vocabulary
//! ([`components`], [`events`], [`spawn_requests`]); the seeded RNG
//! ([`prng`]) and the per-agent runtime adapter ([`sim_runtime`]); and the
//! per-tick systems that bridge physics to the rest of the sim — ground-truth
//! sync and oracle publishing ([`ground_truth`]) plus the TF tree
//! ([`transforms`]).
//!
//! All cross-boundary frame conversions delegate to [`transforms`].

pub mod app_state;
pub mod components;
pub mod events;
pub mod ground_truth;
pub mod host;
pub mod prng;
pub mod sim_runtime;
pub mod simulation_setup;
pub mod spawn_requests;
pub mod transforms;

pub use ground_truth::{ground_truth_sync_system, publish_oracle_channels_system};
