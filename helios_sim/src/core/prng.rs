use bevy::prelude::Resource;
use rand_chacha::ChaCha8Rng;

/// The seed this run actually used, resolved once at startup.
///
/// Distinct from `config.simulation.seed`, which is an `Option` because it is a
/// *request*: `None` means "pick one for me." By the time this resource exists
/// that choice has been made, so it is never optional. A run drawing its seed
/// from OS entropy is reproducible only because the drawn value is materialized
/// here and logged — re-running with `--seed <value>` replays it exactly.
///
/// Every subsystem that needs randomness derives its own stream from this via
/// `helios_core::utils::determinism::stable_hash`, rather than sharing one
/// generator. Sharing couples each subsystem's draws to how many draws every
/// other subsystem happened to take first.
#[derive(Resource)]
pub struct MasterSeed(pub u64);

/// A newtype wrapper around `ChaCha8Rng` to make it a Bevy Resource.
/// This is the central, deterministic pseudo-random number generator for the simulation.
#[derive(Resource)]
pub struct SimulationRng(pub ChaCha8Rng);
