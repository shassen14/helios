// helios_sim/src/simulation/core/prng.rs

use bevy::prelude::Resource;
use rand_chacha::ChaCha8Rng;

/// A newtype wrapper around `ChaCha8Rng` to make it a Bevy Resource.
/// This is the central, deterministic pseudo-random number generator for the simulation.
#[derive(Resource)]
pub struct SimulationRng(pub ChaCha8Rng);
