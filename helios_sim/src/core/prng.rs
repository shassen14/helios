use helios_core::utils::determinism::stable_hash;

use bevy::prelude::{Component, Resource};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// Namespaces sensor seeds against every other subsystem that derives from the
/// same master seed. Two subsystems passing different kinds are guaranteed
/// disjoint streams even if they hash identical ids.
const KIND: &str = "sensor";

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

/// One sensor's private noise stream, seeded at spawn from [`MasterSeed`] and
/// the sensor's name.
///
/// A component rather than a map keyed by entity in some resource: a map would
/// still hand every sensor system exclusive access to one resource, so the
/// scheduler would serialize them exactly as a shared generator does, only with
/// an extra lookup. As a component it enters Bevy's access analysis, and sensors
/// that touch disjoint entities can run in parallel.
///
/// **Exactly one system may draw from a given instance** — the one that publishes
/// that sensor. Two readers reintroduce order-coupling at a smaller scale, where
/// it is harder to notice.
///
/// The payoff is that a reading becomes a pure function of the master seed, the
/// sensor's own name, and how many times that sensor has sampled. Nothing else:
/// not how many other sensors the agent carries, not what order the scheduler
/// happened to run them in, not how many surfaces a lidar hit last tick.
#[derive(Component)]
pub struct SensorRng(pub ChaCha8Rng);

impl SensorRng {
    /// Derives a sensor's stream from the run's master seed and its name.
    ///
    /// `sensor_name` must be the sensor's **full entity name** — the
    /// agent-qualified path, not the config map key on its own. A map key is
    /// unique only within one agent, so keying on it would hand every agent's
    /// GPS the same stream, and a fleet's worth of receivers would report
    /// identical noise.
    ///
    /// For the same reason the IMU's two children pass their own distinct paths
    /// rather than the shared IMU entry's: an accelerometer and a gyroscope
    /// drawing from one stream produce perfectly correlated noise, which is
    /// plausible enough on a plot to go unnoticed.
    pub fn from_sensor(master_seed: u64, sensor_name: &str) -> Self {
        let hash = stable_hash(master_seed, KIND, sensor_name);
        Self(ChaCha8Rng::seed_from_u64(hash))
    }
}
