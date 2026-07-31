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

#[cfg(test)]
mod tests {
    use super::*;

    use rand::RngCore;

    const MASTER_SEED: u64 = 2024;
    const SENSOR: &str = "car_1/gps";

    /// The number of draws compared. Any small number works: two ChaCha streams
    /// from different seeds diverge at the first value, so a longer prefix
    /// would not catch anything a short one misses.
    const PREFIX_LEN: usize = 4;

    /// Compares streams by what they *produce*, not by the seed they were built
    /// from. A test asserting on seeds would still pass if the seed never
    /// reached the generator — which is the exact bug this project has already
    /// shipped once, when a configured seed was read before the config loaded.
    fn prefix(mut rng: SensorRng) -> Vec<u64> {
        (0..PREFIX_LEN).map(|_| rng.0.next_u64()).collect()
    }

    #[test]
    fn the_same_sensor_and_seed_reproduce_the_stream() {
        assert_eq!(
            prefix(SensorRng::from_sensor(MASTER_SEED, SENSOR)),
            prefix(SensorRng::from_sensor(MASTER_SEED, SENSOR))
        );
    }

    #[test]
    fn different_sensors_draw_different_streams() {
        assert_ne!(
            prefix(SensorRng::from_sensor(MASTER_SEED, SENSOR)),
            prefix(SensorRng::from_sensor(MASTER_SEED, "car_1/imu"))
        );
    }

    /// The two halves of one IMU are the case worth naming. They come from a
    /// single config entry and differ only by the suffix each spawner appends,
    /// so a spawner that derived one seed and handed it to both children would
    /// produce perfectly correlated accelerometer and gyroscope noise — a
    /// defect that looks entirely plausible on a plot.
    #[test]
    fn an_imus_two_halves_draw_different_streams() {
        assert_ne!(
            prefix(SensorRng::from_sensor(
                MASTER_SEED,
                "car_1/imu/accelerometer"
            )),
            prefix(SensorRng::from_sensor(MASTER_SEED, "car_1/imu/gyroscope"))
        );
    }

    /// Without this, a derivation that ignored the master seed would satisfy
    /// every other test here while making `--seed` inert.
    #[test]
    fn a_different_master_seed_moves_the_stream() {
        assert_ne!(
            prefix(SensorRng::from_sensor(MASTER_SEED, SENSOR)),
            prefix(SensorRng::from_sensor(MASTER_SEED + 1, SENSOR))
        );
    }
}
