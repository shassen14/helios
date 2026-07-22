//! Magnetometer simulation: spawns receivers from agent configs and publishes
//! the ambient geomagnetic field, expressed in the sensor frame, through the
//! generic state-sensor system.
//!
//! The field itself is a property of the world, not of any one sensor, so it
//! comes from the scenario's world config — every magnetometer in a run reads
//! the same reference field, exactly as every real compass on a site does.

use super::state_sensor::{publish_state_sensor, SensorTimer, StateSensor};

use crate::brain_bridge::components::SensorPublishChannel;
use crate::core::app_state::SimulationSet;
use crate::core::prng::{MasterSeed, SensorRng};
use crate::prelude::*;

use helios_core::data::sensor::MagneticField3D;
use helios_core::sensors::magnetometer::MagnetometerModel;

use nalgebra::{Isometry3, Vector3};
use rand::RngCore;

/// A simulated 3-axis magnetometer: wraps the core forward model, which
/// rotates the world's reference field into the sensor frame and adds bias and
/// per-axis noise.
///
/// The reference field is baked into the model at spawn: it is an
/// environmental constant shared by every sensor in the scenario, not agent
/// state, so the publish loop never re-reads it.
#[derive(Component)]
pub struct Magnetometer {
    model: MagnetometerModel,
}

impl Magnetometer {
    pub fn new(model: MagnetometerModel) -> Self {
        Self { model }
    }
}

impl StateSensor for Magnetometer {
    type Payload = MagneticField3D;

    /// A magnetic field is a free vector, so only the pose's rotation matters
    /// — inverted, because the model wants world-into-sensor and the pose's
    /// rotation goes the other way. The agent's kinematic state goes unused:
    /// the reading depends on where the sensor points, not on how it moves.
    fn sample(
        &mut self,
        _truth: &GroundTruthState,
        sensor_pose_world: &Isometry3<f64>,
        rng: &mut dyn RngCore,
    ) -> Self::Payload {
        self.model.sample(sensor_pose_world.rotation.inverse(), rng)
    }
}

/// Registers magnetometer simulation: spawning during scene build, publishing
/// through the generic state-sensor system while running.
pub struct MagnetometerPlugin;

impl Plugin for MagnetometerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_magnetometer_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            publish_state_sensor::<Magnetometer>.in_set(SimulationSet::Sensors),
        );
    }
}

/// Spawns one magnetometer child entity per magnetometer entry in each agent's
/// spawn request, each carrying the scenario's geomagnetic reference field.
///
/// Model construction doubles as config validation: the core model refuses a
/// stddev with any non-positive axis, so a bad config skips the sensor with an
/// error instead of spawning a degenerate one.
fn spawn_magnetometer_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
    config: Res<ScenarioConfig>,
    master_seed: Res<MasterSeed>,
) {
    let magnetic_field = &config.world.magnetic_field;

    for (agent_entity, agent_name, request) in &request_query {
        for (sensor_name, sensor_config) in &request.0.sensors {
            if let SensorConfig::Magnetometer(mag_config) = sensor_config {
                info!(
                    "  -> Spawning Magnetometer '{}' as child of agent '{}' with rate of {:.1} Hz",
                    sensor_name,
                    agent_name.as_str(),
                    mag_config.get_rate()
                );

                let Some(mag_model) = MagnetometerModel::from_reference_field(
                    magnetic_field.declination_degrees.to_radians(),
                    magnetic_field.inclination_degrees.to_radians(),
                    magnetic_field.magnitude,
                    Vector3::from(mag_config.bias),
                    Vector3::from(mag_config.noise_stddev),
                ) else {
                    error!(
                        "Magnetometer '{}' on agent '{}' has invalid noise_stddev {:?}: \
                         every axis must be > 0. Skipping sensor.",
                        sensor_name,
                        agent_name.as_str(),
                        mag_config.noise_stddev
                    );
                    continue;
                };

                let sensor_label = format!("{}/{}", agent_name.as_str(), sensor_name);
                let sensor_rng = SensorRng::from_sensor(master_seed.0, &sensor_label);

                let sensor_entity = commands
                    .spawn((
                        Name::new(sensor_label),
                        SensorPublishChannel(mag_config.channel.clone()),
                        Magnetometer::new(mag_model),
                        SensorTimer::from_rate(mag_config.rate),
                        sensor_rng,
                        TrackedFrame,
                        mag_config.get_relative_pose().to_bevy_local_transform(),
                    ))
                    .id();

                commands.entity(agent_entity).add_child(sensor_entity);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::FRAC_PI_2;

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    /// Small enough that a sample equals its ideal value within test
    /// tolerance, since the model refuses an exactly-zero stddev.
    const NEGLIGIBLE_STDDEV: f64 = 1e-12;

    /// A unit-magnitude field, so the tests read as direction checks.
    fn magnetometer(declination: f64, inclination: f64) -> Magnetometer {
        biased_magnetometer(declination, inclination, Vector3::zeros())
    }

    fn biased_magnetometer(declination: f64, inclination: f64, bias: Vector3<f64>) -> Magnetometer {
        Magnetometer::new(
            MagnetometerModel::from_reference_field(
                declination,
                inclination,
                1.0,
                bias,
                Vector3::repeat(NEGLIGIBLE_STDDEV),
            )
            .unwrap(),
        )
    }

    /// A pose yawed +90° about world Up, so world North lands on the sensor's
    /// +X axis. The un-inverted rotation would give -X — this pose is what
    /// catches a `sample` impl passing the rotation the wrong way around.
    fn yawed_pose() -> Isometry3<f64> {
        Isometry3::rotation(Vector3::z() * FRAC_PI_2)
    }

    fn sample(mag: &mut Magnetometer, pose: &Isometry3<f64>) -> Vector3<f64> {
        let mut rng = StdRng::seed_from_u64(1);
        mag.sample(&GroundTruthState::default(), pose, &mut rng)
            .value
    }

    #[test]
    fn sample_reads_the_reference_field_in_world_axes_when_unrotated() {
        let mut mag = magnetometer(0.0, 0.0);

        let got = sample(&mut mag, &Isometry3::identity());

        assert!((got - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn sample_rotates_the_field_into_the_sensor_frame() {
        let mut mag = magnetometer(0.0, 0.0);

        let got = sample(&mut mag, &yawed_pose());

        assert!((got - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn sample_applies_the_configured_hard_iron_bias() {
        // Bias is fixed in the sensor frame, so it survives the rotation into
        // sensor axes rather than being rotated along with the world field.
        let mut mag = biased_magnetometer(0.0, 0.0, Vector3::new(0.5, 0.0, 0.0));

        let got = sample(&mut mag, &yawed_pose());

        assert!((got - Vector3::new(1.5, 0.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn sample_carries_the_configured_declination_and_inclination() {
        // A 90° declination swings the field from north to east; the sensor's
        // own translation is irrelevant to a free vector.
        let mut mag = magnetometer(FRAC_PI_2, 0.0);

        let got = sample(&mut mag, &Isometry3::translation(10.0, -20.0, 30.0));

        assert!((got - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-6);
    }
}
