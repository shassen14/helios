//! IMU sensor simulation: spawns one accelerometer and one gyroscope per
//! configured IMU and publishes their readings through the generic
//! state-sensor system.
//!
//! An IMU is one config entry but two sensors: it measures two independent
//! physical quantities that publish on separate channels, so each spawns as
//! its own self-contained state-sensor entity.

use super::state_sensor::{publish_state_sensor, SensorTimer, StateSensor};

use crate::core::app_state::SimulationSet;
use crate::core::transforms::EnuVector;
use crate::prelude::*;

use helios_core::data::sensor::{AngularVelocity3D, LinearAcceleration3D};
use helios_core::sensors::accelerometer::AccelerometerModel;
use helios_core::sensors::gyroscope::GyroscopeModel;

use avian3d::prelude::Gravity;
use nalgebra::{Isometry3, Vector3};
use rand::RngCore;

/// A simulated accelerometer: wraps the core forward model, which reports
/// specific force — coordinate acceleration minus gravity, so a sensor at
/// rest reads one g upward — in the sensor frame, with bias and per-axis
/// noise.
///
/// Gravity is baked in at spawn: the model needs it every sample, but it is
/// an environmental constant, not agent state, so the spawner captures it
/// from the physics world once rather than the publish loop reading it every
/// tick.
///
/// TODO(fidelity): `sample` reads the acceleration of the agent's origin,
/// not of the sensor's mount point. A sensor at offset `r` on a rotating
/// body also feels the lever-arm terms `α × r + ω × (ω × r)` — below the
/// noise floor for near-origin mounts at gentle rates, but real on spinning
/// platforms or long mounting arms. Modeling it needs angular acceleration
/// in the ground-truth state and a richer core forward model.
#[derive(Component)]
pub struct Accelerometer {
    model: AccelerometerModel,
    gravity_world: Vector3<f64>,
}

impl Accelerometer {
    pub fn new(model: AccelerometerModel, gravity_world: Vector3<f64>) -> Self {
        Self {
            model,
            gravity_world,
        }
    }
}

impl StateSensor for Accelerometer {
    type Payload = LinearAcceleration3D;

    /// Acceleration is a free vector, so only the pose's rotation matters —
    /// inverted, because the model wants world-into-sensor and the pose's
    /// rotation goes the other way.
    fn sample(
        &mut self,
        truth: &GroundTruthState,
        sensor_pose_world: &Isometry3<f64>,
        rng: &mut dyn RngCore,
    ) -> Self::Payload {
        self.model.sample(
            truth.linear_acceleration,
            self.gravity_world,
            sensor_pose_world.rotation.inverse(),
            rng,
        )
    }
}

/// A simulated rate gyroscope: wraps the core forward model, which reports
/// the agent's angular velocity in the sensor frame, with bias and per-axis
/// noise.
#[derive(Component)]
pub struct Gyroscope {
    model: GyroscopeModel,
}

impl Gyroscope {
    pub fn new(model: GyroscopeModel) -> Self {
        Self { model }
    }
}

impl StateSensor for Gyroscope {
    type Payload = AngularVelocity3D;

    /// Angular velocity is a free vector, so only the pose's rotation
    /// matters — inverted, same as the accelerometer.
    fn sample(
        &mut self,
        truth: &GroundTruthState,
        sensor_pose_world: &Isometry3<f64>,
        rng: &mut dyn RngCore,
    ) -> Self::Payload {
        self.model.sample(
            truth.angular_velocity,
            sensor_pose_world.rotation.inverse(),
            rng,
        )
    }
}

/// Registers IMU simulation: spawning during scene build, publishing both
/// sensors through the generic state-sensor system while running.
pub struct ImuPlugin;

impl Plugin for ImuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_imu_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            (
                publish_state_sensor::<Gyroscope>,
                publish_state_sensor::<Accelerometer>,
            )
                .in_set(SimulationSet::Sensors),
        );
    }
}

/// Spawns one accelerometer and one gyroscope child entity per IMU entry in
/// each agent's spawn request.
///
/// Model construction doubles as config validation: the core models refuse a
/// stddev with any non-positive axis, so a bad config skips the whole IMU
/// with an error — half an IMU would be more confusing than none.
fn spawn_imu_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
    gravity: Res<Gravity>,
) {
    let gravity_world = EnuVector::from(gravity.0).0;

    for (agent_entity, agent_name, request) in &request_query {
        for sensor_config in request.0.sensors.values() {
            if let SensorConfig::Imu(imu_config) = sensor_config {
                info!(
                    "  -> Spawning IMU '{}' as child of agent '{}' with rate of {:.1} Hz",
                    imu_config.get_name(),
                    agent_name.as_str(),
                    imu_config.get_rate()
                );

                let sensor_pose = imu_config.get_relative_pose();
                let (accel_std, gyro_std) = imu_config.get_noise_stddevs();
                let (accel_bias, gyro_bias) = imu_config.get_biases();

                let Some(accel_model) =
                    AccelerometerModel::new(Vector3::from(accel_bias), Vector3::from(accel_std))
                else {
                    error!(
                        "IMU '{}' on agent '{}' has invalid accel_noise_stddev {:?}: \
                         every axis must be > 0. Skipping IMU.",
                        imu_config.get_name(),
                        agent_name.as_str(),
                        accel_std
                    );
                    continue;
                };

                let Some(gyro_model) =
                    GyroscopeModel::new(Vector3::from(gyro_bias), Vector3::from(gyro_std))
                else {
                    error!(
                        "IMU '{}' on agent '{}' has invalid gyro_noise_stddev {:?}: \
                         every axis must be > 0. Skipping IMU.",
                        imu_config.get_name(),
                        agent_name.as_str(),
                        gyro_std
                    );
                    continue;
                };

                let accel_entity = commands
                    .spawn((
                        Name::new(format!(
                            "{}/{}/accelerometer",
                            agent_name.as_str(),
                            imu_config.get_name()
                        )),
                        SensorPublishChannel(imu_config.get_accel_channel().to_string()),
                        Accelerometer::new(accel_model, gravity_world),
                        SensorTimer::from_rate(imu_config.rate),
                        TrackedFrame,
                        sensor_pose.to_bevy_local_transform(),
                    ))
                    .id();

                let gyro_entity = commands
                    .spawn((
                        Name::new(format!(
                            "{}/{}/gyroscope",
                            agent_name.as_str(),
                            imu_config.get_name()
                        )),
                        SensorPublishChannel(imu_config.get_gyro_channel().to_string()),
                        Gyroscope::new(gyro_model),
                        SensorTimer::from_rate(imu_config.rate),
                        TrackedFrame,
                        sensor_pose.to_bevy_local_transform(),
                    ))
                    .id();

                commands
                    .entity(agent_entity)
                    .add_child(accel_entity)
                    .add_child(gyro_entity);
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
    /// tolerance, since the models refuse an exactly-zero stddev.
    const NEGLIGIBLE_STDDEV: f64 = 1e-12;

    fn accelerometer(gravity_world: Vector3<f64>) -> Accelerometer {
        biased_accelerometer(gravity_world, Vector3::zeros())
    }

    fn biased_accelerometer(gravity_world: Vector3<f64>, bias: Vector3<f64>) -> Accelerometer {
        Accelerometer::new(
            AccelerometerModel::new(bias, Vector3::repeat(NEGLIGIBLE_STDDEV)).unwrap(),
            gravity_world,
        )
    }

    fn gyroscope() -> Gyroscope {
        biased_gyroscope(Vector3::zeros())
    }

    fn biased_gyroscope(bias: Vector3<f64>) -> Gyroscope {
        Gyroscope::new(GyroscopeModel::new(bias, Vector3::repeat(NEGLIGIBLE_STDDEV)).unwrap())
    }

    /// A pose yawed +90° about world Up, so world East lands on the sensor's
    /// -Y axis. The un-inverted rotation would give +Y — this pose is what
    /// catches a `sample` impl passing the rotation the wrong way around.
    fn yawed_pose() -> Isometry3<f64> {
        Isometry3::rotation(Vector3::z() * FRAC_PI_2)
    }

    #[test]
    fn accelerometer_at_rest_reads_the_reaction_to_gravity() {
        let mut accel = accelerometer(Vector3::new(0.0, 0.0, -9.81));
        let mut rng = StdRng::seed_from_u64(1);

        let got = accel.sample(
            &GroundTruthState::default(),
            &Isometry3::identity(),
            &mut rng,
        );

        assert!((got.value - Vector3::new(0.0, 0.0, 9.81)).norm() < 1e-6);
    }

    #[test]
    fn accelerometer_sample_rotates_world_acceleration_into_the_sensor_frame() {
        let mut accel = accelerometer(Vector3::zeros());
        let truth = GroundTruthState {
            linear_acceleration: Vector3::new(2.0, 0.0, 0.0),
            ..GroundTruthState::default()
        };
        let mut rng = StdRng::seed_from_u64(1);

        let got = accel.sample(&truth, &yawed_pose(), &mut rng);

        assert!((got.value - Vector3::new(0.0, -2.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn accelerometer_sample_applies_the_configured_bias() {
        let mut accel = biased_accelerometer(Vector3::zeros(), Vector3::new(0.3, 0.0, 0.0));
        let mut rng = StdRng::seed_from_u64(1);

        let got = accel.sample(
            &GroundTruthState::default(),
            &Isometry3::identity(),
            &mut rng,
        );

        assert!((got.value - Vector3::new(0.3, 0.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn gyroscope_sample_applies_the_configured_bias() {
        // A stationary gyro with a bias still reports a rate — the drift that
        // dominates dead reckoning.
        let mut gyro = biased_gyroscope(Vector3::new(0.0, 0.0, 0.02));
        let mut rng = StdRng::seed_from_u64(1);

        let got = gyro.sample(
            &GroundTruthState::default(),
            &Isometry3::identity(),
            &mut rng,
        );

        assert!((got.value - Vector3::new(0.0, 0.0, 0.02)).norm() < 1e-6);
    }

    #[test]
    fn gyroscope_sample_reads_the_world_angular_velocity() {
        let mut gyro = gyroscope();
        let truth = GroundTruthState {
            angular_velocity: Vector3::new(0.1, 0.2, 0.3),
            ..GroundTruthState::default()
        };
        let mut rng = StdRng::seed_from_u64(1);

        let got = gyro.sample(&truth, &Isometry3::identity(), &mut rng);

        assert!((got.value - Vector3::new(0.1, 0.2, 0.3)).norm() < 1e-6);
    }

    #[test]
    fn gyroscope_sample_rotates_the_rate_into_the_sensor_frame() {
        let mut gyro = gyroscope();
        let truth = GroundTruthState {
            angular_velocity: Vector3::new(1.0, 0.0, 0.0),
            ..GroundTruthState::default()
        };
        let mut rng = StdRng::seed_from_u64(1);

        let got = gyro.sample(&truth, &yawed_pose(), &mut rng);

        assert!((got.value - Vector3::new(0.0, -1.0, 0.0)).norm() < 1e-6);
    }
}
