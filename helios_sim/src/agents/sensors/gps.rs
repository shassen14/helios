//! GPS sensor simulation: spawns receivers from agent configs and publishes
//! noisy ENU world-position fixes through the generic state-sensor system.

use super::state_sensor::{publish_state_sensor, StateSensor};

use crate::core::app_state::SimulationSet;
use crate::prelude::*;
use crate::{
    agents::sensors::state_sensor::SensorTimer, brain_bridge::components::SensorPublishChannel,
};

use helios_core::data::sensor::GpsPosition;
use helios_core::sensors::gps::GpsModel;

use nalgebra::{Isometry3, Vector3};
use rand::RngCore;

/// A simulated GPS receiver: wraps the core forward model, which reports the
/// sensor's ENU world position with a constant bias and per-axis noise.
#[derive(Component)]
pub struct Gps {
    model: GpsModel,
}

impl Gps {
    pub fn new(model: GpsModel) -> Self {
        Self { model }
    }
}

impl StateSensor for Gps {
    type Payload = GpsPosition;

    /// A GPS observes position directly, so only the pose's translation
    /// matters; the agent's kinematic state and the sensor's orientation go
    /// unused.
    fn sample(
        &mut self,
        _t: &GroundTruthState,
        p: &Isometry3<f64>,
        rng: &mut dyn RngCore,
    ) -> GpsPosition {
        self.model.sample(p.translation.vector, rng)
    }
}

/// Registers GPS simulation: spawning during scene build, publishing through
/// the generic state-sensor system while running.
pub struct GpsPlugin;

impl Plugin for GpsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_gps_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            publish_state_sensor::<Gps>.in_set(SimulationSet::Sensors),
        );
    }
}

/// Spawns one GPS child entity per GPS entry in each agent's spawn request.
///
/// Model construction doubles as config validation: [`GpsModel::new`] refuses
/// a stddev with any non-positive axis, so a bad config skips the sensor with
/// an error instead of spawning a degenerate one.
fn spawn_gps_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        for (sensor_name, sensor_config) in &request.0.sensors {
            if let SensorConfig::Gps(gps_config) = sensor_config {
                info!(
                    "  -> Spawning GPS '{}' as child of agent '{}'",
                    &sensor_name,
                    agent_name.as_str()
                );

                let bias = Vector3::from(gps_config.bias);
                let stddev = Vector3::from(gps_config.noise_stddev);

                let Some(model) = GpsModel::new(bias, stddev) else {
                    error!(
                        "GPS sensor '{}' on agent '{}' has invalid noise_stddev {:?}: \
                         every axis must be > 0. Skipping sensor.",
                        sensor_name,
                        agent_name.as_str(),
                        gps_config.noise_stddev
                    );
                    continue;
                };

                let sensor_entity = commands
                    .spawn((
                        Name::new(format!("{}/{}", agent_name.as_str(), gps_config.name)),
                        SensorPublishChannel(gps_config.channel.clone()),
                        Gps::new(model),
                        SensorTimer::from_rate(gps_config.rate),
                        TrackedFrame,
                        gps_config.get_relative_pose().to_bevy_local_transform(),
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

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    /// Small enough that a sample equals its ideal value within test
    /// tolerance, since the model refuses an exactly-zero stddev.
    const NEGLIGIBLE_STDDEV: f64 = 1e-12;

    fn gps(bias: Vector3<f64>) -> Gps {
        Gps::new(GpsModel::new(bias, Vector3::repeat(NEGLIGIBLE_STDDEV)).unwrap())
    }

    #[test]
    fn sample_reads_the_sensor_world_position() {
        let mut gps = gps(Vector3::zeros());
        let pose = Isometry3::translation(3.0, -4.0, 5.0);
        let mut rng = StdRng::seed_from_u64(1);

        let got = gps.sample(&GroundTruthState::default(), &pose, &mut rng);

        assert!((got.position - Vector3::new(3.0, -4.0, 5.0)).norm() < 1e-6);
    }

    #[test]
    fn sample_applies_the_configured_bias() {
        let mut gps = gps(Vector3::new(1.0, 2.0, 3.0));
        let pose = Isometry3::translation(0.0, 0.0, 0.0);
        let mut rng = StdRng::seed_from_u64(1);

        let got = gps.sample(&GroundTruthState::default(), &pose, &mut rng);

        assert!((got.position - Vector3::new(1.0, 2.0, 3.0)).norm() < 1e-6);
    }
}
