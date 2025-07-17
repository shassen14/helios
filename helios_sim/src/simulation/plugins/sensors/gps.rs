use bevy::prelude::*;
use nalgebra::{DMatrix, Vector3};
use rand_distr::{Distribution, Normal};
use std::time::Duration;

// --- Simulation Crate Imports ---
use crate::prelude::*;
use crate::simulation::core::{
    app_state::SimulationSet, events::BevyMeasurementMessage, prng::SimulationRng,
    topics::GroundTruthState,
};

// --- Core Library Imports ---
use helios_core::{
    messages::{MeasurementData, MeasurementMessage},
    models::estimation::measurement::{gps::GpsPositionModel, Measurement},
    types::FrameHandle,
};

// =========================================================================
// == GPS Components & Plugin ==
// =========================================================================

/// A Bevy component attached to a GPS sensor entity, containing its runtime state.
#[derive(Component)]
pub struct Gps {
    pub timer: Timer,
    // Store the noise distribution for efficiency
    noise_dist: Normal<f64>,
}

pub struct GpsPlugin;

impl Plugin for GpsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_gps_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            gps_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

/// Reads the config and spawns GPS sensor entities as children of the appropriate agent.
fn spawn_gps_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, agent_name, request) in &request_query {
        for sensor_config in &request.0.sensors {
            if let SensorConfig::Gps(gps_config) = sensor_config {
                info!(
                    "  -> Spawning GPS '{}' as child of agent '{}'",
                    &gps_config.name,
                    agent_name.as_str()
                );

                // --- 1. Create the `helios_core` Measurement Model ---
                // This model contains the physics and noise characteristics of the sensor.

                // Create the 3x3 measurement noise covariance matrix R.
                let r_matrix = DMatrix::from_diagonal(&nalgebra::DVector::from_vec(vec![
                    gps_config.noise_stddev[0].powi(2) as f64,
                    gps_config.noise_stddev[1].powi(2) as f64,
                    gps_config.noise_stddev[2].powi(2) as f64,
                ]));

                // Get the antenna's physical offset from the config.
                let antenna_offset_body = gps_config.get_relative_pose().translation;

                let core_model = GpsPositionModel {
                    r_matrix,
                    antenna_offset_body,
                };

                // --- 2. Spawn the Sensor Entity as a Child ---
                let mut sensor_entity_commands = commands.spawn_empty();
                let sensor_entity = sensor_entity_commands.id();

                sensor_entity_commands.insert((
                    Name::new(gps_config.name.clone()),
                    // The Bevy component with the runtime timer.
                    Gps {
                        timer: Timer::new(
                            Duration::from_secs_f32(1.0 / gps_config.rate),
                            TimerMode::Repeating,
                        ),
                        // We could create separate noise distributions for each axis if needed.
                        // For simplicity, we'll just use the first standard deviation value.
                        noise_dist: Normal::new(0.0, gps_config.noise_stddev[0] as f64).unwrap(),
                    },
                    // The crucial part: The wrapped `helios_core` model.
                    // This is what the world model spawner will look for.
                    MeasurementModel(Box::new(core_model)),
                    TrackedFrame, // Mark it for the TF system
                    // Its local transform relative to the parent (the agent).
                    gps_config.get_relative_pose().to_bevy_transform(),
                ));

                // --- 3. Add the sensor as a child of the agent ---
                commands.entity(agent_entity).add_child(sensor_entity);
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

/// Runs every frame to simulate GPS physics and publish measurement messages.
fn gps_sensor_system(
    mut measurement_writer: EventWriter<BevyMeasurementMessage>,
    time: Res<Time>,
    mut rng: ResMut<SimulationRng>,

    // We need the GlobalTransform of the parent (agent) to calculate the true
    // position of the antenna.
    parent_query: Query<(Entity, &GlobalTransform, &Children)>,
    mut sensor_query: Query<(Entity, &mut Gps, &GlobalTransform)>,
) {
    let dt = time.delta();

    for (agent_entity, agent_transform, children) in &parent_query {
        for &child_entity in children {
            // Check if this child is a GPS sensor we need to process.
            if let Ok((sensor_entity, mut gps, sensor_global_transform)) =
                sensor_query.get_mut(child_entity)
            {
                gps.timer.tick(dt);
                if !gps.timer.just_finished() {
                    continue;
                }

                // --- Simulate GPS Measurement ---

                // 1. Get the sensor's true position in the Bevy world.
                let true_position_bevy = sensor_global_transform.translation();

                // 2. Convert it to our ENU coordinate system.
                let true_position_enu =
                    crate::simulation::core::transforms::bevy_vector_to_enu_vector(
                        &true_position_bevy,
                    );

                // 3. Add Gaussian noise to simulate GPS inaccuracy.
                let noisy_position_enu = Vector3::new(
                    true_position_enu.x + gps.noise_dist.sample(&mut rng.0),
                    true_position_enu.y + gps.noise_dist.sample(&mut rng.0),
                    true_position_enu.z + gps.noise_dist.sample(&mut rng.0),
                );

                // 4. Create the pure `MeasurementMessage`.
                let pure_message = MeasurementMessage {
                    agent_handle: FrameHandle::from_entity(agent_entity),
                    sensor_handle: FrameHandle::from_entity(sensor_entity),
                    timestamp: time.elapsed_secs_f64(),
                    data: MeasurementData::GpsPosition(noisy_position_enu),
                };

                // println!("Gps message: {:?}", pure_message.data);

                // 5. Wrap it in the Bevy event and send it.
                measurement_writer.write(BevyMeasurementMessage(pure_message));
            }
        }
    }
}
