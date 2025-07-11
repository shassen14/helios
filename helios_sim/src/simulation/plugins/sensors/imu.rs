// helios_sim/src/simulation/plugins/sensors/imu.rs

use crate::{
    prelude::*,
    simulation::core::{
        app_state::SimulationSet,
        events::BevyMeasurementEvent,
        prng::SimulationRng,
        topics::{GroundTruthState, ImuData, TopicBus, TopicTag},
        transforms::bevy_global_transform_to_nalgebra_isometry,
    },
}; // Use the sim prelude
use avian3d::prelude::Gravity;
use nalgebra::{DMatrix, DVector, Isometry3, UnitQuaternion, Vector3};
use rand_distr::{Distribution, Normal};
use std::time::Duration;

// --- Bevy Components for the IMU ---
/// This component is attached to a sensor child entity. It contains the
/// Bevy-specific timer and the topic name for publishing.
#[derive(Component)]
pub struct Imu {
    pub topic_to_publish: String,
    pub timer: Timer,
}

// --- The Bevy Plugin ---
pub struct ImuPlugin;

impl Plugin for ImuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            // This system processes the request and spawns IMU child entities.
            spawn_imu_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            // This system runs at runtime to generate sensor data.
            imu_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// --- Spawning System ---
fn spawn_imu_sensors(
    mut commands: Commands,
    mut topic_bus: ResMut<TopicBus>,
    request_query: Query<(Entity, &SpawnAgentConfigRequest)>,
) {
    for (agent_entity, request) in &request_query {
        let agent_config = &request.0;

        // This takes the first mutable borrow of `commands`.
        // The `agent_entity_commands` variable lives only until the end of this iteration.
        let mut agent_entity_commands = commands.entity(agent_entity);

        // This closure correctly uses the `parent_builder` argument.
        agent_entity_commands.with_children(|parent_builder| {
            for sensor_config in &agent_config.sensors {
                if let SensorConfig::Imu(imu_config) = sensor_config {
                    // --- Step 1: Spawn the child and get its ID and commands ---
                    // `parent_builder.spawn()` creates the new entity and returns a
                    // mutable `EntityCommands` handle that refers to the NEW CHILD.
                    let mut sensor_entity_commands = parent_builder.spawn(());
                    let sensor_entity = sensor_entity_commands.id(); // Get the unique ID

                    info!(
                        "  -> Spawning IMU '{}' (Entity {:?}) as child of agent {:?}",
                        imu_config.get_name(),
                        sensor_entity,
                        agent_entity
                    );

                    // --- Step 2: Prepare all necessary data and models ---
                    let agent_handle = FrameHandle::from_entity(agent_entity);
                    let sensor_handle = FrameHandle::from_entity(sensor_entity);

                    let (accel_std, gyro_std) = imu_config.get_noise_stddevs();
                    let r_matrix = DMatrix::from_diagonal(&DVector::from_vec(vec![
                        (accel_std[0] as f64).powi(2),
                        (accel_std[1] as f64).powi(2),
                        (accel_std[2] as f64).powi(2),
                        (gyro_std[0] as f64).powi(2),
                        (gyro_std[1] as f64).powi(2),
                        (gyro_std[2] as f64).powi(2),
                    ]));

                    // Create the pure mathematical model.
                    let model = Imu6DofModel {
                        agent_handle,
                        sensor_handle,
                        r_matrix,
                    };

                    // Create the topic for this sensor.
                    let topic_name =
                        format!("/agent/{}/imu/{}", agent_config.name, imu_config.get_name());
                    topic_bus.create_topic::<ImuData>(
                        &topic_name,
                        100,
                        TopicTag::Imu,
                        agent_config.name.clone(),
                    );

                    // --- Step 3: Use the child's EntityCommands to insert all components ---
                    // This is the key. We are not using the outer `commands` variable here.
                    // We are using the handle to the child we just created.
                    sensor_entity_commands.insert((
                        Name::new(imu_config.get_name().to_string()),
                        MeasurementModel(Box::new(model)), // The pure model
                        Imu {
                            // The Bevy-specific component
                            topic_to_publish: topic_name,
                            timer: Timer::new(
                                Duration::from_secs_f32(1.0 / imu_config.get_rate()),
                                TimerMode::Repeating,
                            ),
                        },
                        TrackedFrame, // So the TfTree can see it
                        imu_config.get_relative_pose().to_bevy_transform(),
                    ));
                }
            }
        });
    }
}

// --- Runtime System ---
// The runtime system that generates sensor data.
// It uses the modern, parent-centric query pattern.
fn imu_sensor_system(
    // --- Resources ---
    mut measurement_writer: EventWriter<BevyMeasurementEvent>,
    time: Res<Time>,
    gravity: Res<Gravity>,
    mut rng: ResMut<SimulationRng>,

    // --- Queries ---
    // QUERY 1: Find all parent agents that have children and a GroundTruthState.
    // This is our main loop iterator.
    parent_query: Query<(Entity, &GroundTruthState, &Children)>,

    // QUERY 2: This query gives us access to the components on the child sensor entities.
    // We will use `get_mut` on this query inside the loop.
    mut sensor_query: Query<(&mut Imu, &GlobalTransform, &MeasurementModel)>,
) {
    // 1. Iterate over each parent agent that has children.
    for (agent_entity, ground_truth, children) in &parent_query {
        // 2. Iterate through the list of child entities for this specific agent.
        for &child_entity in children {
            // `child_entity` is the Entity ID of a potential sensor.

            // 3. Use `get_mut` to check if this child is an IMU sensor.
            // This is how we link the parent to the specific child component.
            if let Ok((mut imu, sensor_global_transform, _measurement_model)) =
                sensor_query.get_mut(child_entity)
            {
                // SUCCESS! We have found an IMU child belonging to the current agent.

                imu.timer.tick(time.delta());
                if !imu.timer.just_finished() {
                    continue; // Skip to the next sensor if this one isn't ready.
                }

                let sensor_world_pose: Isometry3<f64> =
                    bevy_global_transform_to_nalgebra_isometry(sensor_global_transform);

                // All the logic for calculating the measurement is the same.
                let sensor_world_rotation: UnitQuaternion<f64> = sensor_world_pose.rotation;
                let q_body_from_world = sensor_world_rotation.inverse();

                let perfect_gyro = q_body_from_world * ground_truth.angular_velocity;
                let proper_accel_world =
                    ground_truth.linear_acceleration - Vector3::new(0.0, gravity.0.y as f64, 0.0);
                let perfect_accel = q_body_from_world * proper_accel_world;

                // For this test, we create a 6-element measurement vector.
                let z = DVector::from_vec(vec![
                    perfect_accel.x,
                    perfect_accel.y,
                    perfect_accel.z,
                    perfect_gyro.x,
                    perfect_gyro.y,
                    perfect_gyro.z,
                ]);

                // We can add noise here if desired.
                // let z_noisy = ...;

                info!(
                    "IMU on child {:?} publishing measurement for agent {:?} with data: {}",
                    child_entity, agent_entity, z
                );

                // Create the pure data packet.
                let pure_data = MeasurementEvent {
                    agent_handle: FrameHandle::from_entity(agent_entity),
                    sensor_handle: FrameHandle::from_entity(child_entity),
                    z, // Use perfect `z` for now for easier debugging
                    timestamp: time.elapsed_secs_f64(),
                };

                // Send the Bevy event.
                measurement_writer.write(BevyMeasurementEvent(pure_data));
            }
        }
    }
}
