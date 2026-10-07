use crate::agents::sensors::capture_pose::{CapturePose, LastCapturePose};
use crate::brain_bridge::components::SensorPublishChannel;
use crate::config::structs::SensorConfig;
use crate::core::app_state::SimulationSet;
use crate::core::prng::{MasterSeed, SensorRng};
use crate::core::transforms::{freevector_bevy_to_vec3, ToBevy};
use crate::prelude::*;

use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::prelude::SphericalAngular;
use helios_core::sensors::lidar::{LidarModel, LidarNoise};
use helios_core::sensors::{RayHit, RaycastingOutput, RaycastingSensorModel};
use helios_core::spatial::conventions::Flu;
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::quantities::FreeVector;
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

use avian3d::prelude::{ColliderOf, SpatialQuery, SpatialQueryFilter};
use std::time::Duration;

// =========================================================================
// == Components & Plugin ==
// =========================================================================

#[derive(Component)]
pub struct RaycastingSensor {
    pub timer: Timer,
    pub model: Box<dyn RaycastingSensorModel>,
}

pub struct RaycastingSensorPlugin;

impl Plugin for RaycastingSensorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            OnEnter(AppState::SceneBuilding),
            spawn_raycasting_sensors.in_set(SceneBuildSet::ProcessSensors),
        )
        .add_systems(
            FixedUpdate,
            raycasting_sensor_system.in_set(SimulationSet::Sensors),
        );
    }
}

// =========================================================================
// == Spawning System ==
// =========================================================================

fn spawn_raycasting_sensors(
    mut commands: Commands,
    request_query: Query<(Entity, &Name, &SpawnAgentConfigRequest, &AgentIdComponent)>,
    master_seed: Res<MasterSeed>,
) {
    for (agent_entity, agent_name, request, agent_id) in &request_query {
        for (sensor_name, sensor_config) in &request.0.sensors {
            if let SensorConfig::Lidar(lidar_config) = sensor_config {
                info!(
                    "  -> Spawning LiDAR '{}' as RaycastingSensor for agent '{}' with rate of {:.1} Hz",
                    sensor_name,
                    agent_name.as_str(),
                    lidar_config.get_rate()
                );

                let ring_elevations: Vec<f64> = lidar_config
                    .ring_elevations
                    .iter()
                    .map(|deg| deg.to_radians())
                    .collect();

                let Some(geometry) = SphericalAngular::from_field_of_view(
                    ring_elevations,
                    lidar_config.azimuth_fov.to_radians(),
                    lidar_config.azimuth_beams,
                ) else {
                    error!(
                        "LiDAR '{}' has an unusable beam layout (azimuth_fov={}, azimuth_beams={}, ring_elevations={:?}): the field of view must be finite and non-negative, with at least one azimuth beam and one ring. Skipping sensor.",
                        sensor_name,
                        lidar_config.azimuth_fov,
                        lidar_config.azimuth_beams,
                        lidar_config.ring_elevations,
                    );
                    continue;
                };

                let Some(noise) = LidarNoise::new(
                    lidar_config.range_noise_stddev as f64,
                    (lidar_config.angular_noise_stddev as f64).to_radians(),
                ) else {
                    error!(
                        "LiDAR '{}' has unusable noise (range_noise_stddev={}, angular_noise_stddev={}): both must be finite and > 0. Skipping sensor.",
                        sensor_name,
                        lidar_config.range_noise_stddev,
                        lidar_config.angular_noise_stddev,
                    );
                    continue;
                };

                let Some(model) = LidarModel::new(
                    geometry,
                    lidar_config.sweep_period,
                    lidar_config.range_min as f64,
                    lidar_config.max_range as f64,
                    noise,
                ) else {
                    error!(
                        "LiDAR '{}' has unusable range limits or sweep (range_min={}, max_range={}, sweep_period={}): limits must be finite with 0 <= range_min < max_range, and the sweep period non-negative. Skipping sensor.",
                        sensor_name,
                        lidar_config.range_min,
                        lidar_config.max_range,
                        lidar_config.sweep_period,
                    );
                    continue;
                };
                let core_model: Box<dyn RaycastingSensorModel> = Box::new(model);

                let mut sensor_entity_commands = commands.spawn_empty();
                let sensor_entity = sensor_entity_commands.id();

                let sensor_label = format!("{}/{}", agent_name.as_str(), sensor_name);
                let sensor_rng = SensorRng::from_sensor(master_seed.0, &sensor_label);

                sensor_entity_commands.insert((
                    Name::new(sensor_label),
                    SensorPublishChannel(lidar_config.get_channel().to_string()),
                    RaycastingSensor {
                        timer: Timer::new(
                            Duration::from_secs_f64(1.0 / lidar_config.get_rate()),
                            TimerMode::Repeating,
                        ),
                        model: core_model,
                    },
                    sensor_rng,
                    TrackedFrame::new(
                        FrameId::sensor(agent_id.0.clone(), lidar_config.get_channel().to_string()),
                        Convention::Flu,
                    ),
                    LastCapturePose::default(),
                    lidar_config.get_relative_pose().to_bevy_local_transform(),
                ));

                commands.entity(agent_entity).add_child(sensor_entity);
            }
        }
    }
}

// =========================================================================
// == Runtime System ==
// =========================================================================

fn raycasting_sensor_system(
    time: Res<Time>,
    spatial_query: SpatialQuery,
    collider_bodies: Query<&ColliderOf>,
    mut sensor_query: Query<(
        &mut RaycastingSensor,
        &mut SensorRng,
        &mut LastCapturePose,
        &SensorPublishChannel,
        &TrackedFrame,
        &GlobalTransform,
        &ChildOf,
    )>,
    mut publisher: SensorPublisher,
) {
    let _span = tracing::trace_span!("sim.sensor.publish", sensor = "raycasting").entered();
    let elapsed = time.elapsed_secs_f64();
    let dt = time.delta();

    for (
        mut sensor,
        mut rng,
        mut capture,
        sensor_publish_channel,
        tracked,
        sensor_transform,
        parent,
    ) in &mut sensor_query
    {
        sensor.timer.tick(dt);
        if !sensor.timer.just_finished() {
            continue;
        }

        let local_rays = sensor.model.generate_rays(&mut rng.0);
        let mut hits: Vec<RayHit> = Vec::with_capacity(local_rays.len());
        let sensor_origin = sensor_transform.translation();
        let sensor_rotation = sensor_transform.rotation();
        let max_toi = sensor.model.get_max_range();
        let agent = parent.parent();

        // Checked on the first scan, not at spawn: only now has the sensor's
        // world transform been propagated from its mount.
        if capture.latest.is_none() {
            let intersecting =
                spatial_query.point_intersections(sensor_origin, &SpatialQueryFilter::default());
            if is_inside_own_body(&intersecting, agent, |collider| {
                collider_bodies.get(collider).ok().map(|of| of.body)
            }) {
                warn!(
                    "Sensor '{}' is mounted inside its own agent's body (Bevy world position {:?}): \
                     every ray hits the body at zero range, so it will report almost no \
                     returns. Check its mount `transform` if this is not intended.",
                    tracked.id, sensor_origin,
                );
            }
        }

        // The agent's own body is not excluded: a real sensor sees the vehicle
        // it is mounted on, and the brain has to filter those returns itself.
        let filter = SpatialQueryFilter::default();

        for ray in local_rays {
            let bevy_local_dir = FreeVector::<Flu>::from_raw(ray.direction).to_bevy();

            let world_direction: Vec3 = sensor_rotation * freevector_bevy_to_vec3(bevy_local_dir);

            if let Ok(dir) = Dir3::new(world_direction) {
                if let Some(hit) =
                    spatial_query.cast_ray(sensor_origin, dir, max_toi, true, &filter)
                {
                    hits.push(RayHit {
                        ray_id: ray.id,
                        distance: hit.distance,
                    });
                }
            }
        }

        let output = sensor.model.process_hits(&hits, &mut rng.0);

        let capture_time = MonotonicTime(elapsed);

        capture.latest = Some(CapturePose {
            pose: *sensor_transform,
            timestamp: capture_time,
        });

        // Published as measured: flattening to a point cloud is the autonomy
        // stack's job (a `Deproject` node), so the same field
        // reaches the brain from sim and from a hardware driver alike.
        let RaycastingOutput::RangeField(field) = output;

        let reading = SensorReading {
            sensor: tracked.id.clone(),
            timestamp: capture_time,
            data: field,
        };

        publisher.publish(agent, sensor_publish_channel.0.as_str(), vec![reading]);
    }
}

/// Whether any collider in `intersecting` belongs to `agent`: either the agent
/// entity carries the collider itself, or `body_of` maps the collider to the
/// agent as its rigid body.
fn is_inside_own_body(
    intersecting: &[Entity],
    agent: Entity,
    body_of: impl Fn(Entity) -> Option<Entity>,
) -> bool {
    intersecting
        .iter()
        .any(|&collider| collider == agent || body_of(collider) == Some(agent))
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::{AgentId, RangeField, TfProvider};
    use helios_runtime::port::AlgorithmNodePortDescriptor;
    use helios_runtime::port::{PortBus, SensorChannel};
    use helios_runtime::{
        BodyCapabilities, ChannelKey, PipelineBuilder, PipelineNode, PortDescriptor, Provenance,
        PublishedChannel, Stamped, TickContext,
    };

    use avian3d::collider_tree::ColliderTrees;
    use bevy::ecs::system::RunSystemOnce;
    use std::f64::consts::TAU;
    use std::sync::Arc;

    const CHANNEL: &str = "sensor.lidar.test";
    const RATE_HZ: f64 = 10.0;
    const AZIMUTH_BEAMS: u32 = 8;
    const RANGE_MIN: f64 = 0.1;
    const MAX_RANGE: f64 = 20.0;
    const RANGE_NOISE_STDDEV: f64 = 0.01;
    const ANGULAR_NOISE_STDDEV: f64 = 0.001;
    /// A zero sweep period: every beam shares the reading's instant.
    const FLASH_SWEEP_PERIOD: f64 = 0.0;
    const SEED: u64 = 7;

    type FieldBatch = Vec<SensorReading<RangeField<Flu>>>;

    /// Declares the lidar channel as an optional input, which is all it takes
    /// for the bus to reserve a slot the sensor can publish into.
    struct FakeScanConsumer {
        descriptor: PortDescriptor,
    }

    impl FakeScanConsumer {
        fn new() -> Self {
            Self {
                descriptor: AlgorithmNodePortDescriptor::new()
                    .inputs_from_slices(&[], &[field_channel()])
                    .build(),
            }
        }
    }

    impl PipelineNode for FakeScanConsumer {
        fn name(&self) -> &str {
            "fake_scan_consumer"
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
    }

    fn field_channel() -> ChannelKey {
        SensorChannel::named::<FieldBatch>(CHANNEL).into()
    }

    fn period() -> Duration {
        Duration::from_secs_f64(1.0 / RATE_HZ)
    }

    fn lidar_model() -> Box<dyn RaycastingSensorModel> {
        let geometry = SphericalAngular::from_field_of_view(vec![0.0], TAU, AZIMUTH_BEAMS)
            .expect("a single full-circle ring is a valid layout");
        let noise =
            LidarNoise::new(RANGE_NOISE_STDDEV, ANGULAR_NOISE_STDDEV).expect("positive noise");
        let model = LidarModel::new(geometry, FLASH_SWEEP_PERIOD, RANGE_MIN, MAX_RANGE, noise)
            .expect("valid range limits");
        Box::new(model)
    }

    /// A world holding one agent with a pipeline that consumes `CHANNEL` (the
    /// body declares it, so the build finds a supplier), one lidar mounted on
    /// it at `sensor_pose`, empty collider trees (every ray misses), and a
    /// clock advanced by `advance`. Returns the world and the sensor entity.
    fn world_with_lidar(sensor_pose: GlobalTransform, advance: Duration) -> (World, Entity) {
        let mut world = World::new();

        let mut time = Time::<()>::default();
        time.advance_by(advance);
        world.insert_resource(time);
        world.insert_resource(ColliderTrees::default());

        let pipeline = PipelineBuilder::new()
            .add_node(Box::new(FakeScanConsumer::new()))
            .with_body_capabilities(BodyCapabilities {
                name: "agent".to_string(),
                publishes: vec![PublishedChannel {
                    key: field_channel(),
                    provenance: Provenance::Exact,
                }],
                ..Default::default()
            })
            .build()
            .expect("a single-node pipeline builds");
        let agent = world.spawn(AutonomyPipelineComponent(pipeline)).id();

        let sensor = world
            .spawn((
                RaycastingSensor {
                    timer: Timer::new(period(), TimerMode::Repeating),
                    model: lidar_model(),
                },
                SensorRng::from_sensor(SEED, CHANNEL),
                LastCapturePose::default(),
                SensorPublishChannel(CHANNEL.to_string()),
                TrackedFrame::new(
                    FrameId::sensor(AgentId::new("agent"), CHANNEL.to_string()),
                    Convention::Flu,
                ),
                sensor_pose,
                ChildOf(agent),
            ))
            .id();

        (world, sensor)
    }

    fn agent_bus_field(world: &mut World) -> Option<Arc<Stamped<FieldBatch>>> {
        let mut query = world.query::<&AutonomyPipelineComponent>();
        let pipeline = query.single(world).expect("exactly one agent");
        pipeline.0.bus().read::<FieldBatch>(field_channel())
    }

    /// The viz pairs a capture pose with a reading by exact timestamp equality,
    /// so the two must be written from one value, and the pose must be the one
    /// the rays were cast from.
    #[test]
    fn capture_pose_matches_the_published_reading() {
        let sensor_pose = GlobalTransform::from_translation(Vec3::new(3.0, 0.5, -2.0));
        let (mut world, sensor) = world_with_lidar(sensor_pose, period());

        world
            .run_system_once(raycasting_sensor_system)
            .expect("the system runs");

        let batch = agent_bus_field(&mut world).expect("the scan was published");
        let reading = batch.value.last().expect("one reading in the batch");
        let capture = world
            .get::<LastCapturePose>(sensor)
            .and_then(|c| c.latest.clone())
            .expect("the scan recorded a capture pose");

        assert_eq!(capture.timestamp, reading.timestamp);
        assert_eq!(capture.pose, sensor_pose);
    }

    /// The component holds the *newest* capture: a second scan replaces the
    /// first, and still pairs with the reading now on the bus.
    #[test]
    fn a_later_scan_replaces_the_capture() {
        let (mut world, sensor) = world_with_lidar(GlobalTransform::IDENTITY, period());
        world
            .run_system_once(raycasting_sensor_system)
            .expect("the first scan runs");
        let first = world
            .get::<LastCapturePose>(sensor)
            .and_then(|c| c.latest.clone())
            .expect("the first scan recorded a capture pose");

        let moved_pose = GlobalTransform::from_translation(Vec3::new(1.0, 0.0, 4.0));
        world.resource_mut::<Time>().advance_by(period());
        *world
            .get_mut::<GlobalTransform>(sensor)
            .expect("the sensor has a transform") = moved_pose;
        world
            .run_system_once(raycasting_sensor_system)
            .expect("the second scan runs");

        let batch = agent_bus_field(&mut world).expect("the second scan was published");
        let reading = batch.value.last().expect("one reading in the batch");
        let second = world
            .get::<LastCapturePose>(sensor)
            .and_then(|c| c.latest.clone())
            .expect("the second scan recorded a capture pose");

        assert!(second.timestamp > first.timestamp);
        assert_eq!(second.timestamp, reading.timestamp);
        assert_eq!(second.pose, moved_pose);
    }

    /// Until the sensor first fires there is no capture, and nothing on the bus
    /// to pair one with.
    #[test]
    fn no_capture_before_the_first_scan() {
        let (mut world, sensor) = world_with_lidar(GlobalTransform::IDENTITY, period() / 2);

        world
            .run_system_once(raycasting_sensor_system)
            .expect("the system runs");

        assert!(agent_bus_field(&mut world).is_none());
        let capture = world
            .get::<LastCapturePose>(sensor)
            .expect("inserted at spawn");
        assert!(capture.latest.is_none());
    }

    /// Three distinct entities standing in for the agent, a collider attached
    /// to it, and something else in the scene.
    fn agent_part_and_other() -> (Entity, Entity, Entity) {
        let mut world = World::new();
        (
            world.spawn_empty().id(),
            world.spawn_empty().id(),
            world.spawn_empty().id(),
        )
    }

    /// The agent entity carrying its collider directly, as the raycast car's
    /// cuboid does, encloses the sensor.
    #[test]
    fn collider_on_the_agent_itself_is_its_own_body() {
        let (agent, _, _) = agent_part_and_other();

        assert!(is_inside_own_body(&[agent], agent, |_| None));
    }

    /// A collider on a child entity is the agent's body when its rigid body is
    /// the agent.
    #[test]
    fn collider_attached_to_the_agent_is_its_own_body() {
        let (agent, part, _) = agent_part_and_other();

        assert!(is_inside_own_body(&[part], agent, |collider| {
            (collider == part).then_some(agent)
        }));
    }

    /// A sensor inside another body, such as a wall or another agent, is not
    /// enclosed by its own body; open air is not either.
    #[test]
    fn another_body_or_open_air_is_not_its_own_body() {
        let (agent, _, other) = agent_part_and_other();

        assert!(!is_inside_own_body(&[other], agent, |_| None));
        assert!(!is_inside_own_body(&[], agent, |_| None));
    }
}
