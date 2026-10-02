//! Point-cloud view: every declared `PointCloud<Flu>` output, drawn where the
//! sensor's rays actually landed. Off at startup and flipped by the
//! `viz.toggle_point_cloud` action.
//!
//! Points are placed on the sensor's *true* pose at the instant of capture, read
//! from its [`LastCapturePose`], and never on its pose now: a scan on the bus can
//! be a full capture period old, and placing it on the current pose would slide
//! it with a moving agent. A reading whose capture pose is not the one recorded
//! is skipped for the frame rather than drawn somewhere wrong.
//!
//! The clouds come from [`declared_outputs`], so only node outputs are found:
//! a stack must deproject its `RangeField` to a cloud for it to be drawn here.

use crate::{
    agents::sensors::capture_pose::LastCapturePose,
    core::transforms::{point_bevy_to_vec3, ToBevy},
    prelude::{AutonomyPipelineComponent, TrackedFrame},
    viz::{
        interaction::{
            actions::{
                handle::{ActionHandle, ActionId},
                registry::ActionRegistry,
            },
            sampling::ActionState,
        },
        live::discovery::declared_outputs,
    },
};

use helios_core::{
    prelude::{MonotonicTime, PointCloud, SensorReading},
    spatial::{conventions::Flu, FrameId},
};

use bevy::prelude::*;
use serde::Deserialize;

/// The action that shows and hides the point-cloud view.
pub const TOGGLE_POINT_CLOUD: ActionId = ActionId("viz.toggle_point_cloud");

/// Gizmo group for the points, so they have their own on/off switch.
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct PointCloudGizmos;

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct PointCloudViewTuningFile {
    pub color: Option<[f32; 3]>,
    pub point_radius: Option<f32>,
}

/// Look of the point-cloud view: one operator preference for the session, the
/// same on every point.
#[derive(Resource, Debug, Clone)]
pub struct PointCloudViewTuning {
    /// Color of every point. Distinct from the bounding-box cyan and the path
    /// amber, which can be on screen at the same time.
    pub color: Color,
    /// Radius of the sphere drawn at each point, in meters.
    pub point_radius: f32,
}

impl Default for PointCloudViewTuning {
    fn default() -> Self {
        Self {
            color: Color::srgb(0.5, 0.3, 0.7),
            point_radius: 0.05,
        }
    }
}

impl PointCloudViewTuning {
    /// Overlays sparse overrides onto [`Default`], packing the `[r, g, b]`
    /// triple into an sRGB [`Color`].
    pub(crate) fn resolve(overrides: &PointCloudViewTuningFile) -> Self {
        let mut t = Self::default();
        if let Some([r, g, b]) = overrides.color {
            t.color = Color::srgb(r, g, b);
        }
        if let Some(radius) = overrides.point_radius {
            t.point_radius = radius;
        }
        t
    }
}

/// The pose `sensor` captured the reading stamped `timestamp` from, or `None`
/// when no sensor has that frame, it has not captured yet, or its recorded
/// capture is a different one. There is no fallback pose: a reading that cannot
/// be placed exactly is not placed.
fn capture_pose_for<'a>(
    sensor: &FrameId,
    timestamp: MonotonicTime,
    sensors: impl IntoIterator<Item = (&'a TrackedFrame, &'a LastCapturePose)>,
) -> Option<&'a GlobalTransform> {
    let (_, last) = sensors
        .into_iter()
        .find(|(tracked, _)| tracked.id == *sensor)?;

    last.latest
        .as_ref()
        .filter(|capture| capture.timestamp == timestamp)
        .map(|capture| &capture.pose)
}

/// Carries every point of `cloud` from its sensor's FLU frame into the world,
/// by the same steps the raycasting sensor casts its rays: FLU to Bevy axes,
/// then the sensor's capture pose.
fn world_points(cloud: &PointCloud<Flu>, pose: &GlobalTransform) -> Vec<Vec3> {
    (0..cloud.len())
        .map(|i| pose.transform_point(point_bevy_to_vec3(cloud.point(i).to_bevy())))
        .collect()
}

pub(crate) fn draw_point_clouds(
    pipelines: Query<&AutonomyPipelineComponent>,
    sensors: Query<(&TrackedFrame, &LastCapturePose)>,
    tuning: Res<PointCloudViewTuning>,
    mut gizmos: Gizmos<PointCloudGizmos>,
) {
    for pipeline in &pipelines {
        for batch in declared_outputs::<Vec<SensorReading<PointCloud<Flu>>>>(&pipeline.0) {
            for reading in &batch.value {
                let Some(pose) = capture_pose_for(&reading.sensor, reading.timestamp, &sensors)
                else {
                    continue;
                };
                for point in world_points(&reading.data, pose) {
                    gizmos.sphere(
                        Isometry3d::from_translation(point),
                        tuning.point_radius,
                        tuning.color,
                    );
                }
            }
        }
    }
}

pub(crate) fn toggle_point_cloud(
    registry: Res<ActionRegistry>,
    state: Res<ActionState>,
    mut store: ResMut<GizmoConfigStore>,
    mut handle: Local<Option<ActionHandle>>,
) {
    let h = *handle.get_or_insert_with(|| registry.handle(TOGGLE_POINT_CLOUD).expect("registered"));

    if state.is_active(h) {
        let (config, _) = store.config_mut::<PointCloudGizmos>();
        config.enabled = !config.enabled;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        agents::sensors::capture_pose::CapturePose,
        core::transforms::freevector_bevy_to_vec3,
        viz::interaction::{
            actions::handle::{ActionMetadata, InputKind},
            registration::VIZ_GROUP,
        },
    };

    use helios_core::{
        prelude::{AgentId, PointCloudBuilder},
        spatial::{
            quantities::{FreeVector, Point},
            transforms::Convention,
        },
    };

    use nalgebra::Vector3;
    use std::f32::consts::FRAC_PI_3;

    const AGENT: &str = "agent";
    const FRONT_LIDAR: &str = "sensor.lidar.front";
    const REAR_LIDAR: &str = "sensor.lidar.rear";
    const CAPTURE_TIME: MonotonicTime = MonotonicTime(1.5);
    const LATER_TIME: MonotonicTime = MonotonicTime(1.6);
    const TOLERANCE: f32 = 1e-5;

    fn frame(name: &str) -> FrameId {
        FrameId::sensor(AgentId::new(AGENT), name.to_string())
    }

    fn tracked(name: &str) -> TrackedFrame {
        TrackedFrame::new(frame(name), Convention::Flu)
    }

    fn captured(pose: GlobalTransform, timestamp: MonotonicTime) -> LastCapturePose {
        LastCapturePose {
            latest: Some(CapturePose { pose, timestamp }),
        }
    }

    /// A sensor pose with every component nonzero, so a placement that drops
    /// the translation or the rotation, or applies them in the wrong order,
    /// lands somewhere else.
    fn tilted_pose() -> GlobalTransform {
        GlobalTransform::from(
            Transform::from_xyz(3.0, 0.5, -2.0).with_rotation(Quat::from_euler(
                EulerRot::YXZ,
                FRAC_PI_3,
                0.2,
                -0.1,
            )),
        )
    }

    /// The matching sensor's recorded pose comes back when the timestamps
    /// agree, and from the right sensor when the agent has two.
    #[test]
    fn finds_the_pose_of_the_sensor_that_captured_the_reading() {
        let front_pose = tilted_pose();
        let rear_pose = GlobalTransform::IDENTITY;
        let (front, rear) = (tracked(FRONT_LIDAR), tracked(REAR_LIDAR));
        let (front_capture, rear_capture) = (
            captured(front_pose, CAPTURE_TIME),
            captured(rear_pose, CAPTURE_TIME),
        );

        let pose = capture_pose_for(
            &frame(FRONT_LIDAR),
            CAPTURE_TIME,
            [(&rear, &rear_capture), (&front, &front_capture)],
        );

        assert_eq!(pose, Some(&front_pose));
    }

    /// A recorded capture from a different scan is not used: the reading is
    /// skipped instead of being drawn on the wrong pose.
    #[test]
    fn a_capture_from_another_scan_is_not_used() {
        let front = tracked(FRONT_LIDAR);
        let capture = captured(tilted_pose(), LATER_TIME);

        let pose = capture_pose_for(&frame(FRONT_LIDAR), CAPTURE_TIME, [(&front, &capture)]);

        assert_eq!(pose, None);
    }

    /// A sensor that has not captured yet has no pose to offer.
    #[test]
    fn no_pose_before_the_first_capture() {
        let front = tracked(FRONT_LIDAR);
        let capture = LastCapturePose::default();

        let pose = capture_pose_for(&frame(FRONT_LIDAR), CAPTURE_TIME, [(&front, &capture)]);

        assert_eq!(pose, None);
    }

    /// A reading naming a frame no sensor carries has no pose either.
    #[test]
    fn no_pose_for_an_unknown_sensor() {
        let rear = tracked(REAR_LIDAR);
        let capture = captured(tilted_pose(), CAPTURE_TIME);

        let pose = capture_pose_for(&frame(FRONT_LIDAR), CAPTURE_TIME, [(&rear, &capture)]);

        assert_eq!(pose, None);
    }

    /// Each point lands where the raycasting sensor's ray hit: the sensor
    /// origin plus the measured range along the beam, the beam turned from FLU
    /// into the world by the same steps the cast uses. Agreement with the cast
    /// is what keeps drawn points on the surfaces the rays struck.
    #[test]
    fn points_land_where_the_cast_rays_hit() {
        let pose = tilted_pose();
        let beams = [
            (Vector3::new(1.0, 0.0, 0.0), 4.0),
            (Vector3::new(0.0, 1.0, 0.0), 2.5),
            (Vector3::new(0.6, -0.8, 0.0), 7.0),
        ];

        let mut builder = PointCloudBuilder::<Flu>::default();
        for (direction, range) in beams {
            builder.push(Point::from_raw(direction * range));
        }
        let cloud = builder.finalize().expect("geometry-only cloud builds");

        let placed = world_points(&cloud, &pose);

        assert_eq!(placed.len(), beams.len());
        for ((direction, range), point) in beams.into_iter().zip(placed) {
            let world_direction = pose.rotation()
                * freevector_bevy_to_vec3(FreeVector::<Flu>::from_raw(direction).to_bevy());
            let hit = pose.translation() + world_direction * range as f32;
            assert!(
                point.abs_diff_eq(hit, TOLERANCE),
                "point at {point}, ray hit at {hit}"
            );
        }
    }

    /// A store holding the point group as `VizPlugin` configures it: switched
    /// off.
    fn point_store() -> GizmoConfigStore {
        let mut store = GizmoConfigStore::default();
        store.insert(
            GizmoConfig {
                enabled: false,
                ..default()
            },
            PointCloudGizmos,
        );
        store
    }

    fn point_view_enabled(app: &mut App) -> bool {
        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        store.config_mut::<PointCloudGizmos>().0.enabled
    }

    fn toggle_app(active: bool) -> App {
        let mut registry = ActionRegistry::default();
        let handle = registry.register(
            TOGGLE_POINT_CLOUD,
            ActionMetadata {
                label: "Toggle point cloud",
                group: VIZ_GROUP,
                kind: InputKind::Button,
                default_key: KeyCode::KeyL,
            },
        );
        let firing = if active { vec![handle] } else { Vec::new() };

        let mut app = App::new();
        app.insert_resource(registry);
        app.insert_resource(ActionState::from_active(firing));
        app.insert_resource(point_store());
        app.add_systems(Update, toggle_point_cloud);
        app
    }

    /// Each firing of the action flips the view, on then off.
    #[test]
    fn toggle_flips_the_point_view_each_time_the_action_fires() {
        let mut app = toggle_app(true);

        app.update();
        assert!(point_view_enabled(&mut app), "first press turns it on");
        app.update();
        assert!(!point_view_enabled(&mut app), "second press turns it off");
    }

    /// With no action firing, the view stays as it is.
    #[test]
    fn toggle_leaves_the_point_view_alone_when_the_action_is_idle() {
        let mut app = toggle_app(false);

        app.update();
        assert!(!point_view_enabled(&mut app));
    }

    /// A file with no overrides resolves to the compiled-in look.
    #[test]
    fn empty_file_resolves_to_defaults() {
        let t = PointCloudViewTuning::resolve(&PointCloudViewTuningFile::default());
        let defaults = PointCloudViewTuning::default();

        assert_eq!(t.color, defaults.color);
        assert_eq!(t.point_radius, defaults.point_radius);
    }

    /// Each override replaces only its own field.
    #[test]
    fn overrides_replace_their_fields() {
        const COLOR: [f32; 3] = [0.2, 0.4, 0.6];
        const RADIUS: f32 = 0.12;
        let file = PointCloudViewTuningFile {
            color: Some(COLOR),
            point_radius: Some(RADIUS),
        };

        let t = PointCloudViewTuning::resolve(&file);

        assert_eq!(t.color, Color::srgb(COLOR[0], COLOR[1], COLOR[2]));
        assert_eq!(t.point_radius, RADIUS);
    }
}
