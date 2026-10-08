//! Registers the `RecursiveEstimator` kind, which assembles a
//! [`RecursiveEstimatorNode`] from the parts its `[nodes.<name>]` entry names.
//!
//! The filter, the dynamics and every aiding model come from the
//! [`EstimatorComponents`] tables; the node's own config adds only what ties
//! them together: the aiding wiring, the augmentation blocks and the starting
//! pose.

use super::config::{AidingEntry, AugmentationEntry, InitialPoseConfig, RecursiveEstimatorConfig};
use super::nis_health::NisWindow;
use super::node::{Aiding, RecursiveEstimatorNode};

use crate::assembly::{AutonomyRegistry, BuildContext, BuildFailure, FactoryOutput};
use crate::nodes::estimation::{
    DynamicsComponent, EstimatorComponents, FilterParts, MeasurementSource, MeasurementWiring,
};

use helios_core::estimation::augmentation::augmentation_block;
use helios_core::estimation::schema::{check_measurement_state_agreement, StateSchemaBlock};
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::transforms::tf::stamped::FrameEdge;
use helios_core::spatial::transforms::Transform;
use helios_core::spatial::{FrameAwareState, FrameId};

use nalgebra::{DMatrix, DVector, Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::sync::Arc;

/// The `kind` a profile writes to get a recursive estimator.
pub(crate) const RECURSIVE_ESTIMATOR_KIND: &str = "RecursiveEstimator";

/// The `filter` sub-table's key, which is also its path in errors and the dump.
const FILTER_KEY: &str = "filter";
/// The `dynamics` sub-table's key, likewise.
const DYNAMICS_KEY: &str = "dynamics";
/// The key of the aiding table, and the first segment of each model's path.
const AIDING_KEY: &str = "aiding";
/// The key of an aiding entry's model sub-table.
const MODEL_KEY: &str = "model";
/// The key of an aiding entry's NIS health check, named in its errors.
const NIS_HEALTH_KEY: &str = "nis_health";

/// Adds the `RecursiveEstimator` kind to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node_with::<EstimatorComponents, _, _, _>(RECURSIVE_ESTIMATOR_KIND, build)
        .expect("RecursiveEstimator is registered once, by the default registry")
}

/// Builds the node for one entry: the dynamics, then each aiding source, then
/// the state they agree on, seeded with the starting pose, then the filter
/// around it.
fn build(
    config: RecursiveEstimatorConfig,
    ctx: &BuildContext<'_>,
    components: &EstimatorComponents,
) -> Result<FactoryOutput, BuildFailure> {
    let dynamics = components.build_dynamics(DYNAMICS_KEY, config.dynamics, ctx)?;
    let mut resolved_aiding = Vec::with_capacity(config.aiding.len());
    let mut aiding = Vec::with_capacity(config.aiding.len());
    let mut nis_windows = Vec::with_capacity(config.aiding.len());
    let inputs: Vec<String> = config
        .aiding
        .values()
        .map(|entry| entry.input.clone())
        .collect();
    for (name, entry) in config.aiding {
        let nis_window = nis_window(&name, &entry)?;
        let (source, resolved) = build_aiding(components, &name, entry, ctx)?;
        aiding.push(source);
        nis_windows.push(nis_window);
        resolved_aiding.push((
            [AIDING_KEY.to_string(), name, MODEL_KEY.to_string()],
            resolved,
        ));
    }
    let augmentation = augmentation_blocks(&config.augmentation, &inputs, ctx)?;

    let initial_state = seeded_state(
        &dynamics.component,
        augmentation,
        &aiding,
        &config.initial_pose,
        ctx,
    )?;
    let (process, input) = dynamics.component.into_parts();
    let filter = components.build_filter(
        FILTER_KEY,
        config.filter,
        ctx,
        FilterParts::new(initial_state, process),
    )?;

    let agent = ctx.agent();
    let edge = FrameEdge {
        child: FrameId::base_link(agent.clone()),
        parent: FrameId::odom(agent.clone()),
    };
    let aiding = aiding
        .into_iter()
        .zip(nis_windows)
        .map(|(source, window)| match window {
            Some(window) => Aiding::new(source).with_nis_window(window),
            None => Aiding::new(source),
        })
        .collect();
    let node = RecursiveEstimatorNode::new(ctx.node_name(), edge, filter.component, input, aiding);

    let mut output = FactoryOutput::new(Box::new(node))
        .with_resolved_component([FILTER_KEY], filter.resolved)
        .with_resolved_component([DYNAMICS_KEY], dynamics.resolved);
    for (path, resolved) in resolved_aiding {
        output = output.with_resolved_component(path, resolved);
    }
    Ok(output)
}

/// Builds the source for aiding entry `name`, returning it with its model's
/// resolved sub-table.
fn build_aiding(
    components: &EstimatorComponents,
    name: &str,
    entry: AidingEntry,
    ctx: &BuildContext<'_>,
) -> Result<(Box<dyn MeasurementSource>, toml::Table), BuildFailure> {
    let wiring = MeasurementWiring {
        input: entry.input,
        noise: DMatrix::from_diagonal(&DVector::from_vec(entry.r_diag)),
    };
    let path = format!("{AIDING_KEY}.{name}.{MODEL_KEY}");
    let built = components.build_measurement(&path, entry.model, ctx, wiring)?;
    Ok((built.component, built.resolved))
}

/// The NIS window aiding entry `name` asks for, if any, or why its
/// `nis_health` is unusable.
fn nis_window(name: &str, entry: &AidingEntry) -> Result<Option<NisWindow>, String> {
    entry
        .nis_health
        .as_ref()
        .map(|health| NisWindow::new(health.window, health.band))
        .transpose()
        .map_err(|err| format!("{AIDING_KEY}.{name}.{NIS_HEALTH_KEY}: {err}"))
}

/// Turns every augmentation entry into a state block tagged with its sensor's
/// frame.
///
/// Fails if an entry names a sensor no aiding entry reads: nothing would
/// observe the block, so it would ride through predict and never be
/// corrected. The aiding source builds its sensor frame from the same channel
/// name, so the block's frame is the one its model reads back.
fn augmentation_blocks(
    entries: &[AugmentationEntry],
    aiding_inputs: &[String],
    ctx: &BuildContext<'_>,
) -> Result<Vec<StateSchemaBlock>, String> {
    entries
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            if !aiding_inputs.contains(&entry.sensor) {
                return Err(format!(
                    "augmentation[{index}] (kind '{}') names sensor '{}', but no aiding entry \
                     reads that channel; the block would never be observed",
                    entry.kind, entry.sensor
                ));
            }
            let sensor = FrameId::sensor(ctx.agent().clone(), entry.sensor.as_str());
            augmentation_block(
                &entry.kind,
                sensor,
                entry.init_uncertainty,
                entry.random_walk,
            )
            .map_err(|err| format!("augmentation[{index}]: {err}"))
        })
        .collect()
}

/// The filter's starting state: the dynamics' state extended by the
/// augmentation blocks, with P₀ and Q from that schema and the mean pose from
/// `pose`.
///
/// Fails if an aiding model's measurement disagrees with the state (a frame
/// the state can't anchor, a convention it doesn't use), naming the sensor,
/// or if the state can't hold the pose.
///
/// Built valid at time zero; the node restamps it with the pipeline clock on
/// its first tick.
fn seeded_state(
    dynamics: &DynamicsComponent,
    augmentation: Vec<StateSchemaBlock>,
    aiding: &[Box<dyn MeasurementSource>],
    pose: &InitialPoseConfig,
    ctx: &BuildContext<'_>,
) -> Result<FrameAwareState, String> {
    let base = dynamics.dynamics().schema();
    let schema = if augmentation.is_empty() {
        base
    } else {
        Arc::new(base.extended(augmentation))
    };

    for source in aiding {
        check_measurement_state_agreement(&schema, &source.model().schema())
            .map_err(|err| format!("aiding channel {}: {err}", source.channel()))?;
    }

    let mut state = FrameAwareState::from_schema(schema, MonotonicTime(0.0));
    dynamics
        .seed_pose(&mut state, ctx.agent(), body_pose(pose))
        .map_err(|err| format!("initial pose: {err}"))?;
    Ok(state)
}

/// The pose of `base_link` in `odom` that `pose` describes: a translation and
/// a rotation about up by the heading.
fn body_pose(pose: &InitialPoseConfig) -> Transform<Flu, Enu> {
    let yaw = pose.heading_deg.to_radians();
    Transform::<Flu, Enu>::from_isometry(Isometry3::from_parts(
        Translation3::new(pose.x, pose.y, pose.z),
        UnitQuaternion::from_quaternion(Quaternion::new(
            (yaw / 2.0).cos(),
            0.0,
            0.0,
            (yaw / 2.0).sin(),
        )),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::NoParams;
    use crate::pipeline::node::{PipelineNode, TickContext};
    use crate::port::{ChannelKey, InputPort, InternalChannel, PortBus, PortDescriptor};

    use helios_core::estimation::augmentation::MAGNETOMETER_BIAS;
    use helios_core::estimation::measurement::{MeasurementModel, Prediction};
    use helios_core::estimation::schema::{MeasurementSchema, MeasurementSchemaBlock};
    use helios_core::interchange::measurement::sensor::GpsPosition;
    use helios_core::prelude::AgentId;
    use helios_core::spatial::state::{Component, Quantity};
    use helios_core::spatial::tf::TfProvider;
    use helios_core::spatial::transforms::{Convention, ErasedTransform};
    use helios_core::spatial::StateVariable;

    use nalgebra::DVector;
    use std::collections::HashSet;

    const NODE: &str = "primary";

    /// A filter and an IMU-driven INS reading `imu/accel` and `imu/gyro`.
    const BASE: &str = r#"
        [filter]
        kind = "Ekf"

        [dynamics]
        kind = "IntegratedImu"
        accel_channel = "imu/accel"
        gyro_channel = "imu/gyro"
        accel_noise_stddev = 0.1
        gyro_noise_stddev = 0.01
        accel_bias_instability = 0.001
        gyro_bias_instability = 0.0001
    "#;

    /// A GPS aiding entry reading `gps`.
    const GPS: &str = r#"
        [aiding.gps]
        input = "gps"
        r_diag = [1.0, 1.0, 1.0]
        model = { kind = "gps_position" }
    "#;

    /// A magnetometer aiding entry reading `mag`.
    const MAG: &str = r#"
        [aiding.mag]
        input = "mag"
        r_diag = [1.0, 1.0, 1.0]
        model = { kind = "magnetometer", magnetic_field_enu = [0.0, 50.0, 0.0] }
    "#;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    /// Builds node [`NODE`] from `section` through `registry`'s node map, as
    /// the assembler does, returning the node and its resolved section.
    fn build_with(
        registry: &AutonomyRegistry,
        section: &str,
    ) -> Result<(Box<dyn PipelineNode>, toml::Table), String> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(agent(), NODE, &channels);
        registry
            .build_node(RECURSIVE_ESTIMATOR_KIND, section, &ctx)
            .map(|built| (built.output.into_node(), built.resolved))
            .map_err(|err| err.to_string())
    }

    fn build_section(section: &str) -> (Box<dyn PipelineNode>, toml::Table) {
        match build_with(&AutonomyRegistry::default(), section) {
            Ok(built) => built,
            Err(err) => panic!("expected the node to build, got: {err}"),
        }
    }

    fn build_err(section: &str) -> String {
        match build_with(&AutonomyRegistry::default(), section) {
            Ok(_) => panic!("expected the build to fail"),
            Err(err) => err,
        }
    }

    /// A TF provider that resolves nothing; the ticks below apply no aiding.
    struct NoTransforms;

    impl TfProvider for NoTransforms {
        fn get_transform(
            &self,
            _: FrameId,
            _: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            None
        }
    }

    /// Runs one cold-start tick (every input empty, so predict is skipped) and
    /// returns the state the node publishes.
    fn published_state(node: &dyn PipelineNode) -> FrameAwareState {
        let descriptor = node.port_descriptor();
        let mut channels: Vec<ChannelKey> = descriptor
            .inputs()
            .map(InputPort::channel)
            .cloned()
            .collect();
        channels.extend(descriptor.outputs().iter().cloned());
        let bus = PortBus::new(&[PortDescriptor::new(vec![], vec![], channels, None)]);

        node.execute(
            &bus,
            &NoTransforms,
            TickContext {
                now: MonotonicTime(0.0),
                dt: 0.1,
                node_id: 0,
            },
        );

        bus.read::<FrameAwareState>(InternalChannel::of::<FrameAwareState>().into())
            .expect("the node publishes its state")
            .value
            .clone()
    }

    /// Declares its measurement as `odom` position in ENU, which the INS state
    /// anchors.
    struct OdomPositionModel;

    impl MeasurementModel for OdomPositionModel {
        fn schema(&self) -> MeasurementSchema {
            MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::odom(agent())),
                Convention::Enu,
            )])
        }

        fn predict_measurement(
            &self,
            _: &FrameAwareState,
            _: Option<&dyn TfProvider>,
            _: MonotonicTime,
        ) -> Prediction {
            Prediction::Ready(DVector::zeros(3))
        }
    }

    /// Declares its measurement as a world-frame position, which the INS state
    /// never anchors.
    struct WorldPositionModel;

    impl MeasurementModel for WorldPositionModel {
        fn schema(&self) -> MeasurementSchema {
            MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::world()),
                Convention::Enu,
            )])
        }

        fn predict_measurement(
            &self,
            _: &FrameAwareState,
            _: Option<&dyn TfProvider>,
            _: MonotonicTime,
        ) -> Prediction {
            Prediction::Ready(DVector::zeros(3))
        }
    }

    /// The default registry with two test measurement kinds reading
    /// `GpsPosition`: `test_odom_position` agrees with the INS state,
    /// `test_world_position` does not.
    fn registry_with_test_models() -> AutonomyRegistry {
        let mut registry = AutonomyRegistry::default();
        let components = registry.extension_mut::<EstimatorComponents>();
        components
            .register_measurement::<GpsPosition, _, _>(
                "test_odom_position",
                |_: NoParams, _: &BuildContext<'_>, _: &FrameId| {
                    Ok(Box::new(OdomPositionModel) as Box<dyn MeasurementModel>)
                },
            )
            .expect("test kind is new");
        components
            .register_measurement::<GpsPosition, _, _>(
                "test_world_position",
                |_: NoParams, _: &BuildContext<'_>, _: &FrameId| {
                    Ok(Box::new(WorldPositionModel) as Box<dyn MeasurementModel>)
                },
            )
            .expect("test kind is new");
        registry
    }

    /// The node is named after its key, needs both IMU channels, reads each
    /// aiding channel when present, and publishes the estimate.
    #[test]
    fn the_node_reads_the_imu_and_its_aiding_and_publishes_the_estimate() {
        let (node, _) = build_section(&format!("{BASE}{GPS}"));
        let descriptor = node.port_descriptor();

        assert_eq!(node.name(), NODE);
        let required: Vec<String> = descriptor
            .required_inputs()
            .map(|key| key.instance().to_string())
            .collect();
        assert_eq!(required, ["imu/accel", "imu/gyro"]);
        let optional: Vec<String> = descriptor
            .optional_inputs()
            .map(|key| key.instance().to_string())
            .collect();
        assert_eq!(optional, ["gps"]);
        assert!(descriptor
            .outputs()
            .contains(&InternalChannel::of::<FrameAwareState>().into()));
    }

    /// The resolved section carries each component's resolved sub-table, so
    /// the dump shows the defaults the components filled in, not the
    /// sub-tables as written.
    #[test]
    fn the_resolved_section_shows_each_components_defaults() {
        let (_, resolved) = build_section(&format!("{BASE}{GPS}"));
        let at = |path: &[&str]| -> toml::Value {
            let mut value = toml::Value::Table(resolved.clone());
            for key in path {
                value = value
                    .get(*key)
                    .unwrap_or_else(|| panic!("no `{}` in the resolved section", path.join(".")))
                    .clone();
            }
            value
        };

        assert_eq!(at(&["filter", "kind"]).as_str(), Some("Ekf"));
        assert_eq!(at(&["dynamics", "kind"]).as_str(), Some("IntegratedImu"));
        assert!(at(&["dynamics", "gravity_enu"]).is_array());
        assert!(at(&["dynamics", "initial_uncertainty", "position_m"]).is_float());
        assert_eq!(at(&["aiding", "gps", "input"]).as_str(), Some("gps"));
        assert_eq!(
            at(&["aiding", "gps", "model", "kind"]).as_str(),
            Some("gps_position")
        );
        assert_eq!(at(&["initial_pose", "heading_deg"]).as_float(), Some(0.0));
    }

    /// A component's error is reported as itself, naming the node and the
    /// sub-table once, not under a second node prefix.
    #[test]
    fn a_failing_component_is_reported_once_at_its_path() {
        let err = build_err(&format!("{}{GPS}", BASE.replace("\"Ekf\"", "\"Ekff\"")));
        assert!(
            err.starts_with("nodes.primary.filter: unknown filter kind 'Ekff'"),
            "{err}"
        );
        assert!(!err.contains(RECURSIVE_ESTIMATOR_KIND), "{err}");
    }

    /// An aiding model's error names its entry's path.
    #[test]
    fn an_aiding_models_error_names_its_entry() {
        let err = build_err(&format!(
            "{BASE}{}",
            GPS.replace("gps_position", "gsp_position")
        ));
        assert!(err.starts_with("nodes.primary.aiding.gps.model:"), "{err}");
    }

    /// An aiding entry may ask for a NIS window; an unusable one is refused,
    /// naming its entry.
    #[test]
    fn an_aiding_entrys_nis_health_is_checked_at_build() {
        let with = |health: &str| format!("{BASE}{GPS}nis_health = {health}\n");
        build_section(&with("{ window = 50, band = [0.3, 3.0] }"));

        let err = build_err(&with("{ window = 0, band = [0.3, 3.0] }"));
        assert!(err.contains("aiding.gps.nis_health: window"), "{err}");
        let err = build_err(&with("{ window = 50, band = [3.0, 0.3] }"));
        assert!(err.contains("aiding.gps.nis_health: band"), "{err}");
    }

    /// An augmentation no aiding entry observes is refused rather than left
    /// to ride through predict uncorrected.
    #[test]
    fn an_unobserved_augmentation_is_refused() {
        let err = build_err(&format!(
            "{BASE}{GPS}
            [[augmentation]]
            kind = \"{MAGNETOMETER_BIAS}\"
            sensor = \"mag\"
            init_uncertainty = 5.0
            random_walk = 0.01"
        ));
        assert!(err.contains("no aiding entry reads that channel"), "{err}");
    }

    /// An augmentation grows the filter's state by its block, under the
    /// frame of the sensor it calibrates.
    #[test]
    fn an_augmentation_extends_the_published_state() {
        let (node, _) = build_section(&format!(
            "{BASE}{MAG}
            [[augmentation]]
            kind = \"{MAGNETOMETER_BIAS}\"
            sensor = \"mag\"
            init_uncertainty = 5.0
            random_walk = 0.01"
        ));
        let state = published_state(node.as_ref());

        assert_eq!(state.schema().storage_dim(), 19, "16 INS + 3 bias");
        let bias = StateVariable::new(
            Quantity::MagBias(FrameId::sensor(agent(), "mag")),
            Component::X,
        );
        assert!(state.schema().storage_offset_of(&bias).is_some());
    }

    /// Without augmentation the state is the dynamics' own.
    #[test]
    fn without_augmentation_the_state_is_the_dynamics_own() {
        let (node, _) = build_section(BASE);
        assert_eq!(published_state(node.as_ref()).schema().storage_dim(), 16);
    }

    /// The starting pose is the mean the filter starts from.
    #[test]
    fn the_initial_pose_seeds_the_state() {
        let (node, _) = build_section(&format!(
            "{BASE}
            [initial_pose]
            x = 5.0
            y = -2.0
            heading_deg = 90.0"
        ));
        let state = published_state(node.as_ref());
        let pose = state
            .pose::<Flu, Enu>(FrameId::base_link(agent()), FrameId::odom(agent()))
            .expect("the INS state holds a pose")
            .into_inner();

        assert_eq!(
            (pose.translation.x, pose.translation.y, pose.translation.z),
            (5.0, -2.0, 0.0)
        );
        let yaw = pose.rotation.euler_angles().2;
        assert!(
            (yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-12,
            "yaw {yaw}"
        );
    }

    /// A measurement kind registered from outside the crate's built-ins is
    /// drawn like any other; one whose measurement the state can't anchor is
    /// refused, naming its channel.
    #[test]
    fn a_registered_measurement_kind_is_checked_against_the_state() {
        let registry = registry_with_test_models();
        let agreeing = GPS.replace("gps_position", "test_odom_position");
        let disagreeing = GPS.replace("gps_position", "test_world_position");

        assert!(build_with(&registry, &format!("{BASE}{agreeing}")).is_ok());
        let Err(err) = build_with(&registry, &format!("{BASE}{disagreeing}")) else {
            panic!("expected the disagreeing model to be refused");
        };
        assert!(err.contains("aiding channel"), "{err}");
        assert!(err.contains("gps"), "{err}");
    }

    /// Registering the kind on an empty registry creates the tables it
    /// declares, empty: the first component asked for is an unknown kind, with
    /// none registered.
    #[test]
    fn with_empty_estimator_components_every_kind_is_unknown() {
        let mut registry = AutonomyRegistry::empty();
        register(&mut registry);

        let Err(err) = build_with(&registry, BASE) else {
            panic!("expected the build to fail");
        };
        assert!(err.ends_with("registered dynamics kinds: (none)"), "{err}");
    }
}
