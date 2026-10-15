//! Registers the `RecursiveEstimator` kind, which declares a
//! [`RecursiveEstimatorNode`] from the parts its `[nodes.<name>]` entry names
//! and builds it once the pipeline is wired.
//!
//! The filter, the dynamics and every aiding model come from the
//! [`EstimatorComponents`] tables; the node's own config adds only what ties
//! them together: the aiding wiring, the augmentation blocks and the starting
//! pose.
//!
//! Declaring parses and checks every component and the node's own config, and
//! builds the dynamics, whose input builder names the channels the node
//! reads; the aiding channels come from the measurement kinds. The rest is
//! built with the node, from the [`EstimatorPlan`] declaring leaves.

use super::config::{AidingConfig, AugmentationConfig, RecursiveEstimatorConfig};
use super::nis_health::NisWindow;
use super::node::RecursiveEstimatorNode;
use super::plan::{AidingPlan, AugmentationPlan, EstimatorPlan};

use crate::assembly::{AutonomyRegistry, BuildContext, BuildFailure, FactoryOutput};
use crate::nodes::estimation::EstimatorComponents;

use helios_core::estimation::augmentation::AugmentationSpec;

use nalgebra::{DMatrix, DVector};

/// The `kind` a profile writes to get a recursive estimator.
pub(crate) const RECURSIVE_ESTIMATOR_KIND: &str = "RecursiveEstimator";

/// The `filter` sub-table's key, which is also its path in errors and the dump.
const FILTER_KEY: &str = "filter";
/// The `dynamics` sub-table's key, likewise.
const DYNAMICS_KEY: &str = "dynamics";
/// The key of the aiding table, and the first segment of each model's path.
pub(super) const AIDING_KEY: &str = "aiding";
/// The key of an aiding entry's model sub-table.
const MODEL_KEY: &str = "model";
/// The key of an aiding entry's NIS health check, named in its errors.
const NIS_HEALTH_KEY: &str = "nis_health";
/// The key of an aiding entry's R diagonal, named in its errors.
const R_DIAG_KEY: &str = "r_diag";
/// The key of the augmentation list, named in its errors.
const AUGMENTATION_KEY: &str = "augmentation";

/// Adds the `RecursiveEstimator` kind to `registry`.
pub(crate) fn register(registry: &mut AutonomyRegistry) {
    registry
        .register_node_with::<EstimatorComponents, _, _, _>(RECURSIVE_ESTIMATOR_KIND, build)
        .expect("RecursiveEstimator is registered once, by the default registry")
}

/// Declares the node for one entry: builds the dynamics, parses and checks
/// each aiding entry and the filter, checks every augmentation, and declares
/// the node's ports from the dynamics' input and the aiding channels. The
/// node is built from an [`EstimatorPlan`] once the pipeline is wired.
fn build(
    config: RecursiveEstimatorConfig,
    ctx: &BuildContext<'_>,
    components: &EstimatorComponents,
) -> Result<FactoryOutput, BuildFailure> {
    let dynamics = components.build_dynamics(DYNAMICS_KEY, config.dynamics, ctx)?;
    let mut resolved_aiding = Vec::with_capacity(config.aiding.len());
    let mut aiding = Vec::with_capacity(config.aiding.len());
    for (name, entry) in config.aiding {
        let (plan, resolved) = declare_aiding(components, name, entry, ctx)?;
        resolved_aiding.push((
            [
                AIDING_KEY.to_string(),
                plan.name.clone(),
                MODEL_KEY.to_string(),
            ],
            resolved,
        ));
        aiding.push(plan);
    }
    let augmentation = declare_augmentation(&config.augmentation, &aiding)?;
    let filter = components.parse_filter(FILTER_KEY, config.filter, ctx)?;

    let descriptor = RecursiveEstimatorNode::declare_ports(
        ctx.node_name(),
        dynamics.component.input(),
        aiding
            .iter()
            .map(|entry| (entry.name.as_str(), &entry.channel)),
    );
    let plan = EstimatorPlan {
        agent: ctx.agent().clone(),
        node_name: ctx.node_name().to_string(),
        kind: ctx.kind().to_string(),
        dynamics: dynamics.component,
        filter: filter.deferred,
        aiding,
        augmentation,
        initial_pose: config.initial_pose,
    };

    let mut output = FactoryOutput::plan(ctx, descriptor, move |descriptor, mounts| {
        plan.construct(descriptor, mounts)
    })
    .with_resolved_component([FILTER_KEY], filter.resolved)
    .with_resolved_component([DYNAMICS_KEY], dynamics.resolved);
    for (path, resolved) in resolved_aiding {
        output = output.with_resolved_component(path, resolved);
    }
    Ok(output)
}

/// Declares aiding entry `name`: its NIS window, its R, the channel its
/// model's kind reads, and the source still to build. Returns it with its
/// model's resolved sub-table.
fn declare_aiding(
    components: &EstimatorComponents,
    name: String,
    entry: AidingConfig,
    ctx: &BuildContext<'_>,
) -> Result<(AidingPlan, toml::Table), BuildFailure> {
    let nis_window = nis_window(&name, &entry)?;
    check_r_diag(&name, &entry.r_diag)?;
    let path = format!("{AIDING_KEY}.{name}.{MODEL_KEY}");
    let parsed = components.parse_measurement(&path, entry.model, ctx)?;
    let channel = (parsed.declared)(&entry.input);
    let plan = AidingPlan {
        name,
        input: entry.input,
        channel,
        nis_window,
        noise: DMatrix::from_diagonal(&DVector::from_vec(entry.r_diag)),
        model: parsed.deferred,
    };
    Ok((plan, parsed.resolved))
}

/// The NIS window aiding entry `name` asks for, if any, or why its
/// `nis_health` is unusable.
fn nis_window(name: &str, entry: &AidingConfig) -> Result<Option<NisWindow>, String> {
    entry
        .nis_health
        .as_ref()
        .map(|health| NisWindow::new(health.window, health.band))
        .transpose()
        .map_err(|err| format!("{AIDING_KEY}.{name}.{NIS_HEALTH_KEY}: {err}"))
}

/// Fails if an entry of aiding entry `name`'s R diagonal is not a finite,
/// positive variance: the filter would skip or mis-weigh every update from
/// that sensor. R's length against the model's measurement is checked when
/// the source is built.
fn check_r_diag(name: &str, r_diag: &[f64]) -> Result<(), String> {
    for (index, variance) in r_diag.iter().enumerate() {
        if !variance.is_finite() || *variance <= 0.0 {
            return Err(format!(
                "{AIDING_KEY}.{name}.{R_DIAG_KEY}[{index}] must be a finite, positive \
                 variance, got {variance}"
            ));
        }
    }
    Ok(())
}

/// Checks every augmentation entry: its kind and noise, and that an aiding
/// entry reads its sensor, whose channel the block takes its mount from. An
/// unobserved block would ride through predict and never be corrected.
fn declare_augmentation(
    entries: &[AugmentationConfig],
    aiding: &[AidingPlan],
) -> Result<Vec<AugmentationPlan>, String> {
    entries
        .iter()
        .enumerate()
        .map(|(index, entry)| {
            let Some(reader) = aiding.iter().find(|plan| plan.input == entry.sensor) else {
                return Err(format!(
                    "{AUGMENTATION_KEY}[{index}] (kind '{}') names sensor '{}', but no aiding \
                     entry reads that channel; the block would never be observed",
                    entry.kind, entry.sensor
                ));
            };
            let spec =
                AugmentationSpec::new(&entry.kind, entry.init_uncertainty, entry.random_walk)
                    .map_err(|err| format!("{AUGMENTATION_KEY}[{index}]: {err}"))?;
            Ok(AugmentationPlan {
                channel: reader.channel.clone(),
                spec,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::{no_check, NoParams};
    use crate::channels::estimate::estimator_output;
    use crate::pipeline::node::{PipelineNode, TickContext};
    use crate::pipeline::NodeDeclaration;
    use crate::port::{
        ChannelDescription, ChannelKey, InputPort, PortBus, PortDescriptor, Provenance,
    };
    use crate::BodyChannel;

    use helios_core::estimation::augmentation::MAGNETOMETER_BIAS;
    use helios_core::estimation::measurement::{MeasurementModel, Prediction};
    use helios_core::estimation::schema::{MeasurementSchema, MeasurementSchemaBlock};
    use helios_core::interchange::measurement::sensor::GpsPosition;
    use helios_core::prelude::AgentId;
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::primitives::MonotonicTime;
    use helios_core::spatial::state::{Component, Quantity};
    use helios_core::spatial::tf::TfProvider;
    use helios_core::spatial::transforms::{Convention, ErasedTransform};
    use helios_core::spatial::{FrameAwareState, FrameId, StateVariable};

    use nalgebra::DVector;

    use std::collections::HashMap;

    /// An arbitrary global frame the test expresses quantities in.
    const DATUM: &str = "test_datum";

    const NODE: &str = "primary";
    /// The unit every input here comes from.
    const UNIT: &str = "test_unit";
    /// Appended to a channel's name to name its mount, so every input is
    /// mounted apart from its name: a block or model tagged by the name
    /// instead of the mount would be caught.
    const MOUNT_SUFFIX: &str = "_mount";

    /// A filter and an IMU-driven INS reading `imu.accel` and `imu.gyro`.
    const BASE: &str = r#"
        [filter]
        kind = "Ekf"

        [dynamics]
        kind = "IntegratedImu"
        accel_channel = "imu.accel"
        gyro_channel = "imu.gyro"
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

    /// The frame the channel named `channel` is mounted at.
    fn mount(channel: &str) -> FrameId {
        FrameId::agent(agent(), format!("{channel}{MOUNT_SUFFIX}"))
    }

    /// Input `key` as the body publishes it, at its [`mount`].
    fn mounted_input(key: ChannelKey) -> ChannelDescription {
        let mount = mount(key.instance());
        ChannelDescription::body(
            &BodyChannel::new(key, UNIT, Provenance::Simulated).with_mount(mount),
        )
    }

    /// Input `key` as the body publishes it, with no mount.
    fn unmounted_input(key: ChannelKey) -> ChannelDescription {
        ChannelDescription::body(&BodyChannel::new(key, UNIT, Provenance::Simulated))
    }

    /// Declares node [`NODE`] from `section` through `registry`'s node map,
    /// as the assembler does, then builds it as the pipeline builder does,
    /// with every input described by `describe`, returning the node and its
    /// resolved section.
    fn build_with_inputs(
        registry: &AutonomyRegistry,
        section: &str,
        describe: fn(ChannelKey) -> ChannelDescription,
    ) -> Result<(Box<dyn PipelineNode>, toml::Table), String> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let ctx = BuildContext::new(agent(), NODE, RECURSIVE_ESTIMATOR_KIND);
        let built = registry
            .build_node(section, &ctx)
            .map_err(|err| err.to_string())?;
        let inputs: Vec<ChannelDescription> = declared_inputs(&built.declaration)
            .into_iter()
            .map(describe)
            .collect();
        let by_key: HashMap<&ChannelKey, &ChannelDescription> = inputs
            .iter()
            .map(|description| (description.key(), description))
            .collect();
        let node = built
            .declaration
            .construct(&by_key)
            .map_err(|err| err.to_string())?;
        Ok((node, built.resolved))
    }

    /// Every channel `declaration` reads.
    fn declared_inputs(declaration: &NodeDeclaration) -> Vec<ChannelKey> {
        declaration
            .port_descriptor()
            .inputs()
            .map(InputPort::channel)
            .cloned()
            .collect()
    }

    /// [`build_with_inputs`] with every input mounted.
    fn build_with(
        registry: &AutonomyRegistry,
        section: &str,
    ) -> Result<(Box<dyn PipelineNode>, toml::Table), String> {
        build_with_inputs(registry, section, mounted_input)
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

    /// Declares node [`NODE`] from `section`, as the assembler does, without
    /// building it, and returns the declaration's error.
    fn declare_err(section: &str) -> String {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let ctx = BuildContext::new(agent(), NODE, RECURSIVE_ESTIMATOR_KIND);
        match AutonomyRegistry::default().build_node(section, &ctx) {
            Ok(_) => panic!("expected the declaration to fail"),
            Err(err) => err.to_string(),
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
        channels.extend(descriptor.outputs().cloned());
        let bus = PortBus::new(&[PortDescriptor::new(vec![], vec![], channels, None)]);

        node.execute(
            &bus,
            &NoTransforms,
            TickContext::detached(MonotonicTime(0.0), 0.1, 0),
        );

        bus.read::<FrameAwareState>(estimator_output(NODE).into())
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
                Quantity::Position(FrameId::global(DATUM)),
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
            .register_measurement::<GpsPosition, _, _, _>(
                "test_odom_position",
                no_check,
                |_: NoParams, _: &BuildContext<'_>, _: &FrameId| {
                    Ok(Box::new(OdomPositionModel) as Box<dyn MeasurementModel>)
                },
            )
            .expect("test kind is new");
        components
            .register_measurement::<GpsPosition, _, _, _>(
                "test_world_position",
                no_check,
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
        assert_eq!(required, ["imu.accel", "imu.gyro"]);
        let optional: Vec<String> = descriptor
            .optional_inputs()
            .map(|key| key.instance().to_string())
            .collect();
        assert_eq!(optional, ["gps"]);
        assert!(descriptor
            .outputs()
            .any(|key| *key == estimator_output(NODE).into()));
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

    /// An augmentation no aiding entry observes is refused at declaration
    /// rather than left to ride through predict uncorrected.
    #[test]
    fn an_unobserved_augmentation_is_refused() {
        let err = declare_err(&format!(
            "{BASE}{GPS}
            [[augmentation]]
            kind = \"{MAGNETOMETER_BIAS}\"
            sensor = \"mag\"
            init_uncertainty = 5.0
            random_walk = 0.01"
        ));
        assert!(err.contains("no aiding entry reads that channel"), "{err}");
    }

    /// An augmentation's kind is checked at declaration, before the sensor
    /// frame it is tied to exists.
    #[test]
    fn an_unknown_augmentation_kind_is_refused_at_declaration() {
        let err = declare_err(&format!(
            "{BASE}{MAG}
            [[augmentation]]
            kind = \"gyro_scale_factor\"
            sensor = \"mag\"
            init_uncertainty = 5.0
            random_walk = 0.01"
        ));
        assert!(err.contains("augmentation[0]"), "{err}");
        assert!(err.contains("gyro_scale_factor"), "{err}");
    }

    /// A filter value unusable on its own is refused at declaration.
    #[test]
    fn a_bad_filter_value_is_refused_at_declaration() {
        let ekf = BASE.replace("kind = \"Ekf\"", "kind = \"Ekf\"\ncovariance_floor = -1.0");
        let err = declare_err(&format!("{ekf}{GPS}"));
        assert!(
            err.starts_with("nodes.primary.filter (kind 'Ekf')"),
            "{err}"
        );
        assert!(err.contains("covariance_floor"), "{err}");

        let ukf = BASE.replace(
            "kind = \"Ekf\"",
            "kind = \"Ukf\"\nalpha = nan\nbeta = 2.0\nkappa = 0.0",
        );
        let err = declare_err(&format!("{ukf}{GPS}"));
        assert!(
            err.starts_with("nodes.primary.filter (kind 'Ukf')"),
            "{err}"
        );
        assert!(err.contains("alpha"), "{err}");
    }

    /// An R variance that is not finite and positive is refused at
    /// declaration, naming its entry.
    #[test]
    fn a_bad_r_variance_is_refused_at_declaration() {
        for variance in ["0.0", "-1.0", "nan", "inf"] {
            let gps = GPS.replace("[1.0, 1.0, 1.0]", &format!("[1.0, {variance}, 1.0]"));
            let err = declare_err(&format!("{BASE}{gps}"));
            assert!(err.contains("aiding.gps.r_diag[1]"), "{variance}: {err}");
        }
    }

    /// An augmentation grows the filter's state by its block, under the
    /// mount of the sensor channel it calibrates.
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
        let bias = StateVariable::new(Quantity::MagBias(mount("mag")), Component::X);
        assert!(state.schema().storage_offset_of(&bias).is_some());
    }

    /// An aiding source is built for its channel's mount, so a channel whose
    /// writer states none is refused when the node is built, naming the node
    /// and the channel.
    #[test]
    fn an_aiding_channel_without_a_mount_is_refused() {
        let Err(err) = build_with_inputs(
            &AutonomyRegistry::default(),
            &format!("{BASE}{GPS}"),
            unmounted_input,
        ) else {
            panic!("expected the unmounted GPS to be refused");
        };
        assert!(
            err.contains(&format!("node \"{NODE}\" needs the mount of channel")),
            "{err}"
        );
        assert!(err.contains("gps"), "{err}");
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
