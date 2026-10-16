// `[nodes]` integration tests: entries built through registered factories by
// build_pipeline, using only the public API.

use helios_runtime::channels::{oracle_pose_channel, oracle_twist_channel};
use helios_runtime::port::{ChannelKey, PortBus, PortDescriptorBuilder};
use helios_runtime::{
    build_pipeline, AutonomyPipeline, AutonomyRegistry, AutonomyStackConfig, BodyCapabilities,
    BodyChannel, BuildContext, EstimateSeamConfig, FactoryOutput, PipelineAssemblyError,
    PipelineBuildError, PipelineNode, PortDescriptor, Provenance, TickContext, Writer,
};

use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::prelude::{Acceleration, AgentId, AngularRate, PointCloud, TfProvider};
use helios_core::spatial::conventions::Flu;
use helios_core::spatial::FrameId;

use serde::{Deserialize, Serialize};

/// The kind registered from inside these tests; no built-in kind has it.
const TEST_KIND: &str = "TestCloudSource";

/// The payload [`CloudSourceNode`] reads and writes: the type a mapper's scan
/// channel carries, so a mapper can consume its output.
type CloudBatch = Vec<SensorReading<PointCloud<Flu>>>;

/// A node that writes one point-cloud measurement channel and optionally reads
/// another. It never runs; these tests only build pipelines.
struct CloudSourceNode {
    name: String,
    descriptor: PortDescriptor,
}

impl PipelineNode for CloudSourceNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, _tick: TickContext) {}
}

/// The `[nodes.<name>]` section of a [`TEST_KIND`] entry.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct CloudSourceConfig {
    output: String,
    #[serde(default)]
    input: Option<String>,
}

fn build_cloud_source(
    config: CloudSourceConfig,
    ctx: &BuildContext<'_>,
) -> Result<FactoryOutput, String> {
    let mut ports = PortDescriptorBuilder::algorithm().output(
        ChannelKey::measurement::<CloudBatch>(config.output.as_str()),
    );
    if let Some(input) = &config.input {
        ports = ports.input(ChannelKey::measurement::<CloudBatch>(input.as_str()));
    }
    Ok(FactoryOutput::new(Box::new(CloudSourceNode {
        name: ctx.node_name().to_string(),
        descriptor: ports.build(),
    })))
}

/// The default registry plus [`TEST_KIND`], registered the way any crate
/// outside `helios_runtime` would add a kind.
fn registry_with_test_kind() -> AutonomyRegistry {
    let mut registry = AutonomyRegistry::default();
    registry
        .register_node(TEST_KIND, build_cloud_source)
        .expect("the test kind is not built in");
    registry
}

/// A stack whose only content is `nodes_toml`, a TOML document of
/// `[nodes.<name>]` tables. Tests add other sections with struct update.
fn stack_with_nodes(nodes_toml: &str) -> AutonomyStackConfig {
    toml::from_str(nodes_toml).expect("test TOML parses")
}

/// The unit every point-cloud channel in this file comes from.
const LIDAR_UNIT: &str = "lidar";
/// The unit the IMU channels come from.
const IMU_UNIT: &str = "imu";
/// The frame the IMU is mounted at.
const IMU_MOUNT: &str = "imu_link";
/// The unit the oracle channels come from.
const TRUTH_UNIT: &str = "truth";

/// A channel `unit` publishes, carrying batches of `P` readings.
fn measurement_channel<P: Send + Sync + 'static>(name: &str, unit: &str) -> BodyChannel {
    BodyChannel::new(
        ChannelKey::measurement::<Vec<SensorReading<P>>>(name),
        unit,
        Provenance::Simulated,
    )
}

/// A body publishing one point-cloud channel per name in `clouds`.
fn body_with_clouds(clouds: &[&str]) -> BodyCapabilities {
    BodyCapabilities {
        publishes: clouds
            .iter()
            .map(|name| measurement_channel::<PointCloud<Flu>>(name, LIDAR_UNIT))
            .collect(),
        ..Default::default()
    }
}

fn build(
    stack: &AutonomyStackConfig,
    body: BodyCapabilities,
) -> Result<AutonomyPipeline, Vec<PipelineAssemblyError>> {
    build_pipeline(
        stack,
        &registry_with_test_kind(),
        AgentId::new("test_agent"),
        body,
    )
}

fn build_err(stack: &AutonomyStackConfig, body: BodyCapabilities) -> Vec<PipelineAssemblyError> {
    build(stack, body)
        .err()
        .expect("expected the build to fail")
}

/// The build errors inside `errors`, which a `[nodes]` stack reports after
/// every node is built.
fn build_errors(errors: &[PipelineAssemblyError]) -> Vec<&PipelineBuildError> {
    errors
        .iter()
        .flat_map(|e| match e {
            PipelineAssemblyError::PipelineBuild(build_errors) => build_errors.iter().collect(),
            _ => vec![],
        })
        .collect()
}

fn node_names(pipeline: &AutonomyPipeline) -> Vec<&str> {
    pipeline.channels().map(|(name, _)| name).collect()
}

#[test]
fn a_kind_registered_from_outside_the_crate_is_built() {
    // The goal of node factories: registering a kind and naming it under
    // `[nodes]` is all it takes. No family section, no assembler change.
    let stack = stack_with_nodes(&format!(
        r#"
        [nodes.front_source]
        kind = "{TEST_KIND}"
        output = "front.points"
        "#
    ));

    let pipeline = build(&stack, BodyCapabilities::default())
        .expect("a registered kind under [nodes] must build");

    assert!(
        node_names(&pipeline).contains(&"front_source"),
        "expected `front_source` in {:?}",
        node_names(&pipeline)
    );
}

#[test]
fn a_nodes_entry_reading_a_body_channel_builds() {
    // The node's sensor input is a body channel, so the build counts it as
    // supplied by the body rather than demanding a producer in the graph.
    let stack = stack_with_nodes(&format!(
        r#"
        [nodes.relay]
        kind = "{TEST_KIND}"
        input = "lidar"
        output = "lidar.relayed"
        "#
    ));

    let pipeline = build(&stack, body_with_clouds(&["lidar"]))
        .expect("an input the body publishes must build");

    assert!(
        node_names(&pipeline).contains(&"relay"),
        "expected `relay` in {:?}",
        node_names(&pipeline)
    );
}

#[test]
fn a_nodes_entry_reading_an_unpublished_channel_fails_the_build() {
    let stack = stack_with_nodes(&format!(
        r#"
        [nodes.relay]
        kind = "{TEST_KIND}"
        input = "lidar.typo"
        output = "lidar.relayed"
        "#
    ));

    let errors = build_err(&stack, body_with_clouds(&["lidar"]));

    assert!(
        build_errors(&errors).iter().any(|e| matches!(
            e,
            PipelineBuildError::UnsatisfiedInput { node_name, channel, .. }
                if node_name == "relay" && channel.instance().as_ref() == "lidar.typo"
        )),
        "expected UnsatisfiedInput for `relay` / `lidar.typo`, got {errors:?}"
    );
    assert!(
        errors.iter().any(|e| e
            .to_string()
            .contains("the body does not publish it and no node writes it")),
        "a sensor input's error says where it could come from, got {errors:?}"
    );
}

#[test]
fn a_nodes_entry_reads_a_channel_another_derives() {
    // The body publishes only the IMU. The grid's scan channel is written by
    // another `[nodes]` entry, so it needs no body channel, and the producer
    // is ordered ahead of the grid. The grid sorts first by name, so it is
    // built before its producer; the build must not depend on that order.
    let stack = AutonomyStackConfig {
        estimate: Some(EstimateSeamConfig {
            source: "nav_ekf".to_string(),
        }),
        ..stack_with_nodes(&format!(
            r#"
            [nodes.nav_ekf]
            kind = "RecursiveEstimator"
            filter = {{ kind = "Ekf" }}
            dynamics = {{ kind = "IntegratedImu", accel_channel = "imu.accel", gyro_channel = "imu.gyro", accel_noise_stddev = 0.1, gyro_noise_stddev = 0.01, accel_bias_instability = 0.001, gyro_bias_instability = 0.0001 }}

            [nodes.grid]
            kind = "OccupancyGrid2D"
            rate = 5.0
            resolution = 0.1
            scan_channel = "lidar.points"
            width_m = 10.0
            height_m = 10.0

            [nodes.scan_source]
            kind = "{TEST_KIND}"
            output = "lidar.points"
            "#
        ))
    };

    let mount = FrameId::agent(AgentId::new("test_agent"), IMU_MOUNT);
    let imu = BodyCapabilities {
        publishes: vec![
            measurement_channel::<Acceleration>("imu.accel", IMU_UNIT).with_mount(mount.clone()),
            measurement_channel::<AngularRate>("imu.gyro", IMU_UNIT).with_mount(mount),
        ],
        ..Default::default()
    };
    let pipeline = build(&stack, imu).expect("a grid reading another node's output must build");

    let order = node_names(&pipeline);
    let position = |node: &str| {
        order
            .iter()
            .position(|name| *name == node)
            .unwrap_or_else(|| panic!("node `{node}` missing from {order:?}"))
    };
    assert!(
        position("scan_source") < position("grid"),
        "the source must run before the grid that reads it, got {order:?}"
    );
}

#[test]
fn a_nodes_output_reusing_a_body_channel_name_fails_the_build() {
    // The output and the body's channel are one channel with two writers,
    // and the error names both.
    let stack = stack_with_nodes(&format!(
        r#"
        [nodes.front_source]
        kind = "{TEST_KIND}"
        output = "lidar"
        "#
    ));

    let errors = build_err(&stack, body_with_clouds(&["lidar"]));

    assert!(
        build_errors(&errors).iter().any(|e| matches!(
            e,
            PipelineBuildError::MultipleSuppliers { channel, first, second }
                if channel.instance().as_ref() == "lidar"
                    && matches!(first, Writer::Body { unit, .. } if unit == LIDAR_UNIT)
                    && *second == Writer::Node { node: "front_source".to_string() }
        )),
        "expected MultipleSuppliers naming the body and `front_source`, got {errors:?}"
    );
}

#[test]
fn an_unknown_kind_fails_the_build_listing_the_registered_ones() {
    let stack = stack_with_nodes(
        r#"
        [nodes.front_source]
        kind = "TestCloudSorce"
        output = "front.points"
        "#,
    );

    let errors = build_err(&stack, BodyCapabilities::default());

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::UnknownNodeKind { node_name, kind, registered }
                if node_name == "front_source"
                    && kind == "TestCloudSorce"
                    && registered.iter().any(|k| k == TEST_KIND)
        )),
        "expected UnknownNodeKind for `front_source`, got {errors:?}"
    );
}

#[test]
fn a_nodes_entry_without_a_kind_fails_the_build() {
    let stack = stack_with_nodes(
        r#"
        [nodes.front_source]
        output = "front.points"
        "#,
    );

    let errors = build_err(&stack, BodyCapabilities::default());

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::MissingNodeKind { node_name } if node_name == "front_source"
        )),
        "expected MissingNodeKind for `front_source`, got {errors:?}"
    );
}

/// A stack whose one node is a `MockOracle` named `primary`, the estimator it
/// stands in for.
fn mock_oracle_stack() -> AutonomyStackConfig {
    stack_with_nodes(
        r#"
        [nodes.primary]
        kind = "MockOracle"
        "#,
    )
}

fn build_on(body: BodyCapabilities) -> Result<AutonomyPipeline, Vec<PipelineAssemblyError>> {
    build_pipeline(
        &mock_oracle_stack(),
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        body,
    )
}

#[test]
fn a_mock_oracle_entry_builds_on_a_body_with_truth() {
    let truth = BodyCapabilities {
        publishes: [oracle_pose_channel(), oracle_twist_channel()]
            .into_iter()
            .map(|key: ChannelKey| BodyChannel::new(key, TRUTH_UNIT, Provenance::Exact))
            .collect(),
        ..Default::default()
    };

    let pipeline = build_on(truth).expect("a body publishing oracle truth must build");

    assert!(node_names(&pipeline).contains(&"primary"));
}

#[test]
fn a_mock_oracle_entry_is_refused_on_a_body_without_truth() {
    // The fence is the pipeline builder's, so it holds for a `[nodes]` entry
    // like any other node: oracle truth only reaches a node through the body.
    let errors = build_on(BodyCapabilities::default())
        .err()
        .expect("a body without oracle.pose must refuse the mock");

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::PipelineBuild(build_errors)
                if build_errors.iter().any(|b| matches!(
                    b,
                    PipelineBuildError::UnsatisfiedBodyCapabilities { node_name, .. }
                        if node_name == "primary"
                ))
        )),
        "expected UnsatisfiedBodyCapabilities for `primary`, got {errors:?}"
    );
}
