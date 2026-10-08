// `[nodes]` integration tests: entries built through registered factories by
// build_pipeline, using only the public API.

use std::collections::HashSet;

use helios_runtime::channels::{oracle_pose_channel, oracle_twist_channel};
use helios_runtime::port::{AlgorithmNodePortDescriptor, PortBus, SensorChannel};
use helios_runtime::{
    build_pipeline, AutonomyPipeline, AutonomyRegistry, AutonomyStackConfig, BodyCapabilities,
    BuildContext, EstimateSeamConfig, FactoryOutput, PipelineAssemblyError, PipelineBuildError,
    PipelineNode, PortDescriptor, Provenance, PublishedChannel, TickContext,
};

use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::prelude::{AgentId, PointCloud, TfProvider};
use helios_core::spatial::conventions::Flu;

use serde::{Deserialize, Serialize};

/// The kind registered from inside these tests; no built-in kind has it.
const TEST_KIND: &str = "TestCloudSource";

/// The payload [`CloudSourceNode`] reads and writes: the type a mapper's scan
/// channel carries, so a mapper can consume its output.
type CloudBatch = Vec<SensorReading<PointCloud<Flu>>>;

/// A node that writes one point-cloud sensor channel and optionally reads
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
    let mut ports = AlgorithmNodePortDescriptor::new()
        .output_sensor(SensorChannel::named::<CloudBatch>(config.output.as_str()));
    if let Some(input) = &config.input {
        ports = ports.input_sensor(SensorChannel::named::<CloudBatch>(input.as_str()));
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

fn host_channels(names: &[&str]) -> HashSet<String> {
    names.iter().map(|name| name.to_string()).collect()
}

fn body() -> BodyCapabilities {
    BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        ..Default::default()
    }
}

fn build(
    stack: &AutonomyStackConfig,
    host: &[&str],
) -> Result<AutonomyPipeline, Vec<PipelineAssemblyError>> {
    build_pipeline(
        stack,
        &registry_with_test_kind(),
        AgentId::new("test_agent"),
        &host_channels(host),
        body(),
    )
}

fn build_err(stack: &AutonomyStackConfig, host: &[&str]) -> Vec<PipelineAssemblyError> {
    build(stack, host)
        .err()
        .expect("expected the build to fail")
}

fn node_names(pipeline: &AutonomyPipeline) -> Vec<&str> {
    pipeline.channels().map(|(name, _)| name).collect()
}

const IMU_CHANNELS: [&str; 2] = ["imu/accel", "imu/gyro"];

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

    let pipeline = build(&stack, &[]).expect("a registered kind under [nodes] must build");

    assert!(
        node_names(&pipeline).contains(&"front_source"),
        "expected `front_source` in {:?}",
        node_names(&pipeline)
    );
}

#[test]
fn a_nodes_entry_reading_a_host_channel_is_seeded_from_it() {
    // The node's sensor input is a host channel, so the build counts it as
    // supplied by the body rather than demanding a producer in the graph.
    let stack = stack_with_nodes(&format!(
        r#"
        [nodes.relay]
        kind = "{TEST_KIND}"
        input = "lidar"
        output = "lidar.relayed"
        "#
    ));

    let pipeline = build(&stack, &["lidar"]).expect("an input the host publishes must build");

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

    let errors = build_err(&stack, &["lidar"]);

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::UnpublishedSensorInput { node_name, channel }
                if node_name == "relay" && channel == "lidar.typo"
        )),
        "expected UnpublishedSensorInput for `relay` / `lidar.typo`, got {errors:?}"
    );
}

#[test]
fn a_nodes_entry_reads_a_channel_another_derives() {
    // The host publishes only the IMU. The grid's scan channel is written by
    // another `[nodes]` entry, so it is a derived channel: no host publisher
    // needed, and the producer is ordered ahead of the grid. The grid sorts
    // first by name, so it is built before its producer; the derived set must
    // already hold the producer's output when the grid is seeded.
    let stack = AutonomyStackConfig {
        estimate: Some(EstimateSeamConfig {
            source: "nav_ekf".to_string(),
        }),
        ..stack_with_nodes(&format!(
            r#"
            [nodes.nav_ekf]
            kind = "RecursiveEstimator"
            filter = {{ kind = "Ekf" }}
            dynamics = {{ kind = "IntegratedImu", accel_channel = "imu/accel", gyro_channel = "imu/gyro", accel_noise_stddev = 0.1, gyro_noise_stddev = 0.01, accel_bias_instability = 0.001, gyro_bias_instability = 0.0001 }}

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

    let pipeline =
        build(&stack, &IMU_CHANNELS).expect("a grid reading another node's output must build");

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
fn a_nodes_output_reusing_a_host_channel_name_fails_the_build() {
    let stack = stack_with_nodes(&format!(
        r#"
        [nodes.front_source]
        kind = "{TEST_KIND}"
        output = "lidar"
        "#
    ));

    let errors = build_err(&stack, &["lidar"]);

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::SensorOutputShadowsHost { node_name, channel }
                if node_name == "front_source" && channel == "lidar"
        )),
        "expected SensorOutputShadowsHost for `front_source`, got {errors:?}"
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

    let errors = build_err(&stack, &[]);

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

    let errors = build_err(&stack, &[]);

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
        &HashSet::new(),
        body,
    )
}

#[test]
fn a_mock_oracle_entry_builds_on_a_body_with_truth() {
    let truth = BodyCapabilities {
        publishes: [oracle_pose_channel().into(), oracle_twist_channel().into()]
            .into_iter()
            .map(|key| PublishedChannel {
                key,
                provenance: Provenance::Exact,
            })
            .collect(),
        ..body()
    };

    let pipeline = build_on(truth).expect("a body publishing oracle truth must build");

    assert!(node_names(&pipeline).contains(&"primary"));
}

#[test]
fn a_mock_oracle_entry_is_refused_on_a_body_without_truth() {
    // The fence is the pipeline builder's, so it holds for a `[nodes]` entry
    // like any other node: oracle truth only reaches a node through the body.
    let errors = build_on(body())
        .err()
        .expect("a body without oracle/pose must refuse the mock");

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
