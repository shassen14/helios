// `[nodes]` integration tests: entries built through registered factories by
// build_pipeline, using only the public API.

use std::collections::{HashMap, HashSet};

use helios_runtime::config::{EkfConfig, EkfDynamicsConfig, IntegratedImuConfig};
use helios_runtime::port::{AlgorithmNodePortDescriptor, PortBus, SensorChannel};
use helios_runtime::{
    build_pipeline, AutonomyPipeline, AutonomyRegistry, AutonomyStack, BodyCapabilities,
    BuildContext, EkfInitialStateConfig, EstimatorConfig, FactoryOutput, PipelineAssemblyError,
    PipelineBuildError, PipelineNode, PortDescriptor, TickContext,
};

use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::prelude::{AgentId, PointCloud, TfProvider};
use helios_core::spatial::conventions::Flu;

use serde::{Deserialize, Serialize};

/// The kind registered from inside these tests; no built-in kind has it.
const TEST_KIND: &str = "TestCloudSource";

/// The payload [`CloudSourceNode`] reads and writes: the type a mapper's scan
/// channel carries, so a legacy mapper can consume its output.
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
/// `[nodes.<name>]` tables. Tests add legacy sections with struct update.
fn stack_with_nodes(nodes_toml: &str) -> AutonomyStack {
    toml::from_str(nodes_toml).expect("test TOML parses")
}

fn host_channels(names: &[&str]) -> HashSet<String> {
    names.iter().map(|name| name.to_string()).collect()
}

fn body() -> BodyCapabilities {
    BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
    }
}

fn build(
    stack: &AutonomyStack,
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

fn build_err(stack: &AutonomyStack, host: &[&str]) -> Vec<PipelineAssemblyError> {
    build(stack, host)
        .err()
        .expect("expected the build to fail")
}

fn node_names(pipeline: &AutonomyPipeline) -> Vec<&str> {
    pipeline.channels().map(|(name, _)| name).collect()
}

/// An IMU-only EKF, the minimum estimator a mapper's state input needs.
fn imu_ekf() -> EstimatorConfig {
    EstimatorConfig::Ekf(EkfConfig {
        dynamics: EkfDynamicsConfig::IntegratedImu(IntegratedImuConfig {
            gravity_enu: [0.0, 0.0, -9.81],
            accel_noise_stddev: 0.1,
            gyro_noise_stddev: 0.01,
            accel_bias_instability: 0.001,
            gyro_bias_instability: 0.0001,
            accel_bias_uncertainty_mps2: 0.1,
            gyro_bias_uncertainty_radps: 0.01,
            accel_channel: "imu/accel".to_string(),
            gyro_channel: "imu/gyro".to_string(),
        }),
        aiding: vec![],
        augmentation: vec![],
        initial_state: EkfInitialStateConfig::default(),
    })
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
    let stack = AutonomyStack {
        estimators: HashMap::from([("nav_ekf".to_string(), imu_ekf())]),
        ..stack_with_nodes(&format!(
            r#"
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

#[test]
fn a_nodes_entry_named_like_a_legacy_node_fails_the_build() {
    // `[nodes]` and the legacy family sections share one namespace. A
    // collision must fail loudly, not keep one node and drop the other.
    let stack = AutonomyStack {
        estimators: HashMap::from([("nav_ekf".to_string(), imu_ekf())]),
        ..stack_with_nodes(&format!(
            r#"
            [nodes.nav_ekf]
            kind = "{TEST_KIND}"
            output = "front.points"
            "#
        ))
    };

    let errors = build_err(&stack, &IMU_CHANNELS);

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::PipelineBuild(build_errors)
                if build_errors.iter().any(|b| matches!(
                    b,
                    PipelineBuildError::DuplicateNodeName { name } if name == "nav_ekf"
                ))
        )),
        "expected DuplicateNodeName for `nav_ekf`, got {errors:?}"
    );
}
