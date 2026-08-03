// Assembler integration tests: build_pipeline topology resolution.

use std::collections::HashMap;

use helios_runtime::channels::control;
use helios_runtime::config::{
    AutonomyStack, CommandArbitrationConfig, CommandSource, EkfConfig, EkfDynamicsConfig,
    EkfInitialStateConfig, EstimatorConfig, IntegratedImuConfig, MapLayerConfig, SearchPlannerConfig,
};
use helios_runtime::port::ChannelKey;
use helios_runtime::prelude::{Health, Stamped};
use helios_runtime::{
    build_pipeline, AutonomyRegistry, BodyCapabilities, ConfigValidationError, PipelineAssemblyError,
};

use helios_core::control::commands::BodyTwist;
use helios_core::data::primitives::{FrameHandle, MonotonicTime};

use crate::common::MockRuntime;

/// A body with no autonomy stack, declaring only that it consumes control.
fn teleop_body() -> BodyCapabilities {
    BodyCapabilities {
        name: "teleop_only".to_string(),
        publishes: vec![],
        consumes_control: true,
    }
}

// =========================================================================
// == Teleop-only: a single teleop source drives `command` with no autonomy ==
// =========================================================================

#[test]
fn teleop_only_stack_builds_without_a_controller() {
    // `sources = ["teleop"]` and no controllers: the assembler must resolve a
    // direct topology (no arbiter) in which the host publishes `command`
    // itself. Nothing in the brain produces commands.
    let stack = AutonomyStack {
        command_arbitration: CommandArbitrationConfig {
            sources: vec![CommandSource::Teleop],
            ..Default::default()
        },
        ..Default::default()
    };

    let result = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        teleop_body(),
    );

    assert!(
        result.is_ok(),
        "teleop-only stack must build, got: {:?}",
        result.err()
    );
}

#[test]
fn teleop_only_command_is_host_published_not_brain_produced() {
    // The headline capability: with no autonomy controller, `read_control`
    // stays empty until the host writes `command`, then reflects that write.
    // This proves `command` is a real host-published slot and that nothing in
    // the brain fabricates a command.
    let stack = AutonomyStack {
        command_arbitration: CommandArbitrationConfig {
            sources: vec![CommandSource::Teleop],
            ..Default::default()
        },
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        teleop_body(),
    )
    .expect("teleop-only stack must build");

    assert!(
        pipeline.read_control().is_none(),
        "no source has published yet, so there must be no command"
    );

    let command_key: ChannelKey = control::command::<BodyTwist>().into();
    pipeline
        .bus()
        .write(
            command_key,
            Stamped {
                value: BodyTwist::zero(),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("host write to `command` must succeed");

    pipeline.tick(&MockRuntime, 0.1);

    assert!(
        pipeline.read_control().is_some(),
        "read_control must reflect the host-published command"
    );
}

// =========================================================================
// == Node identity: every node is named by its config-map key, not its kind ==
// =========================================================================

#[test]
fn nodes_are_named_by_their_config_key_not_their_kind() {
    // Whereas each factory's unit test injects `instance_name` directly, this
    // exercises the whole assembler: the estimator and map-layer loops must
    // thread the `HashMap` key through to the built node's identity. Two nodes
    // of a shared kind under distinct keys would otherwise collide on one name
    // in any name-keyed tooling (observability paths, `channels()`).
    let mut estimators = HashMap::new();
    estimators.insert(
        "nav_ekf".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(IntegratedImuConfig {
                gravity_enu: [0.0, 0.0, -9.81],
                accel_noise_stddev: 0.1,
                gyro_noise_stddev: 0.01,
                accel_bias_instability: 0.001,
                gyro_bias_instability: 0.0001,
                accel_channel: "imu/accel".to_string(),
                gyro_channel: "imu/gyro".to_string(),
            }),
            aiding: vec![],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );

    let mut map_layers = HashMap::new();
    map_layers.insert(
        "local_grid".to_string(),
        MapLayerConfig::OccupancyGrid2D {
            rate: 5.0,
            resolution: 0.1,
            scan_channel: "scan".to_string(),
            width_m: 10.0,
            height_m: 10.0,
            pose_source: Default::default(),
        },
    );

    let stack = AutonomyStack {
        estimators,
        map_layers,
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        consumes_control: false,
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        body,
    )
    .expect("estimator + map-layer stack must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"nav_ekf"),
        "estimator node must carry its config key `nav_ekf`, got {names:?}"
    );
    assert!(
        names.contains(&"local_grid"),
        "map-layer node must carry its config key `local_grid`, got {names:?}"
    );
}

#[test]
fn two_map_layers_of_one_kind_publish_to_distinct_channels() {
    // Two `OccupancyGrid2D` layers under distinct keys must not collide on the
    // map producer slot. Each publishes `MapData` on a channel named by its own
    // config key; a single hardcoded output name would make the second layer a
    // `DuplicateProducer` and fail the build.
    let mut estimators = HashMap::new();
    estimators.insert(
        "nav_ekf".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(IntegratedImuConfig {
                gravity_enu: [0.0, 0.0, -9.81],
                accel_noise_stddev: 0.1,
                gyro_noise_stddev: 0.01,
                accel_bias_instability: 0.001,
                gyro_bias_instability: 0.0001,
                accel_channel: "imu/accel".to_string(),
                gyro_channel: "imu/gyro".to_string(),
            }),
            aiding: vec![],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );

    let occupancy = |scan: &str| MapLayerConfig::OccupancyGrid2D {
        rate: 5.0,
        resolution: 0.1,
        scan_channel: scan.to_string(),
        width_m: 10.0,
        height_m: 10.0,
        pose_source: Default::default(),
    };
    let mut map_layers = HashMap::new();
    map_layers.insert("local".to_string(), occupancy("scan/near"));
    map_layers.insert("global".to_string(), occupancy("scan/far"));

    let stack = AutonomyStack {
        estimators,
        map_layers,
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        consumes_control: false,
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        body,
    )
    .expect("two same-kind map layers under distinct keys must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"local") && names.contains(&"global"),
        "both map-layer nodes must be present, got {names:?}"
    );
}

#[test]
fn build_pipeline_rejects_invalid_config_before_assembly() {
    // A planner reading map level "local" with no such layer declared is a
    // config error. `build_pipeline` runs static validation first, so this
    // surfaces as `InvalidConfig` up front rather than as a downstream
    // `UnsatisfiedInput` on the missing `MapData` producer.
    let mut search_planners = HashMap::new();
    search_planners.insert(
        "local_path".to_string(),
        SearchPlannerConfig::AStar {
            rate: 5.0,
            arrival_tolerance_m: 1.5,
            occupancy_threshold: 180,
            max_search_depth: 20_000,
            enable_path_smoothing: false,
            replan_on_path_deviation: false,
            deviation_tolerance_m: 3.0,
            level: "local".to_string(),
            goal_channel: "mission".to_string(),
        },
    );
    let stack = AutonomyStack {
        search_planners,
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        consumes_control: false,
    };

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        body,
    ) else {
        panic!("a planner with no matching map layer must not build");
    };

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::InvalidConfig(v)
                if v.iter().any(|c| matches!(
                    c,
                    ConfigValidationError::PlannerReferencesUnknownMapLayer { level, .. }
                        if level == "local"
                ))
        )),
        "expected InvalidConfig carrying PlannerReferencesUnknownMapLayer, got {errors:?}"
    );
}
