// Validation integration tests: validate_autonomy_config.

use std::collections::HashMap;

use helios_runtime::{
    config::{
        AidingConfig, AllocatorConfig, AugmentationConfig, AutonomyStack, CommandArbitrationConfig,
        CommandSource, ControllerConfig, EkfConfig, EkfDynamicsConfig, EkfInitialStateConfig,
        EstimatorConfig, IntegratedImuConfig, MapLayerConfig, MapperPoseSourceConfig,
        SearchPlannerConfig, SensorModelConfig,
    },
    validation::{validate_autonomy_config, CapabilitySet, ConfigValidationError},
};

// =========================================================================
// == Helpers ==
// =========================================================================

fn empty_caps() -> CapabilitySet {
    CapabilitySet {
        gaussian_estimators: Default::default(),
        measurement_models: Default::default(),
        mappers: Default::default(),
        controllers: Default::default(),
        planners: Default::default(),
        allocators: Default::default(),
    }
}

fn full_caps() -> CapabilitySet {
    fn set(items: &[&str]) -> std::collections::HashSet<String> {
        items.iter().map(|s| s.to_string()).collect()
    }
    CapabilitySet {
        gaussian_estimators: set(&["Ekf"]),
        measurement_models: set(&["gps_position", "accelerometer", "gyroscope", "magnetometer"]),
        mappers: set(&["OccupancyGrid2D"]),
        controllers: set(&["DirectTwist"]),
        planners: set(&["AStar"]),
        allocators: set(&["KinematicAckermann"]),
    }
}

fn imu_noise() -> IntegratedImuConfig {
    IntegratedImuConfig {
        gravity_enu: [0.0, 0.0, -9.81],
        accel_noise_stddev: 0.1,
        gyro_noise_stddev: 0.01,
        accel_bias_instability: 0.001,
        gyro_bias_instability: 0.001,
        accel_channel: "sensor.imu.accel".to_string(),
        gyro_channel: "sensor.imu.gyro".to_string(),
    }
}

fn ekf_config() -> EstimatorConfig {
    EstimatorConfig::Ekf(EkfConfig {
        dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
        aiding: vec![],
        augmentation: vec![],
        initial_state: EkfInitialStateConfig::default(),
    })
}

fn direct_twist() -> ControllerConfig {
    ControllerConfig::DirectTwist {
        state_source: Default::default(),
    }
}

fn astar() -> SearchPlannerConfig {
    SearchPlannerConfig::AStar {
        rate: 5.0,
        arrival_tolerance_m: 1.5,
        occupancy_threshold: 180,
        max_search_depth: 50_000,
        enable_path_smoothing: false,
        replan_on_path_deviation: false,
        deviation_tolerance_m: 3.0,
        level: "local".to_string(),
        goal_channel: "mission".to_string(),
    }
}

fn occupancy_grid() -> MapLayerConfig {
    MapLayerConfig::OccupancyGrid2D {
        rate: 10.0,
        resolution: 0.1,
        scan_channel: "sensor.lidar.front".to_string(),
        width_m: 20.0,
        height_m: 20.0,
        pose_source: MapperPoseSourceConfig::GroundTruth,
    }
}

fn stack_with_arbitration(sources: Vec<CommandSource>, with_controller: bool) -> AutonomyStack {
    let mut controllers = HashMap::new();
    if with_controller {
        controllers.insert("main_ctrl".to_string(), direct_twist());
    }
    AutonomyStack {
        controllers,
        command_arbitration: CommandArbitrationConfig {
            sources,
            ..Default::default()
        },
        ..Default::default()
    }
}

fn ackermann_allocator() -> AllocatorConfig {
    AllocatorConfig::KinematicAckermann {
        wheelbase: 2.0,
        wheel_radius: 0.3,
        drive: "drive".to_string(),
        steer: "steer".to_string(),
    }
}

fn stack_with_allocators(
    count: usize,
    with_controller: bool,
    sources: Vec<CommandSource>,
) -> AutonomyStack {
    let mut allocators = HashMap::new();
    for i in 0..count {
        allocators.insert(format!("alloc_{i}"), ackermann_allocator());
    }
    let mut controllers = HashMap::new();
    if with_controller {
        controllers.insert("main_ctrl".to_string(), direct_twist());
    }
    AutonomyStack {
        controllers,
        allocators,
        command_arbitration: CommandArbitrationConfig {
            sources,
            ..Default::default()
        },
        ..Default::default()
    }
}

fn ekf_aiding_entry() -> AidingConfig {
    AidingConfig {
        sensor_payload: "GpsPosition".to_string(),
        model: SensorModelConfig {
            kind: "gps_position".to_string(),
            gravity_enu: [0.0, 0.0, -9.81],
            magnetic_field_enu: None,
        },
        input_channel: "sensor.gps.primary".to_string(),
        r_diag: vec![1.0, 1.0, 1.0],
    }
}

// =========================================================================
// == validate_autonomy_config ==
// =========================================================================

#[test]
fn validation_empty_stack_passes() {
    let stack = AutonomyStack::default();
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(errors.is_empty(), "Empty stack must produce no errors");
}

#[test]
fn validation_valid_full_stack_passes() {
    let mut search_planners = HashMap::new();
    search_planners.insert("local_planner".to_string(), astar());

    let mut controllers = HashMap::new();
    controllers.insert("main_ctrl".to_string(), direct_twist());

    let mut map_layers = HashMap::new();
    map_layers.insert("local".to_string(), occupancy_grid());

    let mut estimators = HashMap::new();
    estimators.insert("primary".to_string(), ekf_config());

    let stack = AutonomyStack {
        estimators,
        map_layers,
        search_planners,
        path_following: None,
        controllers,
        allocators: Default::default(),
        teleop: None,
        command_arbitration: Default::default(),
    };

    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.is_empty(),
        "Valid full stack must produce no errors, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_unknown_estimator_produces_error() {
    let mut estimators = HashMap::new();
    estimators.insert("primary".to_string(), ekf_config());

    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };
    let mut caps = full_caps();
    caps.gaussian_estimators.clear();
    let errors = validate_autonomy_config(&stack, &caps);
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::UnknownGaussianEstimator { kind, .. } if kind == "Ekf"
        )),
        "Expected UnknownGaussianEstimator for Ekf"
    );
}

#[test]
fn validation_unknown_mapper_produces_error() {
    let mut map_layers = HashMap::new();
    map_layers.insert("local".to_string(), occupancy_grid());

    let stack = AutonomyStack {
        map_layers,
        ..Default::default()
    };
    let mut caps = full_caps();
    caps.mappers.clear();
    let errors = validate_autonomy_config(&stack, &caps);
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::UnknownMapper { kind } if kind == "OccupancyGrid2D"
        )),
        "Expected UnknownMapper for OccupancyGrid2D"
    );
}

#[test]
fn validation_unknown_controller_produces_error() {
    let mut controllers = HashMap::new();
    controllers.insert("ctrl".to_string(), direct_twist());
    let stack = AutonomyStack {
        controllers,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors.iter().any(
            |e| matches!(e, ConfigValidationError::UnknownController { kind } if kind == "DirectTwist")
        ),
        "Expected UnknownController for DirectTwist"
    );
}

#[test]
fn validation_unknown_planner_produces_error() {
    let mut search_planners = HashMap::new();
    search_planners.insert("planner".to_string(), astar());
    let stack = AutonomyStack {
        search_planners,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors.iter().any(
            |e| matches!(e, ConfigValidationError::UnknownPlanner { kind } if kind == "AStar")
        ),
        "Expected UnknownPlanner for AStar"
    );
}

#[test]
fn validation_planner_without_matching_map_layer_produces_error() {
    // `astar()` reads level "local", but no map layer of that key is declared,
    // so nothing produces the `MapData` it requires. Caught here rather than as
    // a downstream `UnsatisfiedInput` at DAG build.
    let mut search_planners = HashMap::new();
    search_planners.insert("planner".to_string(), astar());
    let stack = AutonomyStack {
        search_planners,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::PlannerReferencesUnknownMapLayer { planner, level }
                if planner == "planner" && level == "local"
        )),
        "Expected PlannerReferencesUnknownMapLayer for level 'local'"
    );
}

#[test]
fn validation_planner_referencing_none_map_layer_produces_error() {
    // A layer keyed "local" but declared `None` produces no mapper node, so a
    // planner reading level "local" still has no map producer.
    let mut search_planners = HashMap::new();
    search_planners.insert("planner".to_string(), astar());
    let mut map_layers = HashMap::new();
    map_layers.insert("local".to_string(), MapLayerConfig::None);
    let stack = AutonomyStack {
        search_planners,
        map_layers,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::PlannerReferencesUnknownMapLayer { level, .. } if level == "local"
        )),
        "Expected PlannerReferencesUnknownMapLayer for a None-valued layer"
    );
}

#[test]
fn validation_collects_all_errors_two_bad_controllers() {
    let mut controllers = HashMap::new();
    controllers.insert("ctrl1".to_string(), direct_twist());
    controllers.insert("ctrl2".to_string(), direct_twist());
    let stack = AutonomyStack {
        controllers,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors.len() >= 2,
        "Expected at least 2 errors for two unknown controllers, got {}",
        errors.len()
    );
}

#[test]
fn validation_unknown_measurement_model_in_aiding_produces_error() {
    let bad_aiding = AidingConfig {
        sensor_payload: "GpsPosition".to_string(),
        model: SensorModelConfig {
            kind: "nonexistent_model".to_string(),
            gravity_enu: [0.0, 0.0, -9.81],
            magnetic_field_enu: None,
        },
        input_channel: "sensor.gps.primary".to_string(),
        r_diag: vec![1.0, 1.0, 1.0],
    };
    let mut estimators = HashMap::new();
    estimators.insert(
        "primary".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            aiding: vec![bad_aiding],
            augmentation: vec![],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );
    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::UnknownMeasurementModel { model_kind, .. }
                if model_kind == "nonexistent_model"
        )),
        "Expected UnknownMeasurementModel for nonexistent_model, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_unknown_sensor_payload_in_aiding_produces_error() {
    let bad_aiding = AidingConfig {
        sensor_payload: "UnknownSensorType".to_string(),
        model: SensorModelConfig {
            kind: "gps_position".to_string(),
            gravity_enu: [0.0, 0.0, -9.81],
            magnetic_field_enu: None,
        },
        input_channel: "sensor.unknown".to_string(),
        r_diag: vec![1.0],
    };
    let mut estimators = HashMap::new();
    estimators.insert(
        "primary".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            aiding: vec![bad_aiding],
            augmentation: vec![],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );
    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::UnknownSensorPayload { payload_kind, .. }
                if payload_kind == "UnknownSensorType"
        )),
        "Expected UnknownSensorPayload for UnknownSensorType"
    );
}

#[test]
fn validation_autonomy_source_without_controller_errors() {
    let stack = stack_with_arbitration(vec![CommandSource::Autonomy], false);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, ConfigValidationError::AutonomySourceWithoutController)),
        "Expected AutonomySourceWithoutController, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_controller_not_a_command_source_errors() {
    // Controller present, but the explicit source list omits autonomy: the
    // controller's output is never routed to command.
    let stack = stack_with_arbitration(vec![CommandSource::Teleop], true);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::ControllerConfiguredButNotACommandSource
        )),
        "Expected ControllerConfiguredButNotACommandSource, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_duplicate_command_source_errors() {
    let stack = stack_with_arbitration(vec![CommandSource::Teleop, CommandSource::Teleop], false);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::DuplicateCommandSource { source } if source == "teleop"
        )),
        "Expected DuplicateCommandSource for teleop, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_empty_sources_with_controller_passes() {
    // Empty sources infers [Autonomy] when a controller exists; the guards must
    // not over-fire.
    let stack = stack_with_arbitration(vec![], true);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.is_empty(),
        "Empty sources with a controller must pass, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_explicit_autonomy_with_controller_passes() {
    let stack = stack_with_arbitration(vec![CommandSource::Autonomy], true);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.is_empty(),
        "Explicit autonomy source with a controller must pass, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_allocator_without_command_source_errors() {
    // An allocator with no controller and no teleop source has nothing producing
    // the `command` it consumes.
    let stack = stack_with_allocators(1, false, vec![]);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, ConfigValidationError::AllocatorWithoutCommandSource)),
        "Expected AllocatorWithoutCommandSource, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_allocator_with_controller_passes() {
    // A controller produces `command`, so the allocator's input is satisfied.
    let stack = stack_with_allocators(1, true, vec![]);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.is_empty(),
        "Allocator with a controller must pass, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_allocator_with_teleop_source_passes() {
    // A host-published teleop source also produces `command`, so an allocator
    // with no controller is still satisfiable.
    let stack = stack_with_allocators(1, false, vec![CommandSource::Teleop]);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        !errors
            .iter()
            .any(|e| matches!(e, ConfigValidationError::AllocatorWithoutCommandSource)),
        "Teleop source must satisfy the allocator's command input, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_multiple_allocators_errors() {
    // Two allocators collide on the single actuator terminal.
    let stack = stack_with_allocators(2, true, vec![]);
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::MultipleAllocators { count } if *count == 2
        )),
        "Expected MultipleAllocators {{ count: 2 }}, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_valid_aiding_entry_passes() {
    let mut estimators = HashMap::new();
    estimators.insert(
        "primary".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            aiding: vec![ekf_aiding_entry()],
            augmentation: vec![],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );
    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.is_empty(),
        "Valid aiding entry must pass, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

fn mag_bias_augmentation(sensor: &str) -> AugmentationConfig {
    AugmentationConfig {
        kind: helios_core::estimation::augmentation::MAGNETOMETER_BIAS.to_string(),
        sensor: sensor.to_string(),
        init_uncertainty: 5.0,
        random_walk: 0.01,
    }
}

// An augmentation whose `sensor` matches no aiding channel is unobservable —
// nothing ever updates its state columns. The validator must flag it rather
// than let it become a silent runtime no-op.
#[test]
fn validation_augmentation_without_aiding_source_produces_error() {
    let mut estimators = HashMap::new();
    estimators.insert(
        "primary".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            aiding: vec![], // no source feeds the augmentation's sensor
            augmentation: vec![mag_bias_augmentation("sensor.mag.primary")],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );
    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };

    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::AugmentationHasNoAidingSource { sensor, .. }
                if sensor == "sensor.mag.primary"
        )),
        "expected AugmentationHasNoAidingSource, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

// With an aiding entry on the same channel the augmentation names, the block is
// observable and the lint stays silent.
#[test]
fn validation_augmentation_with_matching_aiding_source_passes() {
    let mag_aiding = AidingConfig {
        sensor_payload: "MagneticField3D".to_string(),
        model: SensorModelConfig {
            kind: "magnetometer".to_string(),
            gravity_enu: [0.0, 0.0, -9.81],
            magnetic_field_enu: Some([22.0, 5.0, -42.0]),
        },
        input_channel: "sensor.mag.primary".to_string(),
        r_diag: vec![0.04, 0.04, 0.04],
    };

    let mut estimators = HashMap::new();
    estimators.insert(
        "primary".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            aiding: vec![mag_aiding],
            augmentation: vec![mag_bias_augmentation("sensor.mag.primary")],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );
    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };

    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        !errors
            .iter()
            .any(|e| matches!(e, ConfigValidationError::AugmentationHasNoAidingSource { .. })),
        "matched aiding source must satisfy the lint, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}
