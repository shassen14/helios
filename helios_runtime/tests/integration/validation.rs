// Validation integration tests: validate_autonomy_config.

use std::collections::HashMap;

use helios_runtime::{
    config::{
        AidingConfig, AutonomyStack, ControllerConfig, EkfConfig, EkfDynamicsConfig,
        EkfInitialStateConfig, EstimatorConfig, IntegratedImuConfig, MapLayerConfig,
        MapperPoseSourceConfig, SearchPlannerConfig, SensorModelConfig,
    },
    validation::{validate_autonomy_config, CapabilitySet, ConfigValidationError},
};

// =========================================================================
// == Helpers ==
// =========================================================================

fn empty_caps() -> CapabilitySet {
    CapabilitySet {
        gaussian_estimators: Default::default(),
        dynamics: Default::default(),
        measurement_models: Default::default(),
        mappers: Default::default(),
        controllers: Default::default(),
        planners: Default::default(),
    }
}

fn full_caps() -> CapabilitySet {
    fn set(items: &[&str]) -> std::collections::HashSet<String> {
        items.iter().map(|s| s.to_string()).collect()
    }
    CapabilitySet {
        gaussian_estimators: set(&["Ekf"]),
        dynamics: set(&["IntegratedImu", "AckermannOdometry"]),
        measurement_models: set(&["gps_position", "accelerometer", "gyroscope", "magnetometer"]),
        mappers: set(&["OccupancyGrid2D"]),
        controllers: set(&["Pid", "Lqr", "FeedforwardPid"]),
        planners: set(&["AStar"]),
    }
}

fn imu_noise() -> IntegratedImuConfig {
    IntegratedImuConfig {
        gravity: 9.81,
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
        initial_state: EkfInitialStateConfig::default(),
    })
}

fn pid() -> ControllerConfig {
    ControllerConfig::Pid {
        rate: 10.0,
        kp: 1.0,
        ki: 0.0,
        kd: 0.0,
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

fn ekf_aiding_entry() -> AidingConfig {
    AidingConfig {
        sensor_payload: "GpsPosition".to_string(),
        model: SensorModelConfig {
            kind: "gps_position".to_string(),
            gravity: 9.81,
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
    controllers.insert("main_ctrl".to_string(), pid());

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
fn validation_unknown_dynamics_produces_error() {
    let mut estimators = HashMap::new();
    estimators.insert("primary".to_string(), ekf_config());

    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };
    let mut caps = full_caps();
    caps.dynamics.clear();
    let errors = validate_autonomy_config(&stack, &caps);
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::UnknownDynamics { kind } if kind == "IntegratedImu"
        )),
        "Expected UnknownDynamics for IntegratedImu"
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
    controllers.insert("ctrl".to_string(), pid());
    let stack = AutonomyStack {
        controllers,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors.iter().any(
            |e| matches!(e, ConfigValidationError::UnknownController { kind } if kind == "Pid")
        ),
        "Expected UnknownController for Pid"
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
fn validation_feedforward_pid_unknown_dynamics_key_produces_error() {
    let mut controllers = HashMap::new();
    controllers.insert(
        "ffpid".to_string(),
        ControllerConfig::FeedforwardPid {
            dynamics_key: "MyCustomDynamics".to_string(),
            kp: vec![1.0],
            ki: vec![0.0],
            kd: vec![0.0],
            u_min: vec![],
            u_max: vec![],
            controlled_indices: vec![],
            state_source: Default::default(),
        },
    );
    let stack = AutonomyStack {
        controllers,
        ..Default::default()
    };
    let mut caps = full_caps();
    caps.dynamics.clear();
    let errors = validate_autonomy_config(&stack, &caps);
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::UnknownControllerDynamics { dynamics_key, .. }
                if dynamics_key == "MyCustomDynamics"
        )),
        "Expected UnknownControllerDynamics for MyCustomDynamics"
    );
}

#[test]
fn validation_collects_all_errors_two_bad_controllers() {
    let mut controllers = HashMap::new();
    controllers.insert("ctrl1".to_string(), pid());
    controllers.insert(
        "ctrl2".to_string(),
        ControllerConfig::Lqr {
            gain_matrix: vec![],
            state_dim: 0,
            control_dim: 0,
            u_min: vec![],
            u_max: vec![],
            state_source: Default::default(),
        },
    );
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
            gravity: 9.81,
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
            gravity: 9.81,
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
fn validation_valid_aiding_entry_passes() {
    let mut estimators = HashMap::new();
    estimators.insert(
        "primary".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            aiding: vec![ekf_aiding_entry()],
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
