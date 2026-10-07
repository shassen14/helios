// Validation integration tests: validate_autonomy_config.

use std::collections::HashMap;

use helios_runtime::{
    config::{
        AidingConfig, AllocatorConfig, AugmentationConfig, AutonomyStack, EkfConfig,
        EkfDynamicsConfig, EkfInitialStateConfig, EstimatorConfig, IntegratedImuConfig,
        MockOracleEstimatorConfig, SensorModelConfig,
    },
    validation::{validate_autonomy_config, CapabilitySet, ConfigValidationError},
    AutonomyRegistry,
};

// =========================================================================
// == Helpers ==
// =========================================================================

fn empty_caps() -> CapabilitySet {
    CapabilitySet {
        gaussian_estimators: Default::default(),
        mock_estimators: Default::default(),
        measurement_models: Default::default(),
        allocators: Default::default(),
    }
}

fn full_caps() -> CapabilitySet {
    fn set(items: &[&str]) -> std::collections::HashSet<String> {
        items.iter().map(|s| s.to_string()).collect()
    }
    CapabilitySet {
        gaussian_estimators: set(&["Ekf"]),
        mock_estimators: set(&["MockOracle"]),
        measurement_models: set(&["gps_position", "accelerometer", "gyroscope", "magnetometer"]),
        allocators: set(&["WheelTorque", "SteerPosition"]),
    }
}

fn imu_noise() -> IntegratedImuConfig {
    IntegratedImuConfig {
        gravity_enu: [0.0, 0.0, -9.81],
        accel_noise_stddev: 0.1,
        gyro_noise_stddev: 0.01,
        accel_bias_instability: 0.001,
        gyro_bias_instability: 0.001,
        accel_bias_uncertainty_mps2: 0.1,
        gyro_bias_uncertainty_radps: 0.01,
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

fn wheel_torque_allocator() -> AllocatorConfig {
    AllocatorConfig::WheelTorque {
        input: "drive_cmd".to_string(),
        wheel_radius: 0.3,
        drive: "drive".to_string(),
    }
}

fn steer_position_allocator() -> AllocatorConfig {
    AllocatorConfig::SteerPosition {
        input: "steer_cmd".to_string(),
        steer: "steer".to_string(),
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
    let mut estimators = HashMap::new();
    estimators.insert("primary".to_string(), ekf_config());

    let stack = AutonomyStack {
        nodes: Default::default(),
        estimators,
        allocators: Default::default(),
        reference: None,
        command: Default::default(),
        tf: Default::default(),
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
fn validation_collects_all_errors_two_bad_estimators() {
    let mut estimators = HashMap::new();
    estimators.insert("primary".to_string(), ekf_config());
    estimators.insert("backup".to_string(), ekf_config());
    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors.len() >= 2,
        "Expected at least 2 errors for two unknown estimators, got {}",
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
fn validation_allocators_sharing_an_actuator_error() {
    // Multiple allocators are legal, but not two that claim the same actuator:
    // the terminal merge unions disjoint actuator sets, so a shared `drive` would
    // silently drop one allocator's setpoint. Two wheel-torque allocators both
    // driving `drive` collide. The claimant list is sorted, so the assertion —
    // and the emitted error — is stable regardless of HashMap iteration order.
    let mut allocators = HashMap::new();
    allocators.insert("front_axle".to_string(), wheel_torque_allocator());
    allocators.insert("rear_axle".to_string(), wheel_torque_allocator());
    let stack = AutonomyStack {
        allocators,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::AllocatorActuatorConflict { actuator, allocators }
                if actuator == "drive"
                    && allocators.as_slice() == ["front_axle".to_string(), "rear_axle".to_string()]
        )),
        "Expected AllocatorActuatorConflict for 'drive' claimed by [front_axle, rear_axle], got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn validation_disjoint_multiple_allocators_pass() {
    // The decoupled car: a wheel-torque allocator owning `drive` and a
    // steer-position allocator owning `steer`. Two allocators, disjoint
    // actuators, no shared terminal. The stack validates clean.
    let mut allocators = HashMap::new();
    allocators.insert("drive_alloc".to_string(), wheel_torque_allocator());
    allocators.insert("steer_alloc".to_string(), steer_position_allocator());
    let stack = AutonomyStack {
        allocators,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &full_caps());
    assert!(
        errors.is_empty(),
        "Disjoint multiple allocators must pass, got: {:?}",
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
        sensor_payload: "MagneticField".to_string(),
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
        !errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::AugmentationHasNoAidingSource { .. }
        )),
        "matched aiding source must satisfy the lint, got: {:?}",
        errors.iter().map(|e| e.to_string()).collect::<Vec<_>>()
    );
}

fn mock_oracle_stack() -> AutonomyStack {
    AutonomyStack {
        estimators: HashMap::from([(
            "primary".to_string(),
            EstimatorConfig::MockOracle(MockOracleEstimatorConfig {}),
        )]),
        ..Default::default()
    }
}

// The mock oracle is registered in the mock-estimator family, not the Gaussian
// one. Checked against the real registry, a stack selecting it must validate;
// before, every estimator kind was checked against the Gaussian set only, so
// any MockOracle stack was rejected as an unknown Gaussian estimator.
#[test]
fn mock_oracle_validates_against_the_default_registry() {
    let errors = validate_autonomy_config(
        &mock_oracle_stack(),
        &AutonomyRegistry::default().capabilities(),
    );
    assert!(
        errors.is_empty(),
        "a MockOracle stack must validate, got {errors:?}"
    );
}

#[test]
fn unregistered_mock_estimator_is_reported_as_a_mock() {
    let errors = validate_autonomy_config(&mock_oracle_stack(), &empty_caps());
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ConfigValidationError::UnknownMockEstimator { instance, kind }
                if instance == "primary" && kind == "MockOracle"
        )),
        "expected UnknownMockEstimator for `primary`, got {errors:?}"
    );
}
