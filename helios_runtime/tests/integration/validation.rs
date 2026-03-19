// Validation integration tests: validate_autonomy_config and PipelineLevel ordering.

use std::collections::HashMap;

use helios_runtime::{
    config::{
        AutonomyStack, ControllerConfig, EkfConfig, EkfDynamicsConfig, EstimatorConfig,
        ImuProcessNoiseConfig, MapperConfig, MapperPoseSourceConfig, PlannerConfig,
        WorldModelConfig,
    },
    stage::PipelineLevel,
    validation::{validate_autonomy_config, CapabilitySet, ValidationError},
};

// =========================================================================
// == Helpers ==
// =========================================================================

fn empty_caps() -> CapabilitySet {
    CapabilitySet {
        estimators: Default::default(),
        dynamics: Default::default(),
        mappers: Default::default(),
        slam: Default::default(),
        controllers: Default::default(),
        planners: Default::default(),
    }
}

fn full_caps() -> CapabilitySet {
    fn set(items: &[&str]) -> std::collections::HashSet<String> {
        items.iter().map(|s| s.to_string()).collect()
    }
    CapabilitySet {
        estimators: set(&["Ekf"]),
        dynamics: set(&["IntegratedImu", "AckermannOdometry"]),
        mappers: set(&["OccupancyGrid2D"]),
        slam: set(&["EkfSlam"]),
        controllers: set(&["Pid", "Lqr", "FeedforwardPid"]),
        planners: set(&["AStar"]),
    }
}

fn imu_noise() -> ImuProcessNoiseConfig {
    ImuProcessNoiseConfig {
        accel_noise_stddev: 0.1,
        gyro_noise_stddev: 0.01,
        accel_bias_instability: 0.001,
        gyro_bias_instability: 0.001,
    }
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

fn astar() -> PlannerConfig {
    PlannerConfig::AStar {
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
    let mut planners = HashMap::new();
    planners.insert("local_planner".to_string(), astar());

    let mut controllers = HashMap::new();
    controllers.insert("main_ctrl".to_string(), pid());

    let stack = AutonomyStack {
        world_model: Some(WorldModelConfig::Separate {
            estimator: Some(EstimatorConfig::Ekf(EkfConfig {
                dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            })),
            mapper: Some(MapperConfig::OccupancyGrid2D {
                rate: 10.0,
                resolution: 0.1,
                width_m: 20.0,
                height_m: 20.0,
                pose_source: MapperPoseSourceConfig::GroundTruth,
            }),
        }),
        planners,
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
    let stack = AutonomyStack {
        world_model: Some(WorldModelConfig::Separate {
            estimator: Some(EstimatorConfig::Ekf(EkfConfig {
                dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            })),
            mapper: None,
        }),
        ..Default::default()
    };
    // Ekf not in capabilities
    let mut caps = full_caps();
    caps.estimators.clear();
    let errors = validate_autonomy_config(&stack, &caps);
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, ValidationError::UnknownEstimator { kind } if kind == "Ekf")),
        "Expected UnknownEstimator for Ekf"
    );
}

#[test]
fn validation_unknown_dynamics_produces_error() {
    let stack = AutonomyStack {
        world_model: Some(WorldModelConfig::Separate {
            estimator: Some(EstimatorConfig::Ekf(EkfConfig {
                dynamics: EkfDynamicsConfig::IntegratedImu(imu_noise()),
            })),
            mapper: None,
        }),
        ..Default::default()
    };
    // IntegratedImu not in capabilities
    let mut caps = full_caps();
    caps.dynamics.clear();
    let errors = validate_autonomy_config(&stack, &caps);
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ValidationError::UnknownDynamics { kind } if kind == "IntegratedImu"
        )),
        "Expected UnknownDynamics for IntegratedImu"
    );
}

#[test]
fn validation_unknown_mapper_produces_error() {
    let stack = AutonomyStack {
        world_model: Some(WorldModelConfig::Separate {
            estimator: None,
            mapper: Some(MapperConfig::OccupancyGrid2D {
                rate: 10.0,
                resolution: 0.1,
                width_m: 20.0,
                height_m: 20.0,
                pose_source: MapperPoseSourceConfig::GroundTruth,
            }),
        }),
        ..Default::default()
    };
    // OccupancyGrid2D not in capabilities
    let mut caps = full_caps();
    caps.mappers.clear();
    let errors = validate_autonomy_config(&stack, &caps);
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ValidationError::UnknownMapper { kind } if kind == "OccupancyGrid2D"
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
    // Pid not in capabilities
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, ValidationError::UnknownController { kind } if kind == "Pid")),
        "Expected UnknownController for Pid"
    );
}

#[test]
fn validation_unknown_planner_produces_error() {
    let mut planners = HashMap::new();
    planners.insert("planner".to_string(), astar());
    let stack = AutonomyStack {
        planners,
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, ValidationError::UnknownPlanner { kind } if kind == "AStar")),
        "Expected UnknownPlanner for AStar"
    );
}

#[test]
fn validation_unknown_slam_produces_error() {
    use helios_runtime::config::{EkfSlamConfig, SlamConfig};
    let stack = AutonomyStack {
        world_model: Some(WorldModelConfig::CombinedSlam {
            slam: SlamConfig::EkfSlam(EkfSlamConfig {}),
        }),
        ..Default::default()
    };
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, ValidationError::UnknownSlam { kind } if kind == "EkfSlam")),
        "Expected UnknownSlam for EkfSlam"
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
    // FeedforwardPid is known, but dynamics_key is not
    let mut caps = full_caps();
    caps.dynamics.clear();
    let errors = validate_autonomy_config(&stack, &caps);
    assert!(
        errors.iter().any(|e| matches!(
            e,
            ValidationError::UnknownControllerDynamics { dynamics_key, .. }
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
    // Neither Pid nor Lqr is registered
    let errors = validate_autonomy_config(&stack, &empty_caps());
    assert!(
        errors.len() >= 2,
        "Expected at least 2 errors for two unknown controllers, got {}",
        errors.len()
    );
}

// =========================================================================
// == PipelineLevel ordering ==
// =========================================================================

#[test]
fn pipeline_level_global_less_than_local() {
    assert!(PipelineLevel::Global < PipelineLevel::Local);
}

#[test]
fn pipeline_level_local_less_than_custom() {
    assert!(PipelineLevel::Local < PipelineLevel::Custom("anything".to_string()));
}

#[test]
fn pipeline_level_custom_variants_sorted_lexicographically() {
    assert!(PipelineLevel::Custom("alpha".to_string()) < PipelineLevel::Custom("beta".to_string()));
    assert!(PipelineLevel::Custom("a".to_string()) < PipelineLevel::Custom("b".to_string()));
}
