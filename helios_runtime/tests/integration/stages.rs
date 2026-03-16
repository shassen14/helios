// Stage integration tests: EstimationCore, MappingCore, ControlCore.

use std::collections::HashMap;

use helios_core::mapping::MapData;
use helios_runtime::{
    pipeline::{
        control_core::ControlCore, mapping_core::MappingCore,
        PipelineBuilder,
    },
    stage::{LeveledController, LeveledMapper, PipelineLevel},
};

use crate::common::{
    make_gps_message, make_path, make_world_state, MockController, MockEstimator, MockMapper,
    MockRuntime,
};

// =========================================================================
// == EstimationCore ==
// =========================================================================

#[test]
fn estimation_no_estimator_get_state_none() {
    let pipeline = PipelineBuilder::new().build();
    let (estimation, _, _) = pipeline.into_parts();
    assert!(estimation.get_state().is_none());
}

#[test]
fn estimation_with_estimator_get_state_some() {
    let pipeline = PipelineBuilder::new()
        .with_estimator(Box::new(MockEstimator::new()))
        .build();
    let (estimation, _, _) = pipeline.into_parts();
    assert!(estimation.get_state().is_some());
}

#[test]
fn estimation_process_measurement_advances_timestamp() {
    let pipeline = PipelineBuilder::new()
        .with_estimator(Box::new(MockEstimator::new()))
        .build();
    let (mut estimation, _, _) = pipeline.into_parts();
    let rt = MockRuntime;
    // Timestamp 0.0 → 2.5: dt = 2.5 > 1e-9, so predict + update both run.
    // MockEstimator::update sets last_update_timestamp = message.timestamp.
    let msg = make_gps_message(2.5);
    estimation.process_measurement(&msg, &rt);
    let state = estimation.get_state().unwrap();
    assert!(
        (state.last_update_timestamp - 2.5).abs() < 1e-9,
        "timestamp should advance to 2.5 after update, got {}",
        state.last_update_timestamp
    );
}

#[test]
fn estimation_process_measurement_no_estimator_no_panic() {
    let pipeline = PipelineBuilder::new().build();
    let (mut estimation, _, _) = pipeline.into_parts();
    let rt = MockRuntime;
    let msg = make_gps_message(1.0);
    // Must not panic when there is no estimator registered.
    estimation.process_measurement(&msg, &rt);
}

// =========================================================================
// == MappingCore ==
// =========================================================================

fn single_local_mapper() -> MappingCore {
    MappingCore {
        mappers: vec![LeveledMapper {
            level: PipelineLevel::Local,
            mapper: Box::new(MockMapper::new()),
        }],
        slam_active: false,
    }
}

#[test]
fn mapping_process_messages_calls_mapper() {
    let mut core = single_local_mapper();
    let rt = MockRuntime;
    core.process_messages(&[make_gps_message(0.0)], &rt);
    match core.get_map(&PipelineLevel::Local).unwrap() {
        MapData::OccupancyGrid2D { version, .. } => assert_eq!(*version, 1),
        _ => panic!("Expected OccupancyGrid2D"),
    }
}

#[test]
fn mapping_process_messages_once_per_message_per_mapper() {
    let mut core = single_local_mapper();
    let rt = MockRuntime;
    let msgs = vec![
        make_gps_message(0.0),
        make_gps_message(1.0),
        make_gps_message(2.0),
    ];
    core.process_messages(&msgs, &rt);
    match core.get_map(&PipelineLevel::Local).unwrap() {
        MapData::OccupancyGrid2D { version, .. } => {
            assert_eq!(*version, 3, "3 messages → 3 process() calls → version = 3")
        }
        _ => panic!("Expected OccupancyGrid2D"),
    }
}

#[test]
fn mapping_global_mapper_skipped_when_slam_active() {
    let mut core = MappingCore {
        mappers: vec![
            LeveledMapper {
                level: PipelineLevel::Global,
                mapper: Box::new(MockMapper::new()),
            },
            LeveledMapper {
                level: PipelineLevel::Local,
                mapper: Box::new(MockMapper::new()),
            },
        ],
        slam_active: true,
    };
    let rt = MockRuntime;
    core.process_messages(&[make_gps_message(0.0)], &rt);
    match core.get_map(&PipelineLevel::Global).unwrap() {
        MapData::OccupancyGrid2D { version, .. } => {
            assert_eq!(*version, 0, "Global mapper must be skipped when slam_active")
        }
        _ => panic!("Expected OccupancyGrid2D"),
    }
}

#[test]
fn mapping_local_mapper_still_runs_when_slam_active() {
    let mut core = MappingCore {
        mappers: vec![
            LeveledMapper {
                level: PipelineLevel::Global,
                mapper: Box::new(MockMapper::new()),
            },
            LeveledMapper {
                level: PipelineLevel::Local,
                mapper: Box::new(MockMapper::new()),
            },
        ],
        slam_active: true,
    };
    let rt = MockRuntime;
    core.process_messages(&[make_gps_message(0.0)], &rt);
    match core.get_map(&PipelineLevel::Local).unwrap() {
        MapData::OccupancyGrid2D { version, .. } => {
            assert_eq!(*version, 1, "Local mapper must still run when slam_active")
        }
        _ => panic!("Expected OccupancyGrid2D"),
    }
}

#[test]
fn mapping_get_map_none_for_unregistered_level() {
    let core = single_local_mapper();
    assert!(
        core.get_map(&PipelineLevel::Global).is_none(),
        "No Global mapper registered"
    );
}

// =========================================================================
// == ControlCore ==
// =========================================================================

fn empty_control_core() -> ControlCore {
    ControlCore {
        planners: vec![],
        controllers: vec![],
        cached_paths: HashMap::new(),
        lookahead_indices: HashMap::new(),
    }
}

#[test]
fn control_set_path_caches_at_correct_level() {
    let mut core = empty_control_core();
    let path = make_path("local", &[[1.0, 0.0], [5.0, 0.0]]);
    core.set_path(PipelineLevel::Local, path);
    assert!(
        core.get_cached_path(&PipelineLevel::Local).is_some(),
        "Path must be cached at Local level"
    );
    assert!(
        core.get_cached_path(&PipelineLevel::Global).is_none(),
        "Global level must be unset"
    );
}

#[test]
fn control_set_path_resets_lookahead_to_zero() {
    let mut core = empty_control_core();
    // Pre-set a non-zero index to verify reset.
    core.lookahead_indices.insert(PipelineLevel::Local, 5);
    let path = make_path("local", &[[1.0, 0.0]]);
    core.set_path(PipelineLevel::Local, path);
    assert_eq!(
        *core.lookahead_indices.get(&PipelineLevel::Local).unwrap(),
        0,
        "set_path must reset lookahead index to 0"
    );
}

#[test]
fn control_get_active_lookahead_returns_first_waypoint() {
    let mut core = empty_control_core();
    let path = make_path("local", &[[3.0, 4.0], [7.0, 8.0]]);
    core.set_path(PipelineLevel::Local, path);
    let wp = core
        .get_active_lookahead_waypoint(&PipelineLevel::Local)
        .unwrap();
    assert!(
        (wp.state.vector[0] - 3.0).abs() < 1e-9,
        "First waypoint x = 3.0"
    );
    assert!(
        (wp.state.vector[1] - 4.0).abs() < 1e-9,
        "First waypoint y = 4.0"
    );
}

#[test]
fn control_step_controllers_returns_none_with_no_controllers() {
    let mut core = empty_control_core();
    let state = make_world_state(0.0, 0.0);
    let rt = MockRuntime;
    assert!(core.step_controllers(&state, 0.1, &rt).is_none());
}

#[test]
fn control_step_controllers_returns_some_with_controller() {
    let mut core = ControlCore {
        planners: vec![],
        controllers: vec![LeveledController {
            level: PipelineLevel::Local,
            controller: Box::new(MockController),
        }],
        cached_paths: HashMap::new(),
        lookahead_indices: HashMap::new(),
    };
    let state = make_world_state(0.0, 0.0);
    let rt = MockRuntime;
    assert!(core.step_controllers(&state, 0.1, &rt).is_some());
}

#[test]
fn control_lookahead_advances_when_robot_close_to_waypoint() {
    // Robot at (0,0). Waypoints: (1,0) then (5,0).
    // Distance to first waypoint = 1.0 m < 2.0 m threshold → index advances to 1.
    let mut core = empty_control_core();
    let path = make_path("local", &[[1.0, 0.0], [5.0, 0.0]]);
    core.set_path(PipelineLevel::Local, path);

    let state = make_world_state(0.0, 0.0);
    let rt = MockRuntime;
    core.step_controllers(&state, 0.1, &rt);

    assert_eq!(
        *core.lookahead_indices.get(&PipelineLevel::Local).unwrap(),
        1,
        "Lookahead index must advance past the nearby waypoint"
    );
}

#[test]
fn control_lookahead_does_not_advance_when_robot_far() {
    // Robot at (0,0). Waypoints: (5,0) then (10,0).
    // Distance to first waypoint = 5.0 m >= 2.0 m threshold → index stays at 0.
    let mut core = empty_control_core();
    let path = make_path("local", &[[5.0, 0.0], [10.0, 0.0]]);
    core.set_path(PipelineLevel::Local, path);

    let state = make_world_state(0.0, 0.0);
    let rt = MockRuntime;
    core.step_controllers(&state, 0.1, &rt);

    assert_eq!(
        *core.lookahead_indices.get(&PipelineLevel::Local).unwrap(),
        0,
        "Lookahead index must not advance when robot is far from waypoint"
    );
}
