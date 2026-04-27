// Stage integration tests: EstimationCore, MappingCore, ControlCore.

use helios_core::mapping::MapData;
use helios_runtime::{
    pipeline::{
        control_core::ControlCore, mapping_core::MappingCore, path_following_core::PathFollowingCore,
        PipelineBuilder,
    },
    stage::{LeveledController, LeveledMapper, PipelineLevel},
};

use crate::common::{
    make_gps_message, make_path, make_world_state, MockController, MockEstimator, MockMapper,
    MockPathFollower, MockRuntime,
};

// =========================================================================
// == EstimationCore ==
// =========================================================================

#[test]
fn estimation_no_estimator_get_state_none() {
    let pipeline = PipelineBuilder::new().build();
    let (estimation, _, _, _) = pipeline.into_parts();
    assert!(estimation.get_state().is_none());
}

#[test]
fn estimation_with_estimator_get_state_some() {
    let pipeline = PipelineBuilder::new()
        .with_estimator(Box::new(MockEstimator::new()))
        .build();
    let (estimation, _, _, _) = pipeline.into_parts();
    assert!(estimation.get_state().is_some());
}

#[test]
fn estimation_process_measurement_advances_timestamp() {
    let pipeline = PipelineBuilder::new()
        .with_estimator(Box::new(MockEstimator::new()))
        .build();
    let (mut estimation, _, _, _) = pipeline.into_parts();
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
    let (mut estimation, _, _, _) = pipeline.into_parts();
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
            assert_eq!(
                *version, 0,
                "Global mapper must be skipped when slam_active"
            )
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

#[test]
fn control_step_controllers_returns_none_with_no_controllers() {
    let mut core = ControlCore {
        planners: vec![],
        controllers: vec![],
    };
    let state = make_world_state(0.0, 0.0);
    let rt = MockRuntime;
    assert!(core.step_controllers(&state, None, 0.1, &rt).is_none());
}

#[test]
fn control_step_controllers_returns_some_with_controller() {
    let mut core = ControlCore {
        planners: vec![],
        controllers: vec![LeveledController {
            level: PipelineLevel::Local,
            controller: Box::new(MockController),
        }],
    };
    let state = make_world_state(0.0, 0.0);
    let rt = MockRuntime;
    assert!(core.step_controllers(&state, None, 0.1, &rt).is_some());
}

// =========================================================================
// == PathFollowingCore ==
// =========================================================================

#[test]
fn path_following_no_path_step_returns_none() {
    let mut core = PathFollowingCore::new(Box::new(MockPathFollower::new()));
    let state = make_world_state(0.0, 0.0);
    assert!(core.step(&state, 0.1).is_none());
}

#[test]
fn path_following_set_path_step_returns_some() {
    let mut core = PathFollowingCore::new(Box::new(MockPathFollower::new()));
    let path = make_path("local", &[[3.0, 0.0], [6.0, 0.0]]);
    core.set_path(path);
    let state = make_world_state(0.0, 0.0);
    assert!(core.step(&state, 0.1).is_some());
}

#[test]
fn path_following_get_path_none_before_set() {
    let core = PathFollowingCore::new(Box::new(MockPathFollower::new()));
    assert!(core.get_path().is_none());
}

#[test]
fn path_following_get_path_some_after_set() {
    let mut core = PathFollowingCore::new(Box::new(MockPathFollower::new()));
    let path = make_path("local", &[[1.0, 0.0]]);
    core.set_path(path);
    assert!(core.get_path().is_some());
}

#[test]
fn path_following_get_lookahead_waypoint_returns_first_waypoint() {
    let mut core = PathFollowingCore::new(Box::new(MockPathFollower::new()));
    let path = make_path("local", &[[3.0, 4.0], [7.0, 8.0]]);
    core.set_path(path);
    let wp = core.get_lookahead_waypoint().unwrap();
    assert!(
        (wp.state.vector[0] - 3.0).abs() < 1e-9,
        "First waypoint x = 3.0"
    );
    assert!(
        (wp.state.vector[1] - 4.0).abs() < 1e-9,
        "First waypoint y = 4.0"
    );
}
