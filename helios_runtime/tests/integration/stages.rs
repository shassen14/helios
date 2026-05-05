// Stage integration tests: EstimationCore, MappingCore, ControlCore.

use std::collections::HashMap;

use helios_core::mapping::MapData;
use helios_runtime::{
    pipeline::{
        control_core::ControlCore, mapping_core::MappingCore,
        path_following_core::PathFollowingCore, PipelineBuilder,
    },
    stage::{LeveledController, PipelineLevel},
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
    let msg = make_gps_message(2.5);
    estimation.process_measurement(&msg, &rt);
    let state = estimation.get_state().unwrap();
    assert!(
        (state.state.timestamp - 2.5).abs() < 1e-9,
        "timestamp should advance to 2.5 after update, got {}",
        state.state.timestamp
    );
}

#[test]
fn estimation_process_measurement_no_estimator_no_panic() {
    let pipeline = PipelineBuilder::new().build();
    let (mut estimation, _, _, _) = pipeline.into_parts();
    let rt = MockRuntime;
    let msg = make_gps_message(1.0);
    estimation.process_measurement(&msg, &rt);
}

// =========================================================================
// == MappingCore ==
// =========================================================================

fn single_local_mapper() -> MappingCore {
    let mut mappers: HashMap<String, Box<dyn helios_core::mapping::Mapper>> = HashMap::new();
    mappers.insert("local".to_string(), Box::new(MockMapper::new()));
    MappingCore { mappers }
}

#[test]
fn mapping_process_messages_calls_mapper() {
    let mut core = single_local_mapper();
    let rt = MockRuntime;
    core.process_messages(&[make_gps_message(0.0)], &rt);
    match core.get_map("local").unwrap() {
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
    match core.get_map("local").unwrap() {
        MapData::OccupancyGrid2D { version, .. } => {
            assert_eq!(*version, 3, "3 messages → 3 process() calls → version = 3")
        }
        _ => panic!("Expected OccupancyGrid2D"),
    }
}

#[test]
fn mapping_get_map_none_for_unregistered_key() {
    let core = single_local_mapper();
    assert!(
        core.get_map("global").is_none(),
        "No 'global' layer registered"
    );
}

#[test]
fn mapping_multiple_layers_each_updated_independently() {
    let mut mappers: HashMap<String, Box<dyn helios_core::mapping::Mapper>> = HashMap::new();
    mappers.insert("local".to_string(), Box::new(MockMapper::new()));
    mappers.insert("global".to_string(), Box::new(MockMapper::new()));
    let mut core = MappingCore { mappers };

    let rt = MockRuntime;
    core.process_messages(&[make_gps_message(0.0)], &rt);

    match core.get_map("local").unwrap() {
        MapData::OccupancyGrid2D { version, .. } => assert_eq!(*version, 1),
        _ => panic!("Expected OccupancyGrid2D"),
    }
    match core.get_map("global").unwrap() {
        MapData::OccupancyGrid2D { version, .. } => assert_eq!(*version, 1),
        _ => panic!("Expected OccupancyGrid2D"),
    }
}

// =========================================================================
// == ControlCore ==
// =========================================================================

#[test]
fn control_step_controllers_returns_none_with_no_controllers() {
    let mut core = ControlCore {
        planners: vec![],
        controllers: vec![],
        current_goal: None,
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
        current_goal: None,
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
