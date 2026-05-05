// Pipeline integration tests: GroundTruthPassthrough, StaticMapProvider,
// PipelineBuilder, and AutonomyPipeline.

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

use helios_core::mapping::MapData;
use helios_runtime::{
    estimation::{EstimationDriver, GroundTruthPassthrough},
    mapping::{MapDriver, StaticMapProvider},
    pipeline::PipelineBuilder,
};

use crate::common::{make_gps_message, MockEstimator, MockMapper, MockRuntime};

// =========================================================================
// == GroundTruthPassthrough ==
// =========================================================================

#[test]
fn gt_none_before_injection() {
    let gt = GroundTruthPassthrough::default();
    assert!(gt.get_state().is_none());
}

#[test]
fn gt_process_measurement_noop() {
    let mut gt = GroundTruthPassthrough::default();
    let rt = MockRuntime;
    let msg = make_gps_message(1.0);
    gt.process_measurement(&msg, &rt);
    assert!(
        gt.get_state().is_none(),
        "process_measurement must be a no-op for GroundTruthPassthrough"
    );
}

#[test]
fn gt_inject_sets_position() {
    let mut gt = GroundTruthPassthrough::default();
    let pose = Isometry3::from_parts(Translation3::new(3.0, 4.0, 5.0), UnitQuaternion::identity());
    gt.inject_ground_truth(&pose, Vector3::zeros(), 0.0);
    let state = gt.get_state().unwrap();
    assert!((state.state.vector[0] - 3.0).abs() < 1e-9, "Px mismatch");
    assert!((state.state.vector[1] - 4.0).abs() < 1e-9, "Py mismatch");
    assert!((state.state.vector[2] - 5.0).abs() < 1e-9, "Pz mismatch");
}

#[test]
fn gt_inject_sets_velocity() {
    let mut gt = GroundTruthPassthrough::default();
    gt.inject_ground_truth(&Isometry3::identity(), Vector3::new(1.0, 2.0, 3.0), 0.0);
    let state = gt.get_state().unwrap();
    assert!((state.state.vector[3] - 1.0).abs() < 1e-9, "Vx mismatch");
    assert!((state.state.vector[4] - 2.0).abs() < 1e-9, "Vy mismatch");
    assert!((state.state.vector[5] - 3.0).abs() < 1e-9, "Vz mismatch");
}

#[test]
fn gt_inject_identity_quaternion() {
    let mut gt = GroundTruthPassthrough::default();
    gt.inject_ground_truth(&Isometry3::identity(), Vector3::zeros(), 0.0);
    let state = gt.get_state().unwrap();
    let q = state.get_orientation().unwrap();
    assert!((q.i - 0.0).abs() < 1e-9, "Qx should be 0");
    assert!((q.j - 0.0).abs() < 1e-9, "Qy should be 0");
    assert!((q.k - 0.0).abs() < 1e-9, "Qz should be 0");
    assert!((q.w - 1.0).abs() < 1e-9, "Qw should be 1");
}

#[test]
fn gt_inject_nontrivial_rotation() {
    let mut gt = GroundTruthPassthrough::default();
    let q =
        UnitQuaternion::from_axis_angle(&nalgebra::Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
    let pose = Isometry3::from_parts(Translation3::identity(), q);
    gt.inject_ground_truth(&pose, Vector3::zeros(), 0.0);
    let state = gt.get_state().unwrap();
    let extracted = state.get_orientation().unwrap();
    assert!(
        extracted.k.abs() > 0.5,
        "90° Z-rotation should produce large k component, got {}",
        extracted.k
    );
}

// =========================================================================
// == StaticMapProvider ==
// =========================================================================

#[test]
fn static_map_grid_dimensions() {
    let provider = StaticMapProvider::from_fixture(10.0, 20.0, 1.0, Isometry3::identity(), "local");
    match provider.get_map("local").unwrap() {
        MapData::OccupancyGrid2D { data, .. } => {
            assert_eq!(data.ncols(), 10, "cols = width / resolution");
            assert_eq!(data.nrows(), 20, "rows = height / resolution");
        }
        _ => panic!("Expected OccupancyGrid2D"),
    }
}

#[test]
fn static_map_returns_at_registered_layer() {
    let provider =
        StaticMapProvider::from_fixture(10.0, 10.0, 1.0, Isometry3::identity(), "global");
    assert!(provider.get_map("global").is_some());
}

#[test]
fn static_map_returns_none_at_wrong_layer() {
    let provider =
        StaticMapProvider::from_fixture(10.0, 10.0, 1.0, Isometry3::identity(), "global");
    assert!(provider.get_map("local").is_none());
}

#[test]
fn static_map_version_stays_zero_after_updates() {
    let mut provider =
        StaticMapProvider::from_fixture(10.0, 10.0, 1.0, Isometry3::identity(), "local");
    let rt = MockRuntime;
    let msg = make_gps_message(0.0);
    provider.process_messages(&[msg], &rt);
    provider.process_pose_update(Isometry3::identity());
    match provider.get_map("local").unwrap() {
        MapData::OccupancyGrid2D { version, .. } => {
            assert_eq!(
                *version, 0,
                "StaticMapProvider must never increment version"
            );
        }
        _ => panic!("Expected OccupancyGrid2D"),
    }
}

// =========================================================================
// == PipelineBuilder + AutonomyPipeline ==
// =========================================================================

#[test]
fn empty_pipeline_all_queries_none() {
    let mut pipeline = PipelineBuilder::new().build();
    let rt = MockRuntime;
    let msg = make_gps_message(1.0);
    let output = pipeline.process_measurement(&msg, &rt);
    assert!(output.ego_state.is_none());
    assert!(output.global_map.is_none());
    assert!(output.local_map.is_none());
    assert!(pipeline.get_state().is_none());
    assert!(pipeline.get_map("global").is_none());
}

#[test]
fn with_estimator_state_is_some_after_measurement() {
    let mut pipeline = PipelineBuilder::new()
        .with_estimator(Box::new(MockEstimator::new()))
        .build();
    let rt = MockRuntime;
    let msg = make_gps_message(1.0);
    pipeline.process_measurement(&msg, &rt);
    assert!(pipeline.get_state().is_some());
}

#[test]
fn mappers_keyed_by_string_are_each_accessible() {
    let pipeline = PipelineBuilder::new()
        .with_mapper("local", Box::new(MockMapper::new()))
        .with_mapper("global", Box::new(MockMapper::new()))
        .build();
    let (_, mapping, _, _) = pipeline.into_parts();
    assert!(mapping.get_map("local").is_some());
    assert!(mapping.get_map("global").is_some());
    assert!(mapping.get_map("semantic").is_none());
}

#[test]
fn process_mapper_pose_update_delegates_to_mapping_core() {
    let mut pipeline = PipelineBuilder::new()
        .with_mapper("local", Box::new(MockMapper::new()))
        .build();
    pipeline.process_mapper_pose_update(Isometry3::identity());
    match pipeline.get_map("local").unwrap() {
        MapData::OccupancyGrid2D { version, .. } => {
            assert_eq!(*version, 1, "mapper must have been called exactly once");
        }
        _ => panic!("Expected OccupancyGrid2D"),
    }
}
