// Shared mock types and helpers for helios_runtime integration tests.
#![allow(dead_code)]

use std::any::Any;

use nalgebra::{DMatrix, DVector, Isometry3};

use helios_core::{
    control::{ControlContext, ControlOutput, Controller},
    estimation::{FilterContext, StateEstimator},
    frames::{FrameAwareState, FrameId, StateVariable},
    mapping::{MapData, Mapper},
    messages::{MeasurementData, MeasurementMessage, ModuleInput},
    models::estimation::dynamics::EstimationDynamics,
    planning::{
        context::PlannerContext,
        types::{Path, PlannerGoal, PlannerResult, PlannerStatus},
        Planner,
    },
    tracking::{Track, Tracker},
    types::{Control, FrameHandle, MonotonicTime, State, TrajectoryPoint},
};
use helios_runtime::runtime::AgentRuntime;

// =========================================================================
// == MockRuntime ==
// =========================================================================

pub struct MockRuntime;

impl AgentRuntime for MockRuntime {
    fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
        Some(Isometry3::identity())
    }

    fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
        Some(Isometry3::identity())
    }

    fn now(&self) -> MonotonicTime {
        MonotonicTime(0.0)
    }
}

// =========================================================================
// == MockDynamics ==
// =========================================================================

#[derive(Debug)]
pub struct MockDynamics;

impl EstimationDynamics for MockDynamics {
    fn get_control_dim(&self) -> usize {
        2
    }

    fn get_control_from_measurement(
        &self,
        _data: &helios_core::messages::MeasurementData,
    ) -> Option<DVector<f64>> {
        None
    }

    fn get_derivatives(&self, x: &State, _u: &Control, _t: f64) -> State {
        DVector::zeros(x.nrows())
    }

    fn calculate_jacobian(
        &self,
        x: &State,
        u: &Control,
        _t: f64,
    ) -> (nalgebra::DMatrix<f64>, nalgebra::DMatrix<f64>) {
        (
            DMatrix::zeros(x.nrows(), x.nrows()),
            DMatrix::zeros(x.nrows(), u.nrows()),
        )
    }
}

// =========================================================================
// == MockEstimator ==
// =========================================================================

pub struct MockEstimator {
    state: FrameAwareState,
    dynamics: MockDynamics,
}

impl MockEstimator {
    pub fn new() -> Self {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Pz(FrameId::World),
            StateVariable::Qx(FrameId::World, FrameId::World),
            StateVariable::Qy(FrameId::World, FrameId::World),
            StateVariable::Qz(FrameId::World, FrameId::World),
            StateVariable::Qw(FrameId::World, FrameId::World),
        ];
        Self {
            state: FrameAwareState::new(layout, 1e-6, 0.0),
            dynamics: MockDynamics,
        }
    }
}

impl StateEstimator for MockEstimator {
    fn predict(&mut self, _dt: f64, _u: &Control, _context: &FilterContext) {}

    fn update(&mut self, message: &MeasurementMessage, _context: &FilterContext) {
        self.state.last_update_timestamp = message.timestamp;
    }

    fn get_state(&self) -> &FrameAwareState {
        &self.state
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }

    fn get_dynamics_model(&self) -> &dyn EstimationDynamics {
        &self.dynamics
    }
}

// =========================================================================
// == MockMapper ==
// =========================================================================

pub struct MockMapper {
    map: MapData,
}

impl MockMapper {
    pub fn new() -> Self {
        Self {
            map: MapData::OccupancyGrid2D {
                origin: Isometry3::identity(),
                resolution: 1.0,
                data: DMatrix::zeros(10, 10),
                version: 0,
            },
        }
    }

    pub fn version(&self) -> u64 {
        match &self.map {
            MapData::OccupancyGrid2D { version, .. } => *version,
            _ => 0,
        }
    }
}

impl Mapper for MockMapper {
    fn process(&mut self, _input: &ModuleInput, _context: &FilterContext) {
        if let MapData::OccupancyGrid2D { version, .. } = &mut self.map {
            *version += 1;
        }
    }

    fn get_map(&self) -> &MapData {
        &self.map
    }
}

// =========================================================================
// == MockTracker ==
// =========================================================================

pub struct MockTracker;

impl Tracker for MockTracker {
    fn update(&mut self, _msg: &MeasurementMessage, _context: &FilterContext) {}

    fn get_tracks(&self) -> &[Track] {
        &[]
    }
}

// =========================================================================
// == MockPlanner ==
// =========================================================================

pub struct MockPlanner {
    pub goal: Option<PlannerGoal>,
}

impl MockPlanner {
    pub fn new() -> Self {
        Self { goal: None }
    }
}

impl Planner for MockPlanner {
    fn set_goal(&mut self, goal: PlannerGoal) {
        self.goal = Some(goal);
    }

    fn plan(
        &mut self,
        _state: &FrameAwareState,
        _map: &MapData,
        _ctx: &PlannerContext,
    ) -> PlannerResult {
        PlannerResult::NoGoal
    }

    fn should_replan(&self, _state: &FrameAwareState, _ctx: &PlannerContext) -> bool {
        false
    }

    fn status(&self) -> PlannerStatus {
        PlannerStatus::Idle
    }

    fn current_path(&self) -> Option<&Path> {
        None
    }
}

// =========================================================================
// == MockController ==
// =========================================================================

pub struct MockController;

impl Controller for MockController {
    fn compute(
        &mut self,
        _state: &FrameAwareState,
        _dt: f64,
        _ctx: &ControlContext,
    ) -> ControlOutput {
        ControlOutput::Raw(DVector::zeros(2))
    }

    fn reset(&mut self) {}
}

// =========================================================================
// == Helpers ==
// =========================================================================

/// Builds a GPS MeasurementMessage at the given timestamp.
pub fn make_gps_message(t: f64) -> MeasurementMessage {
    MeasurementMessage {
        agent_handle: FrameHandle(0),
        sensor_handle: FrameHandle(1),
        timestamp: t,
        data: MeasurementData::GpsPosition(nalgebra::Vector3::zeros()),
    }
}

/// Builds a FrameAwareState with Px/Py/Pz + quaternion in World frame,
/// with the robot positioned at (px, py, 0).
pub fn make_world_state(px: f64, py: f64) -> FrameAwareState {
    let layout = vec![
        StateVariable::Px(FrameId::World),
        StateVariable::Py(FrameId::World),
        StateVariable::Pz(FrameId::World),
        StateVariable::Qx(FrameId::World, FrameId::World),
        StateVariable::Qy(FrameId::World, FrameId::World),
        StateVariable::Qz(FrameId::World, FrameId::World),
        StateVariable::Qw(FrameId::World, FrameId::World),
    ];
    // FrameAwareState::new initializes Qw to 1.0 (identity quaternion).
    let mut state = FrameAwareState::new(layout, 1e-6, 0.0);
    state.vector[0] = px;
    state.vector[1] = py;
    state
}

/// Builds a Path whose waypoints have 2-element state vectors [x, y].
/// `advance_lookahead` uses `wp.state.vector[0]` and `wp.state.vector[1]` directly.
pub fn make_path(level_key: &str, waypoints: &[[f64; 2]]) -> Path {
    let layout = vec![
        StateVariable::Px(FrameId::World),
        StateVariable::Py(FrameId::World),
    ];
    Path {
        waypoints: waypoints
            .iter()
            .map(|&[x, y]| {
                let mut state = FrameAwareState::new(layout.clone(), 1e-6, 0.0);
                state.vector[0] = x;
                state.vector[1] = y;
                TrajectoryPoint {
                    state,
                    state_dot: None,
                    time: 0.0,
                }
            })
            .collect(),
        timestamp: 0.0,
        level_key: level_key.to_string(),
    }
}
