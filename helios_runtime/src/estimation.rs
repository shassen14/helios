// helios_runtime/src/estimation.rs
//
// EstimationDriver trait and GroundTruthPassthrough implementation.

use helios_core::{frames::FrameAwareState, mapping::MapData, messages::MeasurementMessage};
use nalgebra::{Isometry3, Vector3};

use crate::runtime::AgentRuntime;

/// Polymorphic estimation driver. Implemented by both `EstimationCore` (real filter)
/// and `GroundTruthPassthrough` (sim-only mock).
pub trait EstimationDriver: Send + Sync {
    fn process_measurement(&mut self, msg: &MeasurementMessage, runtime: &dyn AgentRuntime);
    fn get_state(&self) -> Option<&FrameAwareState>;
    fn get_slam_map(&self) -> Option<&MapData> {
        None
    }
    /// Default no-op. Overridden by `GroundTruthPassthrough`.
    fn inject_ground_truth(
        &mut self,
        _pose: &Isometry3<f64>,
        _velocity: Vector3<f64>,
        _timestamp: f64,
    ) {
    }
}

/// Ground-truth passthrough for mock-estimator profiles.
/// Stores an injected state and returns it from `get_state()`.
/// Never runs a real filter — used when physics provides perfect pose.
#[derive(Default)]
pub struct GroundTruthPassthrough {
    state: Option<FrameAwareState>,
}

impl EstimationDriver for GroundTruthPassthrough {
    fn process_measurement(&mut self, _: &MeasurementMessage, _: &dyn AgentRuntime) {}

    fn get_state(&self) -> Option<&FrameAwareState> {
        self.state.as_ref()
    }

    fn inject_ground_truth(
        &mut self,
        pose: &Isometry3<f64>,
        velocity: Vector3<f64>,
        timestamp: f64,
    ) {
        use helios_core::frames::{FrameId, StateVariable};

        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Pz(FrameId::World),
            StateVariable::Vx(FrameId::World),
            StateVariable::Vy(FrameId::World),
            StateVariable::Vz(FrameId::World),
            StateVariable::Qx(FrameId::World, FrameId::World),
            StateVariable::Qy(FrameId::World, FrameId::World),
            StateVariable::Qz(FrameId::World, FrameId::World),
            StateVariable::Qw(FrameId::World, FrameId::World),
        ];
        let mut state = FrameAwareState::new(layout, 1e-6, timestamp);
        state.vector[0] = pose.translation.x;
        state.vector[1] = pose.translation.y;
        state.vector[2] = pose.translation.z;
        state.vector[3] = velocity.x;
        state.vector[4] = velocity.y;
        state.vector[5] = velocity.z;
        let q = pose.rotation.quaternion();
        state.vector[6] = q.i;
        state.vector[7] = q.j;
        state.vector[8] = q.k;
        state.vector[9] = q.w;
        self.state = Some(state);
    }
}
