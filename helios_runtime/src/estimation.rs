//! Polymorphic estimation driver used by `AutonomyPipeline`.
//!
//! [`EstimationDriver`] abstracts over real filter execution (`EstimationCore`) and
//! simulation mock injection ([`GroundTruthPassthrough`]). Hardware code paths only ever
//! instantiate `EstimationCore`; `GroundTruthPassthrough` is simulation-only.

use helios_core::{frames::FrameAwareState, mapping::MapData, messages::MeasurementMessage};
use nalgebra::{Isometry3, Vector3};

use crate::runtime::AgentRuntime;

/// Polymorphic estimation driver used by [`AutonomyPipeline`](crate::pipeline::AutonomyPipeline).
///
/// Two implementations exist:
/// - `EstimationCore` — wraps a real [`StateEstimator`](helios_core::estimation::StateEstimator)
///   (EKF, UKF, etc.) and runs full filter math.
/// - [`GroundTruthPassthrough`] — injects physics ground-truth as a perfect state estimate.
///   Used by `MappingOnly`, `PlanningOnly`, and `ControlOnly` simulation profiles.
///
/// `helios_hw` will only ever instantiate `EstimationCore`. `GroundTruthPassthrough` is
/// simulation-only and must never appear in hardware code paths.
pub trait EstimationDriver: Send + Sync {
    /// Process one incoming sensor measurement through the estimation pipeline.
    ///
    /// For `EstimationCore`, this triggers a predict step followed by an update.
    /// For `GroundTruthPassthrough`, this is a no-op (ground truth is injected separately).
    fn process_measurement(&mut self, msg: &MeasurementMessage, runtime: &dyn AgentRuntime);

    /// Returns the current state estimate, if one is available.
    ///
    /// Returns `None` before the first measurement has been processed, or when
    /// ground truth has not yet been injected into a `GroundTruthPassthrough`.
    fn get_state(&self) -> Option<&FrameAwareState>;

    /// Returns the SLAM global map if a SLAM system is active. Default: `None`.
    fn get_slam_map(&self) -> Option<&MapData> {
        None
    }

    /// Injects a physics ground-truth pose and velocity as the current state estimate.
    ///
    /// Default is a no-op; overridden by [`GroundTruthPassthrough`] for mock profiles.
    /// Never called in `EstimationCore` — real filters use sensor measurements only.
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
        state.set_variable(&StateVariable::Px(FrameId::World), pose.translation.x);
        state.set_variable(&StateVariable::Py(FrameId::World), pose.translation.y);
        state.set_variable(&StateVariable::Pz(FrameId::World), pose.translation.z);
        state.set_variable(&StateVariable::Vx(FrameId::World), velocity.x);
        state.set_variable(&StateVariable::Vy(FrameId::World), velocity.y);
        state.set_variable(&StateVariable::Vz(FrameId::World), velocity.z);
        let q = pose.rotation.quaternion();
        state.set_variable(&StateVariable::Qx(FrameId::World, FrameId::World), q.i);
        state.set_variable(&StateVariable::Qy(FrameId::World, FrameId::World), q.j);
        state.set_variable(&StateVariable::Qz(FrameId::World, FrameId::World), q.k);
        state.set_variable(&StateVariable::Qw(FrameId::World, FrameId::World), q.w);
        self.state = Some(state);
    }
}
