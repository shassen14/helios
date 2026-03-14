// helios_runtime/src/pipeline/estimation_core.rs
//
// EstimationCore struct + impl + EstimationDriver trait impl.

use helios_core::{
    estimation::{FilterContext, StateEstimator},
    frames::FrameAwareState,
    mapping::MapData,
    messages::MeasurementMessage,
    slam::SlamSystem,
    tracking::Tracker,
    types::Control,
};

use crate::{
    estimation::EstimationDriver,
    runtime::{AgentRuntime, TfProviderAdapter},
};

/// Estimation stage: trackers + estimator/SLAM + shared control input state.
/// Owns all logic for predict + update sequencing.
pub struct EstimationCore {
    pub trackers: Vec<Box<dyn Tracker>>,
    pub estimator: Option<Box<dyn StateEstimator>>,
    pub slam: Option<Box<dyn SlamSystem>>,
    pub(crate) last_u: Control,
    #[allow(dead_code)]
    pub(crate) control_dim: usize,
}

impl EstimationCore {
    /// Process one incoming sensor measurement through trackers + estimator/SLAM.
    /// Called once per measurement event.
    pub fn process_measurement(&mut self, msg: &MeasurementMessage, runtime: &dyn AgentRuntime) {
        let adapter = TfProviderAdapter(runtime);
        let context = FilterContext { tf: Some(&adapter) };

        for tracker in &mut self.trackers {
            tracker.update(msg, &context);
        }

        if let Some(estimator) = &mut self.estimator {
            let dt = msg.timestamp - estimator.get_state().last_update_timestamp;
            if dt > 1e-9 {
                estimator.predict(dt, &self.last_u, &context);
            }
            let dynamics = estimator.get_dynamics_model();
            if let Some(new_u) = dynamics.get_control_from_measurement(&msg.data) {
                self.last_u = new_u;
            } else {
                estimator.update(msg, &context);
            }
        } else if let Some(slam) = &mut self.slam {
            let dt = msg.timestamp - slam.get_state().last_update_timestamp;
            if dt > 1e-9 {
                slam.predict(dt, &self.last_u, &context);
            }
            let dynamics = slam.get_dynamics_model();
            if let Some(new_u) = dynamics.get_control_from_measurement(&msg.data) {
                self.last_u = new_u;
            } else {
                slam.update(msg, &context);
            }
        }
    }

    /// Current best estimate of the ego state (from estimator or SLAM).
    pub fn get_state(&self) -> Option<&FrameAwareState> {
        if let Some(est) = &self.estimator {
            Some(est.get_state())
        } else {
            self.slam.as_ref().map(|s| s.get_state())
        }
    }

    /// SLAM global map, if a SLAM system is active.
    pub fn get_slam_map(&self) -> Option<&MapData> {
        self.slam.as_ref().map(|s| s.get_map())
    }
}

impl EstimationDriver for EstimationCore {
    fn process_measurement(&mut self, msg: &MeasurementMessage, runtime: &dyn AgentRuntime) {
        self.process_measurement(msg, runtime);
    }
    fn get_state(&self) -> Option<&FrameAwareState> {
        self.get_state()
    }
    fn get_slam_map(&self) -> Option<&MapData> {
        self.get_slam_map()
    }
    // inject_ground_truth: default no-op — real estimator ignores GT injection
}
