// helios_runtime/src/pipeline/estimation_core.rs
//
// EstimationCore struct + impl + EstimationDriver trait impl.

use helios_core::{
    estimation::{EstimatorInputs, FilterContext, StateEstimator},
    frames::FrameAwareState,
    messages::MeasurementMessage,
    tracking::Tracker,
    types::Control,
};

use crate::{
    estimation::EstimationDriver,
    runtime::{AgentRuntime, TfProviderAdapter},
};

/// Estimation stage: trackers + estimator + shared control input state.
/// Owns all logic for predict + update sequencing.
pub struct EstimationCore {
    pub trackers: Vec<Box<dyn Tracker>>,
    pub estimator: Option<Box<dyn StateEstimator>>,
    pub(crate) last_u: Control,
}

impl EstimationCore {
    /// Process one incoming sensor measurement through trackers + estimator.
    /// Called once per measurement event.
    pub fn process_measurement(&mut self, msg: &MeasurementMessage, runtime: &dyn AgentRuntime) {
        let adapter = TfProviderAdapter(runtime);
        let context = FilterContext { tf: Some(&adapter) };

        for tracker in &mut self.trackers {
            tracker.update(msg, &context);
        }

        if let Some(estimator) = &mut self.estimator {
            let dt = msg.timestamp - estimator.get_state().state.timestamp;
            if dt > 1e-9 {
                estimator.predict(
                    dt,
                    &EstimatorInputs {
                        control: self.last_u.clone(),
                    },
                );
            }
            let dynamics = estimator.get_dynamics_model();
            if let Some(new_u) = dynamics.get_control_from_measurement(&msg.data) {
                self.last_u = new_u;
            } else {
                estimator.update(msg, &context);
            }
        }
    }

    /// Current best estimate of the ego state.
    pub fn get_state(&self) -> Option<&FrameAwareState> {
        self.estimator.as_ref().map(|e| e.get_state())
    }
}

impl EstimationDriver for EstimationCore {
    fn process_measurement(&mut self, msg: &MeasurementMessage, runtime: &dyn AgentRuntime) {
        self.process_measurement(msg, runtime);
    }
    fn get_state(&self) -> Option<&FrameAwareState> {
        self.get_state()
    }
    // inject_ground_truth: default no-op — real estimator ignores GT injection
}
