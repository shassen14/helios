// helios_core/src/control/pid.rs
//
// VelocityPidController: three SisoPid loops (vx, vy, yaw_rate) for holonomic or
// near-holonomic robots. Outputs BodyVelocity.

use nalgebra::Vector3;

use crate::frames::{FrameAwareState, FrameId, StateVariable};

use super::{
    siso_pid::SisoPid, ControlContext, ControlOutput, Controller, TrajectoryPoint,
};

/// Pure feedback velocity controller composed of three independent SISO PID loops.
///
/// Each loop tracks one body-frame velocity component:
///   - vx (forward), vy (lateral), yaw_rate
///
/// The output is `ControlOutput::BodyVelocity`. The actuator plugin is
/// responsible for converting this to forces/torques via the vehicle model.
pub struct VelocityPidController {
    vx_pid: SisoPid,
    vy_pid: SisoPid,
    yaw_pid: SisoPid,
}

impl VelocityPidController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            vx_pid: SisoPid::new(kp, ki, kd),
            vy_pid: SisoPid::new(kp, ki, kd),
            yaw_pid: SisoPid::new(kp, ki, kd),
        }
    }

    /// Construct with separate gains per channel.
    pub fn with_gains(
        vx: (f64, f64, f64),
        vy: (f64, f64, f64),
        yaw: (f64, f64, f64),
    ) -> Self {
        Self {
            vx_pid: SisoPid::new(vx.0, vx.1, vx.2),
            vy_pid: SisoPid::new(vy.0, vy.1, vy.2),
            yaw_pid: SisoPid::new(yaw.0, yaw.1, yaw.2),
        }
    }
}

/// Extract body-frame vx/vy/yaw_rate from a FrameAwareState by scanning the layout.
fn extract_body_velocities(state: &FrameAwareState) -> (f64, f64, f64) {
    let mut vx = 0.0f64;
    let mut vy = 0.0f64;
    let mut yaw_rate = 0.0f64;

    for (i, var) in state.layout.iter().enumerate() {
        match var {
            StateVariable::Vx(FrameId::Body(_)) | StateVariable::Vx(FrameId::World) => {
                vx = state.vector[i];
            }
            StateVariable::Vy(FrameId::Body(_)) | StateVariable::Vy(FrameId::World) => {
                vy = state.vector[i];
            }
            StateVariable::Wz(FrameId::Body(_)) | StateVariable::Wz(FrameId::World) => {
                yaw_rate = state.vector[i];
            }
            _ => {}
        }
    }
    (vx, vy, yaw_rate)
}

fn extract_ref_velocities(reference: &TrajectoryPoint) -> (f64, f64, f64) {
    extract_body_velocities(&reference.state)
}

impl Controller for VelocityPidController {
    fn compute(
        &mut self,
        state: &FrameAwareState,
        dt: f64,
        ctx: &ControlContext,
    ) -> ControlOutput {
        let (cur_vx, cur_vy, cur_yaw) = extract_body_velocities(state);

        let (ref_vx, ref_vy, ref_yaw) = ctx
            .reference
            .map(extract_ref_velocities)
            .unwrap_or((0.0, 0.0, 0.0));

        let out_vx = self.vx_pid.update(ref_vx - cur_vx, dt);
        let out_vy = self.vy_pid.update(ref_vy - cur_vy, dt);
        let out_yaw = self.yaw_pid.update(ref_yaw - cur_yaw, dt);

        ControlOutput::BodyVelocity {
            linear: Vector3::new(out_vx, out_vy, 0.0),
            angular: Vector3::new(0.0, 0.0, out_yaw),
        }
    }

    fn reset(&mut self) {
        self.vx_pid.reset();
        self.vy_pid.reset();
        self.yaw_pid.reset();
    }
}
