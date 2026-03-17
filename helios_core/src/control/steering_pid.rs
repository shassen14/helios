// helios_core/src/control/steering_pid.rs
//
// SteeringPidController: heading-error path follower for Ackermann vehicles.
// Outputs BodyVelocity { vx = cruise_speed, yaw_rate = pid(heading_error) }.

use nalgebra::Vector3;

use super::{siso_pid::SisoPid, ControlContext, ControlOutput, Controller};
use crate::frames::{FrameAwareState, FrameId, StateVariable};

/// Heading-error path follower for non-holonomic (Ackermann) vehicles.
///
/// Each tick it:
/// 1. Extracts current world-frame position and heading from the EKF state.
/// 2. Reads the bearing to the active lookahead waypoint (Px/Py in world frame).
/// 3. Applies a PID on the heading error to produce a yaw-rate command.
/// 4. Outputs `BodyVelocity { vx = cruise_speed, yaw_rate }`.
///
/// Compatible with `DualSisoPidAdapter` on the Ackermann vehicle plugin.
pub struct SteeringPidController {
    heading_pid: SisoPid,
    cruise_speed: f64,
    tick_count: u64,
}

impl SteeringPidController {
    pub fn new(kp: f64, ki: f64, kd: f64, cruise_speed: f64) -> Self {
        Self {
            heading_pid: SisoPid::new(kp, ki, kd),
            cruise_speed,
            tick_count: 0,
        }
    }
}

fn normalize_angle(a: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let a = ((a % two_pi) + two_pi) % two_pi;
    if a > std::f64::consts::PI {
        a - two_pi
    } else {
        a
    }
}

fn world_xy(state: &FrameAwareState) -> Option<(f64, f64)> {
    let mut px = None;
    let mut py = None;
    for (i, var) in state.layout.iter().enumerate() {
        match var {
            StateVariable::Px(FrameId::World) => px = Some(state.vector[i]),
            StateVariable::Py(FrameId::World) => py = Some(state.vector[i]),
            _ => {}
        }
    }
    px.zip(py)
}

impl Controller for SteeringPidController {
    fn compute(&mut self, state: &FrameAwareState, dt: f64, ctx: &ControlContext) -> ControlOutput {
        self.tick_count += 1;
        let log = self.tick_count % 60 == 1;

        let zero = ControlOutput::BodyVelocity {
            linear: Vector3::zeros(),
            angular: Vector3::zeros(),
        };

        let reference = match ctx.reference {
            Some(r) => r,
            None => {
                if log {
                    eprintln!(
                        "[SteeringPid #{tick}] no reference waypoint — idling",
                        tick = self.tick_count
                    );
                }
                self.heading_pid.reset();
                return zero;
            }
        };

        let (cx, cy) = match world_xy(state) {
            Some(p) => p,
            None => {
                if log {
                    eprintln!(
                        "[SteeringPid #{tick}] EKF state has no Px/Py(World)",
                        tick = self.tick_count
                    );
                }
                return zero;
            }
        };
        let (rx, ry) = match world_xy(&reference.state) {
            Some(p) => p,
            None => {
                if log {
                    eprintln!(
                        "[SteeringPid #{tick}] reference state has no Px/Py(World)",
                        tick = self.tick_count
                    );
                }
                return zero;
            }
        };
        let orientation = match state.get_orientation() {
            Some(q) => q,
            None => {
                if log {
                    eprintln!(
                        "[SteeringPid #{tick}] EKF state has no orientation quaternion",
                        tick = self.tick_count
                    );
                }
                return zero;
            }
        };
        let (roll, pitch, current_yaw) = orientation.euler_angles();

        // ENU: east = +X, north = +Y → atan2(Δnorth, Δeast)
        let desired_yaw = (ry - cy).atan2(rx - cx);
        let heading_error = normalize_angle(desired_yaw - current_yaw);
        let yaw_rate = self.heading_pid.update(heading_error, dt);

        if log {
            let q = orientation.quaternion();
            eprintln!(
                "[SteeringPid #{tick}]\n  \
                 ekf_pos_enu   = ({cx:.2}, {cy:.2})\n  \
                 ref_pos_enu   = ({rx:.2}, {ry:.2})\n  \
                 ekf_quat      = (i={qi:.4}, j={qj:.4}, k={qk:.4}, w={qw:.4})\n  \
                 ekf_rpy_deg   = ({roll:.1}°, {pitch:.1}°, {yaw:.1}°)\n  \
                 desired_yaw   = {des:.1}°\n  \
                 heading_error = {err:.1}°\n  \
                 yaw_rate_cmd  = {yr:.4} rad/s",
                tick = self.tick_count,
                cx = cx,
                cy = cy,
                rx = rx,
                ry = ry,
                qi = q.i,
                qj = q.j,
                qk = q.k,
                qw = q.w,
                roll = roll.to_degrees(),
                pitch = pitch.to_degrees(),
                yaw = current_yaw.to_degrees(),
                des = desired_yaw.to_degrees(),
                err = heading_error.to_degrees(),
                yr = yaw_rate,
            );
        }

        ControlOutput::BodyVelocity {
            linear: Vector3::new(self.cruise_speed, 0.0, 0.0),
            angular: Vector3::new(0.0, 0.0, yaw_rate),
        }
    }

    fn reset(&mut self) {
        self.heading_pid.reset();
    }
}
