// helios_core/src/control/feedforward_pid.rs
//
// FeedforwardPidController: model-based feedforward + PID feedback.
// Injects ControlDynamics at construction; degrades to pure feedback when
// ctx.reference.state_dot is None.

use nalgebra::DVector;

use crate::frames::FrameAwareState;

use crate::models::controls::ControlDynamics;

use super::{siso_pid::SisoPid, ControlContext, ControlOutput, Controller};

/// Feedforward + feedback controller.
///
/// When `ctx.reference.state_dot` is `Some`, computes the feedforward term
/// `u_ff` such that `f(x_ref, u_ff) ≈ x_dot_ref` (Newton step inversion).
/// When `None`, logs a debug message and returns pure feedback.
///
/// Outputs `ControlOutput::Raw(u_ff + u_fb)`.
pub struct FeedforwardPidController {
    dynamics: Box<dyn ControlDynamics>,
    pids: Vec<SisoPid>,
    /// Indices of the state vector components each PID loop tracks.
    controlled_indices: Vec<usize>,
    u_min: DVector<f64>,
    u_max: DVector<f64>,
}

impl FeedforwardPidController {
    pub fn new(
        dynamics: Box<dyn ControlDynamics>,
        kp: Vec<f64>,
        ki: Vec<f64>,
        kd: Vec<f64>,
        u_min: Vec<f64>,
        u_max: Vec<f64>,
        controlled_indices: Vec<usize>,
    ) -> Result<Self, String> {
        let m = dynamics.get_control_dim();
        if kp.len() != m || ki.len() != m || kd.len() != m {
            return Err(format!(
                "FeedforwardPid: kp/ki/kd must each have {} elements (control_dim)",
                m
            ));
        }
        if u_min.len() != m || u_max.len() != m {
            return Err(format!(
                "FeedforwardPid: u_min/u_max must each have {} elements (control_dim)",
                m
            ));
        }

        let pids = (0..m)
            .map(|i| SisoPid::new(kp[i], ki[i], kd[i]))
            .collect();

        Ok(Self {
            dynamics,
            pids,
            controlled_indices,
            u_min: DVector::from_vec(u_min),
            u_max: DVector::from_vec(u_max),
        })
    }
}

impl Controller for FeedforwardPidController {
    fn compute(
        &mut self,
        state: &FrameAwareState,
        dt: f64,
        ctx: &ControlContext,
    ) -> ControlOutput {
        let m = self.dynamics.get_control_dim();

        // --- Feedforward ---
        let u_ff = if let Some(ref_pt) = ctx.reference {
            if let Some(x_dot_ref) = &ref_pt.state_dot {
                // Single Newton step to invert f(x_ref, u_ff) = x_dot_ref.
                // u_ff ≈ B_pinv * (x_dot_ref - A * x_ref) where A, B come from calculate_jacobian.
                let x_ref = &ref_pt.state.vector;
                let u0 = DVector::zeros(m);
                let (_, b) = self.dynamics.calculate_jacobian(x_ref, &u0, ref_pt.time);
                let b_pinv = b.clone().pseudo_inverse(1e-8).unwrap_or_else(|_| {
                    eprintln!(
                        "FeedforwardPid: pseudo-inverse failed, falling back to zero feedforward"
                    );
                    nalgebra::DMatrix::zeros(m, x_dot_ref.len())
                });
                let f0 = self.dynamics.get_derivatives(x_ref, &u0, ref_pt.time);
                let residual = x_dot_ref - f0;
                b_pinv * residual
            } else {
                eprintln!(
                    "FeedforwardPid: reference.state_dot is None; using pure feedback for this step"
                );
                DVector::zeros(m)
            }
        } else {
            DVector::zeros(m)
        };

        // --- Feedback ---
        let mut u_fb = DVector::zeros(m);
        for (i, pid) in self.pids.iter_mut().enumerate() {
            let state_idx = self.controlled_indices.get(i).copied().unwrap_or(i);
            let x_cur = state.vector.get(state_idx).copied().unwrap_or(0.0);
            let x_ref = ctx
                .reference
                .and_then(|r| r.state.vector.get(state_idx).copied())
                .unwrap_or(0.0);
            u_fb[i] = pid.update(x_ref - x_cur, dt);
        }

        // --- Combine and clamp ---
        let mut u = u_ff + u_fb;
        for i in 0..m {
            u[i] = u[i].clamp(self.u_min[i], self.u_max[i]);
        }

        ControlOutput::Raw(u)
    }

    fn reset(&mut self) {
        for pid in &mut self.pids {
            pid.reset();
        }
    }
}
