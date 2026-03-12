// helios_core/src/control/lqr.rs
//
// LQR controller with a precomputed gain matrix K.
// u = -K * (x - x_ref). No dynamics model at runtime.

use nalgebra::{DMatrix, DVector};

use crate::frames::FrameAwareState;

use super::{ControlContext, ControlOutput, Controller};

/// Linear-Quadratic Regulator with a precomputed gain matrix K.
///
/// K is loaded from config at construction time (offline computation).
/// At runtime: `u = clamp(-K * (x - x_ref), u_min, u_max)`.
///
/// Outputs `ControlOutput::Raw(u)`. The vehicle actuator plugin maps `u`
/// to forces/torques.
pub struct LqrController {
    /// Gain matrix: shape (control_dim × state_dim).
    k: DMatrix<f64>,
    state_dim: usize,
    control_dim: usize,
    /// Per-channel lower bounds on u (length = control_dim).
    u_min: DVector<f64>,
    /// Per-channel upper bounds on u (length = control_dim).
    u_max: DVector<f64>,
}

impl LqrController {
    /// Construct from a flat, row-major K matrix.
    pub fn new(
        gain_matrix: Vec<f64>,
        state_dim: usize,
        control_dim: usize,
        u_min: Vec<f64>,
        u_max: Vec<f64>,
    ) -> Result<Self, String> {
        if gain_matrix.len() != control_dim * state_dim {
            return Err(format!(
                "LQR gain_matrix has {} elements but control_dim × state_dim = {} × {} = {}",
                gain_matrix.len(),
                control_dim,
                state_dim,
                control_dim * state_dim
            ));
        }
        if u_min.len() != control_dim || u_max.len() != control_dim {
            return Err(format!(
                "LQR u_min/u_max must each have {} elements (control_dim)",
                control_dim
            ));
        }
        let k = DMatrix::from_row_slice(control_dim, state_dim, &gain_matrix);
        Ok(Self {
            k,
            state_dim,
            control_dim,
            u_min: DVector::from_vec(u_min),
            u_max: DVector::from_vec(u_max),
        })
    }
}

impl Controller for LqrController {
    fn compute(
        &mut self,
        state: &FrameAwareState,
        _dt: f64,
        ctx: &ControlContext,
    ) -> ControlOutput {
        let x = &state.vector;

        // Build reference vector from TrajectoryPoint if available.
        let x_ref = ctx
            .reference
            .map(|r| r.state.vector.clone())
            .unwrap_or_else(|| DVector::zeros(self.state_dim));

        // Pad or truncate to match expected state dimension.
        let dx: DVector<f64> = if x.len() >= self.state_dim {
            x.rows(0, self.state_dim) - x_ref.rows(0, self.state_dim.min(x_ref.len()))
        } else {
            let mut padded = DVector::zeros(self.state_dim);
            for i in 0..x.len() {
                padded[i] = x[i] - if i < x_ref.len() { x_ref[i] } else { 0.0 };
            }
            padded
        };

        let mut u = -&self.k * dx;

        // Per-channel clamping.
        for i in 0..self.control_dim {
            u[i] = u[i].clamp(self.u_min[i], self.u_max[i]);
        }

        ControlOutput::Raw(u)
    }

    fn reset(&mut self) {
        // LQR is memoryless; nothing to reset.
    }
}
