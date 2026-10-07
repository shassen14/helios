//! The innovation diagnostic every Gaussian filter reports on an applied update,
//! computed one way so the EKF and UKF report the same number for the same
//! `y` and `S`.

use crate::estimation::Innovation;

use nalgebra::{Cholesky, DVector, Dyn};

/// Measures innovation `y` against its predicted covariance `S`, given `S`'s
/// Cholesky factor: `nis = yᵀS⁻¹y` over `dof = y.nrows()`.
///
/// Solves with the factor the filter already holds for its gain rather than
/// forming `S⁻¹`. Reads only; the estimate is untouched, so computing it changes
/// no filter output.
pub(crate) fn measure_innovation(s_chol: &Cholesky<f64, Dyn>, y: &DVector<f64>) -> Innovation {
    let nis = y.dot(&s_chol.solve(y));
    Innovation::new(nis, y.nrows())
}
