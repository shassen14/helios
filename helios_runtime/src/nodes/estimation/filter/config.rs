//! [`EkfConfig`] and [`UkfConfig`] — each filter kind's own keys in a
//! `filter` sub-table.

use helios_core::estimation::filters::ekf::CovarianceConditioning;
use helios_core::estimation::filters::ukf;

use serde::{Deserialize, Serialize};

/// The EKF's own keys in a `filter` sub-table.
///
/// The EKF always symmetrises `P` after a step. The floor and the jitter are
/// off unless set: each hides a diverging or over-confident filter rather
/// than fixing it.
#[derive(Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct EkfConfig {
    /// The smallest variance `P` may hold on its diagonal; a smaller one is
    /// raised to it after each step. Must be finite and positive.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) covariance_floor: Option<f64>,
    /// Added to every variance after each step. Must be finite and positive.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) covariance_jitter: Option<f64>,
}

impl EkfConfig {
    /// The conditioning these keys ask for, or why a value is unusable.
    pub(super) fn conditioning(&self) -> Result<CovarianceConditioning, String> {
        for (key, value) in [
            ("covariance_floor", self.covariance_floor),
            ("covariance_jitter", self.covariance_jitter),
        ] {
            if let Some(value) = value {
                if !value.is_finite() || value <= 0.0 {
                    return Err(format!("{key} must be finite and positive, got {value}"));
                }
            }
        }
        Ok(CovarianceConditioning {
            floor: self.covariance_floor,
            jitter: self.covariance_jitter,
        })
    }
}

/// The UKF's own keys in a `filter` sub-table: the sigma-point spread.
/// All three are required; there is no one right default.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct UkfConfig {
    /// How far the sigma points sit from the mean. Must be finite and
    /// positive.
    pub(crate) alpha: f64,
    /// Prior knowledge of the distribution's shape (2 for a Gaussian). Must
    /// be finite.
    pub(crate) beta: f64,
    /// Secondary spread. Must be finite, and `n + kappa` positive for the
    /// state's `n` degrees of freedom, or the sigma points have no spread.
    pub(crate) kappa: f64,
}

impl UkfConfig {
    /// The core parameters for a state with `dof` degrees of freedom, or why
    /// a value is unusable.
    pub(super) fn for_dof(&self, dof: usize) -> Result<ukf::UkfParams, String> {
        if !self.alpha.is_finite() || self.alpha <= 0.0 {
            return Err(format!(
                "alpha must be finite and positive, got {}",
                self.alpha
            ));
        }
        for (key, value) in [("beta", self.beta), ("kappa", self.kappa)] {
            if !value.is_finite() {
                return Err(format!("{key} must be finite, got {value}"));
            }
        }
        if dof as f64 + self.kappa <= 0.0 {
            return Err(format!(
                "kappa must exceed -{dof}, the state's degrees of freedom, got {}",
                self.kappa
            ));
        }
        Ok(ukf::UkfParams {
            alpha: self.alpha,
            beta: self.beta,
            kappa: self.kappa,
        })
    }
}
