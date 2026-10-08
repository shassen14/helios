//! The finiteness check every Gaussian filter runs before it corrects, shared
//! so the EKF and UKF skip on the same inputs for the same reason.

use crate::estimation::SkipReason;
use crate::spatial::FrameAwareState;

use nalgebra::{DMatrix, DVector};

/// Checks that measurement `z`, its noise `r` and the estimate `state` are all
/// finite. Returns [`SkipReason::NonFiniteInput`] otherwise.
///
/// Run after the shape check and before the model predicts: a NaN in any of
/// them reaches the gain and the mean, and nothing after the update can clear it.
pub(crate) fn check_update_finite(
    z: &DVector<f64>,
    r: &DMatrix<f64>,
    state: &FrameAwareState,
) -> Result<(), SkipReason> {
    let finite =
        z.iter().all(|v| v.is_finite()) && r.iter().all(|v| v.is_finite()) && state.is_finite();
    if finite {
        Ok(())
    } else {
        Err(SkipReason::NonFiniteInput)
    }
}
