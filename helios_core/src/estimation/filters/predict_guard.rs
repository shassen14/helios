//! The preconditions every Gaussian filter checks before it propagates, shared
//! so the EKF and UKF skip on the same inputs for the same reasons.

use crate::estimation::dynamics::EstimationDynamics;
use crate::estimation::PredictSkipReason;

use nalgebra::DVector;

/// Checks that a predict over `dt` with control `u` may call
/// [`EstimationDynamics::propagate`]: `dt > 0`, and `u` is exactly as long as the
/// dynamics' input schema. Returns the reason to skip otherwise.
///
/// The step is checked first: on a zero-length step nothing would be integrated,
/// so the input's shape does not matter yet. A length mismatch is reported, never
/// padded or truncated: a zero-filled input would propagate as if the body felt
/// no force and no rotation, which is a silent model error, not a safe default.
pub(crate) fn check_predict(
    dynamics: &dyn EstimationDynamics,
    dt: f64,
    u: &DVector<f64>,
) -> Result<(), PredictSkipReason> {
    if dt <= 0.0 {
        return Err(PredictSkipReason::NonPositiveDt);
    }
    let expected = dynamics.input_schema().dim();
    if u.nrows() != expected {
        return Err(PredictSkipReason::InputShapeMismatch {
            expected,
            supplied: u.nrows(),
        });
    }
    Ok(())
}
