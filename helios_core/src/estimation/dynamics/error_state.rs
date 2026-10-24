//! The continuous error-state model a dynamics model may supply in place of
//! the filter's numeric linearisation.

use nalgebra::DMatrix;

/// The continuous-time error-state model `δẋ = F_c·δx + G_c·w`, where `w` is
/// white noise with spectral density `Q_c`, all in tangent coordinates.
///
/// Sized to the dynamics model's own (base) schema: `n` is its tangent
/// dimension and `m` the number of noise channels. A filter carrying
/// augmentation blocks embeds it into the larger state; the model never sizes
/// for them.
///
/// The rows follow the schema's perturbation convention, not the textbook one
/// a derivation happens to use. The orientation block retracts by
/// right-multiplication (`q ⊗ exp(δθ)`, an error in the body frame), so an INS
/// rotation-error row is the body-frame `−[ω]×`, not the world-frame form. A
/// matrix in the other convention is the right size and plausible-looking, and
/// silently wrong.
#[derive(Debug, Clone, PartialEq)]
pub struct ErrorStateModel {
    /// `F_c`, `n × n`: how the tangent error evolves on its own.
    pub f_c: DMatrix<f64>,
    /// `G_c`, `n × m`: how each noise channel enters the tangent error.
    pub g_c: DMatrix<f64>,
    /// `Q_c`, `m × m`: the noise's continuous spectral density, per second.
    pub q_c: DMatrix<f64>,
}
