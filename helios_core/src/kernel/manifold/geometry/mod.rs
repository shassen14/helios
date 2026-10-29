//! The kind of space a run of stored numbers lives in, and the three
//! operations every consumer needs on it: step, difference, and in-between.
//!
//! A value of one kind is `storage_dim` numbers; a step on it (a difference,
//! a velocity, an error) is `tangent_dim` numbers. The two differ only for a
//! rotation, which stores 4 and moves in 3.
//!
//! - `oplus(x, δ)` (⊞): the point `x` moved by the step `δ`.
//! - `ominus(y, x)` (⊟): the step that carries `x` to `y`, the short way.
//! - `interpolate(a, b, t)`: `a ⊞ (t · (b ⊟ a))`, the point a fraction `t` of
//!   the way from `a` to `b`, exactly `a` at 0 and exactly `b` at 1.
//!
//! Every operation reads slices and writes into the caller's `out`, so none
//! allocates: callers hold views into one flat buffer and call these in loops.
//!
//! **Lengths and the fraction are the caller's guarantee.** Whoever slices
//! storage by a kind knows its widths, so a mismatched slice is a bug in that
//! caller, not an input to refuse here. The checks below are `debug_assert`s:
//! a tripwire for that bug in tests, never the guard in a release build.
//! Values are the writer's guarantee too: a NaN, or a stored rotation off unit
//! length, is refused where it is written (a stored rotation is still read as
//! the nearest one).

use crate::kernel::{angle, manifold::rotation};

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::num::NonZeroUsize;

/// The space a stored value lives in. Serialized internally tagged:
/// `{ kind = "torus", n = 1 }`, `{ kind = "rotation3" }`.
///
/// A rate is flat whatever it is the rate of: angular velocity is
/// `Euclidean { n: 3 }`, a heading rate `Euclidean { n: 1 }`, never the kind
/// of the value they move.
///
/// Callers use the methods and never match on the kind themselves, so a new
/// kind is added here and nowhere else. The derived `Hash` is for in-memory
/// maps only: its output is not stable across Rust releases, so an identity
/// that is stored or sent hashes the serialized form instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum GeometryKind {
    /// `n` flat numbers: a position, a velocity, a joint that does not wrap.
    /// Steps add and subtract.
    Euclidean { n: NonZeroUsize },
    /// `n` angles that each wrap on their own: a heading, revolute joints.
    /// Each component is kept in (−π, π] and differs from another the short
    /// way round. Not a 3D attitude: roll, pitch and yaw do not wrap
    /// independently, so an orientation is [`Rotation3`](Self::Rotation3).
    Torus { n: NonZeroUsize },
    /// One body's orientation in 3D, stored as a unit quaternion `[x, y, z, w]`
    /// and stepped on the body side; see [`rotation`].
    Rotation3,
}

impl GeometryKind {
    /// Numbers one value occupies in storage.
    pub fn storage_dim(&self) -> usize {
        match self {
            Self::Euclidean { n } | Self::Torus { n } => n.get(),
            Self::Rotation3 => rotation::STORAGE_DIM,
        }
    }

    /// Numbers one step occupies: the value's degrees of freedom.
    pub fn tangent_dim(&self) -> usize {
        match self {
            Self::Euclidean { n } | Self::Torus { n } => n.get(),
            Self::Rotation3 => rotation::TANGENT_DIM,
        }
    }

    /// Writes `x ⊞ delta` into `out`. `x` and `out` are `storage_dim` long,
    /// `delta` is `tangent_dim` long.
    pub fn oplus(&self, x: &[f64], delta: &[f64], out: &mut [f64]) {
        // Caller-guaranteed; see the module doc.
        debug_assert_eq!(x.len(), self.storage_dim());
        debug_assert_eq!(delta.len(), self.tangent_dim());
        debug_assert_eq!(out.len(), self.storage_dim());

        match self {
            Self::Euclidean { .. } => {
                for ((out_i, x_i), delta_i) in out.iter_mut().zip(x).zip(delta) {
                    *out_i = x_i + delta_i;
                }
            }
            Self::Torus { .. } => {
                for ((out_i, x_i), delta_i) in out.iter_mut().zip(x).zip(delta) {
                    *out_i = angle::wrap(x_i + delta_i);
                }
            }
            Self::Rotation3 => {
                let q_x = rotation::from_storage(x);
                let step = Vector3::from_column_slice(delta);

                let q_out = rotation::oplus(&q_x, &step);

                out.copy_from_slice(&rotation::to_storage(&q_out));
            }
        }
    }

    /// Writes `y ⊟ x`, the step from `x` to `y`, into `out`, so that
    /// `oplus(x, out)` lands on `y`. `y` and `x` are `storage_dim` long, `out`
    /// is `tangent_dim` long. A torus component or a rotation takes the short
    /// way: at most π. Exactly a half turn apart, a torus component steps +π;
    /// a rotation steps π about an axis chosen by the log map, deterministic
    /// on one platform but not promised beyond it.
    pub fn ominus(&self, y: &[f64], x: &[f64], out: &mut [f64]) {
        // Caller-guaranteed; see the module doc.
        debug_assert_eq!(y.len(), self.storage_dim());
        debug_assert_eq!(x.len(), self.storage_dim());
        debug_assert_eq!(out.len(), self.tangent_dim());

        match self {
            Self::Euclidean { .. } => {
                for ((out_i, y_i), x_i) in out.iter_mut().zip(y).zip(x) {
                    *out_i = y_i - x_i;
                }
            }
            Self::Torus { .. } => {
                for ((out_i, y_i), x_i) in out.iter_mut().zip(y).zip(x) {
                    *out_i = angle::difference(*x_i, *y_i);
                }
            }
            Self::Rotation3 => {
                let q_x = rotation::from_storage(x);
                let q_y = rotation::from_storage(y);

                let step = rotation::ominus(&q_y, &q_x);

                out.copy_from_slice(step.as_slice());
            }
        }
    }

    /// Writes the point a fraction `t` ∈ [0, 1] of the way from `a` to `b`
    /// into `out`: `a ⊞ (t · (b ⊟ a))`, along the short way. All three slices
    /// are `storage_dim` long.
    ///
    /// At `t` = 0 and 1 it copies `a` or `b` bit for bit rather than computing
    /// them, so a stored value read back at its own stamp is the value written.
    /// Each arm writes the rule out per component instead of calling `ominus`
    /// and `oplus`, which would need a step buffer allocated per call.
    pub fn interpolate(&self, a: &[f64], b: &[f64], t: f64, out: &mut [f64]) {
        // Caller-guaranteed; see the module doc.
        debug_assert_eq!(a.len(), self.storage_dim());
        debug_assert_eq!(b.len(), self.storage_dim());
        debug_assert_eq!(out.len(), self.storage_dim());
        debug_assert!((0.0..=1.0).contains(&t));

        if t == 0.0 {
            out.copy_from_slice(a);
            return;
        }
        if t == 1.0 {
            out.copy_from_slice(b);
            return;
        }

        match self {
            Self::Euclidean { .. } => {
                for ((out_i, a_i), b_i) in out.iter_mut().zip(a).zip(b) {
                    *out_i = a_i + t * (b_i - a_i);
                }
            }
            Self::Torus { .. } => {
                for ((out_i, a_i), b_i) in out.iter_mut().zip(a).zip(b) {
                    *out_i = angle::wrap(a_i + t * angle::difference(*a_i, *b_i));
                }
            }
            Self::Rotation3 => {
                let q_a = rotation::from_storage(a);
                let q_b = rotation::from_storage(b);

                let step = rotation::ominus(&q_b, &q_a);
                let q_out = rotation::oplus(&q_a, &(step * t));

                out.copy_from_slice(&rotation::to_storage(&q_out));
            }
        }
    }
}

#[cfg(test)]
mod tests;
