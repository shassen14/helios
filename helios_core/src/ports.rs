use crate::data::primitives::FrameHandle;

use nalgebra::Isometry3;

/// Abstraction over any system that can answer transform queries between coordinate frames.
///
/// `helios_sim` implements this as `TfTree` (Bevy resource). `helios_hw` will implement it
/// as a hardware-clock-backed calibration tree. Filters receive `&dyn TfProvider` via
/// `FilterContext` — they never depend on the concrete host type.
pub trait TfProvider {
    fn get_transform(&self, from: FrameHandle, to: FrameHandle) -> Option<Isometry3<f64>>;

    /// Returns the world (ENU) pose of a frame directly from physics.
    /// Use this to bypass the estimator entirely (e.g. ground-truth mapping).
    fn world_pose(&self, frame: FrameHandle) -> Option<Isometry3<f64>>;
}
