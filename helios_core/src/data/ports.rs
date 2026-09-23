use crate::{
    prelude::MonotonicTime,
    frames::{transforms::ErasedTransform, FrameId},
};

/// Abstraction over any system that can answer transform queries between coordinate frames.
///
/// `helios_sim` implements this as `TfTree` (Bevy resource). `helios_hw` will implement it
/// as a hardware-clock-backed calibration tree. Filters receive `&dyn TfProvider` via
/// `FilterContext` — they never depend on the concrete host type.
pub trait TfProvider {
    /// The pose of `from` expressed in `to`'s coordinates — equivalently, the
    /// transform that maps a point's coordinates from the `from` frame into the
    /// `to` frame. Mnemonic: "`from`, as seen by `to`."
    ///
    /// This direction is canonical and every impl must honour it. Concretely,
    /// for a sensor mounted 0.5 m forward of the body origin,
    /// `get_transform(sensor, base_link).translation.x == +0.5` (the sensor's
    /// origin, in body axes), while the reverse query returns its inverse. A
    /// caller wanting a sensor's mount *in body axes* therefore queries
    /// `get_transform(sensor, base_link)`, not the reverse.
    ///
    /// Returns `None` when no chain of edges connects the two frames at `at`.
    fn get_transform(
        &self,
        from: FrameId,
        to: FrameId,
        at: MonotonicTime,
    ) -> Option<ErasedTransform>;
}
