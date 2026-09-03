use crate::{
    data::MonotonicTime,
    frames::{transforms::ErasedTransform, FrameId},
};

/// Abstraction over any system that can answer transform queries between coordinate frames.
///
/// `helios_sim` implements this as `TfTree` (Bevy resource). `helios_hw` will implement it
/// as a hardware-clock-backed calibration tree. Filters receive `&dyn TfProvider` via
/// `FilterContext` — they never depend on the concrete host type.
pub trait TfProvider {
    fn get_transform(
        &self,
        from: FrameId,
        to: FrameId,
        at: MonotonicTime,
    ) -> Option<ErasedTransform>;
}
