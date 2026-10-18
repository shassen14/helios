use crate::spatial::primitives::MonotonicTime;
use crate::spatial::FrameId;

// =========================================================================
// == SensorReading<T> ==
// =========================================================================

pub struct SensorReading<T> {
    pub sensor: FrameId,
    pub timestamp: MonotonicTime,
    pub data: T,
}
