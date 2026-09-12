use super::primitives::MonotonicTime;
use crate::frames::FrameId;

// =========================================================================
// == SensorReading<T> ==
// =========================================================================

pub struct SensorReading<T> {
    pub sensor: FrameId,
    pub timestamp: MonotonicTime,
    pub data: T,
}
