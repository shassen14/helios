use super::primitives::{FrameHandle, MonotonicTime};

// =========================================================================
// == SensorReading<T> ==
// =========================================================================

pub struct SensorReading<T> {
    pub sensor_handle: FrameHandle,
    pub timestamp: MonotonicTime,
    pub data: T,
}
