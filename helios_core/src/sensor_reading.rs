use crate::types::{FrameHandle, MonotonicTime};

pub struct SensorReading<T> {
    pub sensor_handle: FrameHandle,
    pub timestamp: MonotonicTime,
    pub data: T,
}
