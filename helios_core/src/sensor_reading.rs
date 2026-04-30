// helios_core/src/sensor_reading.rs

use crate::types::FrameHandle;

pub struct SensorReading<T> {
    pub sensor_handle: FrameHandle,
    pub timestamp: f64,
    pub data: T,
}
