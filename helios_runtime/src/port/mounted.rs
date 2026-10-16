//! Mounted payloads: values that each name the frame they were measured in.
//!
//! A channel's description states its mount, the frame its writer promises
//! the values are measured in; a payload implementing [`Mounted`] carries the
//! frame each value was actually measured in, in its header. The bus checks
//! one against the other on [`write_mounted`](crate::port::PortBus::write_mounted),
//! so a reader trusting the description's mount and a reader trusting the
//! header never see different frames for one value.
//!
//! The rule is membership: one batch may hold values of several frames, each
//! of which some writer of the channel declares.

use helios_core::prelude::SensorReading;
use helios_core::spatial::FrameId;

use std::any::Any;

/// A payload whose values each name the frame they were measured in.
pub trait Mounted: Any + Send + Sync {
    /// The frame of each value, in order, read off the values' own headers.
    fn mounts(&self) -> impl Iterator<Item = &FrameId>;
}

/// A batch of readings, each naming its sensor's frame.
impl<P: Send + Sync + 'static> Mounted for Vec<SensorReading<P>> {
    fn mounts(&self) -> impl Iterator<Item = &FrameId> {
        self.iter().map(|reading| &reading.sensor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::{AgentId, MonotonicTime};

    fn reading(leaf: &str) -> SensorReading<f64> {
        SensorReading {
            sensor: FrameId::agent(AgentId::new("car"), leaf),
            timestamp: MonotonicTime(0.0),
            data: 0.0,
        }
    }

    #[test]
    fn a_batch_names_each_readings_frame_in_order() {
        let batch = vec![reading("lidar_front"), reading("lidar_rear")];
        let leaves: Vec<&str> = batch.mounts().map(|frame| frame.leaf().as_str()).collect();
        assert_eq!(leaves, ["lidar_front", "lidar_rear"]);
    }
}
