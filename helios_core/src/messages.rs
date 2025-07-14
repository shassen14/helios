// helios_core/src/messages.rs

use crate::types::FrameHandle;
use nalgebra::{Vector3, Vector6}; // Example static vectors

/// A rich, self-describing container for sensor data.
#[derive(Clone, Debug)]
pub enum MeasurementData {
    Imu6Dof(Vector6<f64>),
    Imu9Dof {
        base_data: Vector6<f64>,
        magnetic_field: Vector3<f64>,
    },
    Gps(Vector3<f64>),
    // ...
}

/// The generic event carrying measurement data.
/// This replaces the old MeasurementEvent.
#[derive(Clone, Debug)]
pub struct MeasurementMessage {
    pub agent_handle: FrameHandle,
    pub sensor_handle: FrameHandle,
    pub timestamp: f64,
    pub data: MeasurementData,
}

// And our universal input packet also fits perfectly here.
use crate::frames::FrameAwareState;
use crate::types::Control;

pub enum ModuleInput<'a> {
    TimeStep { dt: f64, current_time: f64 },
    Control { u: &'a Control },
    Measurement { message: &'a MeasurementMessage },
    PoseUpdate { pose: &'a FrameAwareState },
}
