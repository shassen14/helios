//! Where a world sensor was when it captured its latest reading.
//!
//! A world sensor's measurement is expressed in its own frame, so drawing it in
//! the scene needs the sensor's pose at the instant of capture. The pose *now*
//! will not do: the reading on the bus can be a full capture period old, and a
//! moving agent has carried the sensor away from where it measured. The
//! capturing system is the one place that holds the exact pose, so it records
//! it here, beside the reading it publishes.
//!
//! This is sim-side truth and never reaches the bus. The brain receives only
//! the measurement, exactly as it would from a hardware driver.

use helios_core::prelude::MonotonicTime;

use bevy::prelude::*;

/// The pose and time of a world sensor's most recent capture.
///
/// Inserted empty when the sensor spawns and overwritten on every capture, so
/// it always holds the newest one. A consumer pairs it with a reading by
/// matching [`CapturePose::timestamp`] against the reading's timestamp exactly,
/// and treats a mismatch as "pose unknown" rather than falling back to another
/// pose.
#[derive(Default, Debug, Clone, Component)]
pub struct LastCapturePose {
    /// `None` until the sensor's first capture.
    pub latest: Option<CapturePose>,
}

/// One capture: where the sensor was and when.
#[derive(Debug, Clone)]
pub struct CapturePose {
    /// The sensor's world transform the capture was taken from.
    pub pose: GlobalTransform,
    /// The capture instant. The same value as the published reading's
    /// timestamp, so the two can be paired by exact equality.
    pub timestamp: MonotonicTime,
}
