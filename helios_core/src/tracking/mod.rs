// helios_core/src/tracking/mod.rs
//
// Tracker trait for object-level tracking. No concrete implementations in Phase 2.

use crate::estimation::FilterContext;
use crate::frames::FrameAwareState;
use crate::messages::MeasurementMessage;

/// A single tracked object in the environment.
pub struct Track {
    pub id: u64,
    pub state: FrameAwareState,
}

/// A stateful tracker that maintains a set of tracked objects.
pub trait Tracker: Send + Sync {
    fn update(&mut self, msg: &MeasurementMessage, context: &FilterContext);
    fn get_tracks(&self) -> &[Track];
}
