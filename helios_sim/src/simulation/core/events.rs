// helios_sim/src/simulation/core/events.rs
use bevy::prelude::Event;
// Import the pure data struct from the core library
use helios_core::frames::MeasurementEvent;

// This is the Bevy-specific event. It can derive `Event`.
#[derive(Event, Clone)]
pub struct BevyMeasurementEvent(pub MeasurementEvent); // It's a simple "tuple-struct" wrapper
