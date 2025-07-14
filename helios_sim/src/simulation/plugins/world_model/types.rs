// helios_sim/src/plugins/world_model/types.rs

use bevy::prelude::Resource;
use helios_core::{
    messages::{MeasurementMessage, ModuleInput},
    types::Control,
};

/// The owned version of the ModuleInput enum for storing in a resource.
#[derive(Clone, Debug)]
pub enum OwnedModuleInput {
    TimeStep { dt: f64, current_time: f64 },
    Control { u: Control },
    Measurement { message: MeasurementMessage },
}

impl OwnedModuleInput {
    /// Helper to convert the owned type to a borrowed reference for processing.
    pub fn as_ref<'a>(&'a self) -> ModuleInput<'a> {
        match self {
            OwnedModuleInput::TimeStep { dt, current_time } => ModuleInput::TimeStep {
                dt: *dt,
                current_time: *current_time,
            },
            OwnedModuleInput::Control { u } => ModuleInput::Control { u },
            OwnedModuleInput::Measurement { message } => ModuleInput::Measurement { message },
        }
    }
}

/// The Bevy resource that holds all inputs for the current frame.
#[derive(Resource, Default)]
pub struct FrameInputs(pub Vec<OwnedModuleInput>);
