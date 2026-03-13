// helios_sim/src/simulation/plugins/research/
//
// ResearchPlugin: adds DataLoggerPlugin + wires Monte Carlo state.

pub mod data_logger;
pub mod monte_carlo;

pub use data_logger::DataLoggerPlugin;
pub use monte_carlo::{MonteCarloConfig, RunResult};

use bevy::prelude::*;

/// Adds the data logger and exposes Monte Carlo configuration.
pub struct ResearchPlugin;

impl Plugin for ResearchPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(DataLoggerPlugin);
    }
}
