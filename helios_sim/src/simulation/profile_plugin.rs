// helios_sim/src/simulation/profile_plugin.rs
//
// ProfiledSimulationPlugin: adds only the subsystem plugins required by the
// active SimulationProfile.  HeliosSimulationPlugin delegates to this.

use bevy::prelude::*;

use crate::simulation::core::simulation_setup::SimulationSetupPlugin;
use crate::simulation::core::TopicBusPlugin;
use crate::simulation::plugins::autonomy::{EstimationPlugin, MappingPlugin};
use crate::simulation::plugins::control::ControlPlugin;
use crate::simulation::plugins::debugging::DebuggingPlugin;
use crate::simulation::plugins::foxglove::FoxgloveWebSocketPlugin;
use crate::simulation::plugins::isolation::{
    MockGroundTruthEstimatorPlugin, MockMapInjectorPlugin, MockPathInjectorPlugin,
};
use crate::simulation::plugins::metrics::ControlMetricsPlugin;
use crate::simulation::plugins::planning::PlanningPlugin;
use crate::simulation::plugins::sensors::HeliosSensorsPlugin;
use crate::simulation::plugins::vehicles::HeliosVehiclesPlugin;
use crate::simulation::plugins::world::HeliosWorldPlugin;
use crate::simulation::profile::SimulationProfile;
use crate::simulation::registry::plugin::AutonomyRegistryPlugin;

pub struct ProfiledSimulationPlugin {
    pub profile: SimulationProfile,
}

impl Default for ProfiledSimulationPlugin {
    fn default() -> Self {
        Self {
            profile: SimulationProfile::default(),
        }
    }
}

impl Plugin for ProfiledSimulationPlugin {
    fn build(&self, app: &mut App) {
        // Insert the profile as a resource so systems can query it.
        app.insert_resource(self.profile.clone());

        // Always-present infrastructure.
        app.add_plugins((
            TopicBusPlugin,
            SimulationSetupPlugin,
            AutonomyRegistryPlugin,
            HeliosWorldPlugin,
            HeliosVehiclesPlugin,
            FoxgloveWebSocketPlugin::default(),
            DebuggingPlugin,
        ));

        // Conditional subsystems.
        if self.profile.needs_sensors() {
            app.add_plugins(HeliosSensorsPlugin);
        }
        if self.profile.needs_real_estimation() {
            app.add_plugins(EstimationPlugin);
        }
        if self.profile.needs_mock_estimator() {
            app.add_plugins(MockGroundTruthEstimatorPlugin);
        }
        if self.profile.needs_mapping() {
            app.add_plugins(MappingPlugin);
        }
        if self.profile.needs_planning() {
            app.add_plugins(PlanningPlugin);
        }
        if self.profile.needs_control() {
            app.add_plugins(ControlPlugin);
        }
        if self.profile.needs_mock_path() {
            app.add_plugins(MockPathInjectorPlugin);
        }
        if self.profile.needs_mock_map() {
            app.add_plugins(MockMapInjectorPlugin);
        }
        if self.profile.needs_metrics() {
            app.add_plugins(ControlMetricsPlugin);
        }
    }
}
