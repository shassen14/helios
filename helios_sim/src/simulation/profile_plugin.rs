// helios_sim/src/simulation/profile_plugin.rs
//
// ProfiledSimulationPlugin: adds only the subsystem plugins required by the
// active SimulationProfile.  HeliosSimulationPlugin delegates to this.

use bevy::prelude::*;

use crate::simulation::core::simulation_setup::SimulationSetupPlugin;
use crate::simulation::plugins::autonomy::{EstimationPlugin, MappingPlugin};
use crate::simulation::plugins::control::ControlPlugin;
use crate::simulation::plugins::debugging::DebuggingPlugin;
use crate::simulation::plugins::isolation::{
    MockGroundTruthEstimatorPlugin, MockMapInjectorPlugin, MockPathInjectorPlugin,
};
use crate::simulation::plugins::metrics::ControlMetricsPlugin;
use crate::simulation::plugins::path_following::PathFollowingPlugin;
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
        let caps = self.profile.capabilities();
        app.insert_resource(self.profile.clone());
        app.insert_resource(caps.clone());

        app.add_plugins((
            SimulationSetupPlugin,
            AutonomyRegistryPlugin,
            HeliosWorldPlugin,
            HeliosVehiclesPlugin,
            DebuggingPlugin,
        ));

        if caps.sensors() {
            app.add_plugins(HeliosSensorsPlugin);
        }
        if caps.real_estimation() {
            app.add_plugins(EstimationPlugin);
        }
        if caps.mock_estimator() {
            app.add_plugins(MockGroundTruthEstimatorPlugin);
        }
        if caps.mapping() {
            app.add_plugins(MappingPlugin);
        }
        if caps.planning() {
            app.add_plugins(PlanningPlugin);
            app.add_plugins(PathFollowingPlugin);
        }
        if caps.control() {
            app.add_plugins(ControlPlugin);
        }
        if caps.mock_path() {
            app.add_plugins(MockPathInjectorPlugin);
        }
        if caps.mock_map() {
            app.add_plugins(MockMapInjectorPlugin);
        }
        if caps.metrics() {
            app.add_plugins(ControlMetricsPlugin);
        }
    }
}
