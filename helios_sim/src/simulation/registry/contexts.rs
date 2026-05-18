// helios_sim/src/simulation/registry/contexts.rs
//
// Factory type aliases and build context structs for the VehicleAdapterRegistry.

use std::sync::Arc;

use bevy::prelude::Entity;

use crate::simulation::config::structs::AckermannAdapterConfig;
use crate::simulation::plugins::vehicles::ackermann::adapter::AckermannOutputAdapter;

pub type AdapterFactory = Arc<
    dyn Fn(AdapterBuildContext) -> Result<Box<dyn AckermannOutputAdapter>, String> + Send + Sync,
>;

pub struct AdapterBuildContext {
    pub agent_entity: Entity,
    pub adapter_cfg: AckermannAdapterConfig,
}
