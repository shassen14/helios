// Factory type aliases and build context structs for the VehicleAdapterRegistry.

use std::sync::Arc;

use bevy::prelude::Entity;

use crate::agents::vehicles::ackermann::adapter::AckermannOutputAdapter;
use crate::config::structs::AckermannAdapterConfig;

pub type AdapterFactory = Arc<
    dyn Fn(AdapterBuildContext) -> Result<Box<dyn AckermannOutputAdapter>, String> + Send + Sync,
>;

pub struct AdapterBuildContext {
    pub agent_entity: Entity,
    pub adapter_cfg: AckermannAdapterConfig,
}
