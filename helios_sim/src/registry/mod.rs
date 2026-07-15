// VehicleAdapterRegistry: sim-only registry for AckermannOutputAdapter factories.
// All algorithm factories (estimators, mappers, controllers, etc.) live in
// `helios_runtime::registry::AutonomyRegistry`, wrapped as `RuntimeAutonomyRegistry`.

pub mod adapters;
pub mod contexts;
pub mod plugin;

pub use contexts::{AdapterBuildContext, AdapterFactory};

use std::collections::HashMap;

use bevy::prelude::Resource;

use crate::agents::vehicles::ackermann::adapter::AckermannOutputAdapter;
use contexts::AdapterFactory as AdapterFactoryAlias;

/// Sim-only registry for vehicle output adapters.
///
/// Algorithm factories (EKF, mappers, planners, etc.) are in
/// `RuntimeAutonomyRegistry` (wraps `helios_runtime::registry::AutonomyRegistry`).
#[derive(Resource, Default)]
pub struct VehicleAdapterRegistry {
    pub adapters: HashMap<String, AdapterFactoryAlias>,
}

impl VehicleAdapterRegistry {
    pub fn register_adapter<F>(&mut self, key: &str, factory: F) -> &mut Self
    where
        F: Fn(AdapterBuildContext) -> Result<Box<dyn AckermannOutputAdapter>, String>
            + Send
            + Sync
            + 'static,
    {
        self.adapters
            .insert(key.to_string(), std::sync::Arc::new(factory));
        self
    }

    pub fn build_adapter(
        &self,
        key: &str,
        ctx: AdapterBuildContext,
    ) -> Result<Box<dyn AckermannOutputAdapter>, String> {
        self.adapters
            .get(key)
            .ok_or_else(|| format!("No adapter registered for '{key}'."))
            .and_then(|f| f(ctx))
    }
}
