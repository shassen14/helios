use std::collections::HashMap;

use super::contexts::{CollisionBuildContext, PlantBuildContext, TopologyBuildContext};

use bevy::prelude::*;

/// A builder writes an axis's components onto `ctx.entity` through `ctx.commands`
/// and returns `Ok(())`, or `Err(msg)` if it was handed the wrong config variant
/// — a registry-wiring bug, not bad user config. Plain fn pointers (like the
/// runtime's node factories), so they are `Copy` and live in a resource.
pub type TopologyBuilder = fn(&mut TopologyBuildContext) -> Result<(), String>;
pub type PlantBuilder = fn(&mut PlantBuildContext) -> Result<(), String>;
pub type CollisionBuilder = fn(&mut CollisionBuildContext) -> Result<(), String>;

/// Maps each embodiment axis's `kind` string to the builder that constructs it.
///
/// Populated by the vehicle plugins at startup (never auto-registered), so a new
/// agent type contributes a builder without any existing code changing. Read by
/// the `build_embodiment` dispatch system, which looks up each section's
/// `kind_str` and calls the matching builder.
#[derive(Resource, Default)]
pub struct EmbodimentRegistry {
    topology: HashMap<String, TopologyBuilder>,
    plant: HashMap<String, PlantBuilder>,
    collision: HashMap<String, CollisionBuilder>,
}

impl EmbodimentRegistry {
    pub fn register_topology(&mut self, kind: &str, builder: TopologyBuilder) {
        self.topology.insert(kind.to_string(), builder);
    }

    pub fn register_plant(&mut self, kind: &str, builder: PlantBuilder) {
        self.plant.insert(kind.to_string(), builder);
    }

    pub fn register_collision(&mut self, kind: &str, builder: CollisionBuilder) {
        self.collision.insert(kind.to_string(), builder);
    }

    pub fn topology(&self, kind: &str) -> Option<TopologyBuilder> {
        self.topology.get(kind).copied()
    }

    pub fn plant(&self, kind: &str) -> Option<PlantBuilder> {
        self.plant.get(kind).copied()
    }

    pub fn collision(&self, kind: &str) -> Option<CollisionBuilder> {
        self.collision.get(kind).copied()
    }
}

/// Initialises the empty [`EmbodimentRegistry`]. Add it to `HeliosSimulationPlugin`
/// *before* the vehicle plugins, which populate it during their own `build`.
pub struct EmbodimentRegistryPlugin;

impl Plugin for EmbodimentRegistryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<EmbodimentRegistry>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn noop_topology(_: &mut TopologyBuildContext) -> Result<(), String> {
        Ok(())
    }

    #[test]
    fn registered_kind_is_found() {
        let mut registry = EmbodimentRegistry::default();
        registry.register_topology("RigidBodyWithMount", noop_topology);
        assert!(registry.topology("RigidBodyWithMount").is_some());
    }

    #[test]
    fn unknown_kind_is_none() {
        let registry = EmbodimentRegistry::default();
        assert!(registry.topology("NoSuchTopology").is_none());
    }
}
