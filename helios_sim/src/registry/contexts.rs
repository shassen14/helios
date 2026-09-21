use crate::config::structs::{
    CollisionConfig, MountConfig, PlantConfig, TopologyConfig, VisualConfig,
};

use helios_core::control::actuation_model::ActuationModel;
use helios_core::data::AgentId;

use bevy::prelude::*;

//   'a  — borrows the dispatch system's data for one build call
//   'w  — Bevy's "world" lifetime on Commands
//   's  — Bevy's "system-state" lifetime on Commands
pub struct TopologyBuildContext<'a, 'w, 's> {
    pub entity: Entity,
    /// Stable identity of the agent being built. The topology builder stamps the
    /// body's `FrameId::base_link(agent)` and each mount's frame from it, so the
    /// TF tree keys them by the same identity the pipeline's models reference.
    pub agent: AgentId,
    pub commands: &'a mut Commands<'w, 's>,
    pub config: &'a TopologyConfig,
    pub start_transform: Transform,
}

pub struct PlantBuildContext<'a, 'w, 's> {
    pub entity: Entity,
    pub commands: &'a mut Commands<'w, 's>,
    pub config: &'a PlantConfig,
    pub actuation: &'a ActuationModel,
    pub mounts: &'a [MountConfig],
}

pub struct CollisionBuildContext<'a, 'w, 's> {
    pub entity: Entity,
    pub commands: &'a mut Commands<'w, 's>,
    pub config: &'a CollisionConfig,
}

pub struct VisualBuildContext<'a, 'w, 's> {
    pub entity: Entity,
    pub commands: &'a mut Commands<'w, 's>,
    pub config: &'a VisualConfig,
    pub mounts: &'a [MountConfig],
    pub meshes: &'a mut Assets<Mesh>,
    pub materials: &'a mut Assets<StandardMaterial>,
}
