// src/simulation/core/registries.rs

use bevy::ecs::system::EntityCommands;
use bevy::prelude::*;
use std::collections::HashMap;

use crate::prelude::AutonomyStack;
// Import config enums. We need them to define the function signatures.
use crate::simulation::core::config::{
    ControllerConfig, EstimatorConfig, MapperConfig, PlannerConfig, SensorConfig, Vehicle,
};
use crate::simulation::core::topics::TopicBus;

// --- Define the "Spawner" Function Types ---
// A spawner is a function that knows how to add the right components to an entity
// based on a specific piece of configuration.

// Takes the vehicle-specific part of the config and adds components like AckermannParameters.
pub type VehicleSpawner = Box<dyn Fn(&mut EntityCommands, &Vehicle) + Send + Sync>;

// Takes a single sensor's config and adds components like ImuSuite or LidarModel.
// Sensors CREATE topics, so they need mutable access.
// pub type SensorSpawner =
//     Box<dyn Fn(&str, &mut EntityCommands, &SensorConfig, &mut TopicBus) + Send + Sync>;

// Estimators READ topics, so they only need read-only access.
// pub type EstimatorSpawner =
//     Box<dyn Fn(&str, &mut EntityCommands, &EstimatorConfig, &TopicBus) + Send + Sync>;
pub type EstimatorSpawner = Box<
    dyn Fn(
            // It needs these arguments to do its job
            &mut EntityCommands,
            &Name,          // The agent's name
            &AutonomyStack, // The full autonomy stack config
            &TopicBus,
        ) + Send
        + Sync,
>;

pub type MapperSpawner = Box<dyn Fn(&mut EntityCommands, &MapperConfig) + Send + Sync>;
pub type PlannerSpawner = Box<dyn Fn(&mut EntityCommands, &PlannerConfig) + Send + Sync>;
pub type ControllerSpawner = Box<dyn Fn(&mut EntityCommands, &ControllerConfig) + Send + Sync>;

// --- Define the Registry Resources ---
// These are the Bevy resources that will store the mapping from a
// string (from the TOML file) to the spawner function.

#[derive(Resource, Default)]
pub struct VehicleRegistry(pub HashMap<String, VehicleSpawner>);

// #[derive(Resource, Default)]
// pub struct SensorRegistry(pub HashMap<String, SensorSpawner>);

#[derive(Resource, Default)]
pub struct EstimatorRegistry(pub HashMap<String, EstimatorSpawner>);

#[derive(Resource, Default)]
pub struct MapperRegistry(pub HashMap<String, MapperSpawner>);

#[derive(Resource, Default)]
pub struct PlannerRegistry(pub HashMap<String, PlannerSpawner>);

#[derive(Resource, Default)]
pub struct ControllerRegistry(pub HashMap<String, ControllerSpawner>);
