// src/simulation/core/spawn_requests.rs
use crate::prelude::AgentConfig;
use bevy::prelude::Component;

#[derive(Component, Clone)]
pub struct SpawnAgentConfigRequest(pub AgentConfig);
