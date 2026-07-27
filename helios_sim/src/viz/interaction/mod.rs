use crate::viz::interaction::actions::registry::ActionRegistry;

use bevy::prelude::*;

pub mod actions;
pub mod registration;

// TODO: future Sampling, Keybinding set?
#[derive(SystemSet, Clone, PartialEq, Eq, Hash, Debug)]
pub enum InteractionSet {
    Registration,
}

pub struct ActionRegistryPlugin;

impl Plugin for ActionRegistryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActionRegistry>()
            .configure_sets(Startup, InteractionSet::Registration);
    }
}
