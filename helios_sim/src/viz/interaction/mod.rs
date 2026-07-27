use crate::viz::interaction::actions::registry::ActionRegistry;

use bevy::prelude::*;

pub mod actions;
pub mod keybindings;
pub mod registration;

// TODO: future Sampling
#[derive(SystemSet, Clone, PartialEq, Eq, Hash, Debug)]
pub enum InteractionSet {
    Registration,
    KeyBinding,
}

pub struct ActionRegistryPlugin;

impl Plugin for ActionRegistryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActionRegistry>().configure_sets(
            Startup,
            (InteractionSet::Registration, InteractionSet::KeyBinding).chain(),
        );

        app.add_systems(
            Startup,
            keybindings::load_keybindings.in_set(InteractionSet::KeyBinding),
        );
    }
}
