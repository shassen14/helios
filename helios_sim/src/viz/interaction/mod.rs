use crate::viz::interaction::{
    actions::registry::ActionRegistry,
    keybindings::load_keybindings,
    sampling::{sample_actions, ActionState},
};

use bevy::prelude::*;

pub mod actions;
pub mod camera;
pub mod keybindings;
pub mod registration;
pub mod sampling;

#[derive(SystemSet, Clone, PartialEq, Eq, Hash, Debug)]
pub enum InteractionSet {
    Registration,
    KeyBinding,
    Sampling,
}

pub struct ActionRegistryPlugin;

impl Plugin for ActionRegistryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActionRegistry>()
            .init_resource::<ActionState>()
            .configure_sets(
                Startup,
                (InteractionSet::Registration, InteractionSet::KeyBinding).chain(),
            );

        // `InteractionSet::Sampling` needs no cross-plugin ordering here: it is
        // the *producer* of `ActionState`. The consumer (`VizSet::Live`) declares
        // that it runs after this set, keeping this reusable input infrastructure
        // ignorant of viz — knowledge flows one way, matching the dependency.
        app.add_systems(Startup, load_keybindings.in_set(InteractionSet::KeyBinding));
        app.add_systems(Update, sample_actions.in_set(InteractionSet::Sampling));
    }
}
