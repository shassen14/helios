pub mod actions;
pub mod camera;
pub mod goal;
pub mod inspector;
pub mod keybindings;
pub mod registration;
pub mod sampling;
pub mod selection;
pub mod teleop;
pub mod tf_panel;
pub mod tuning;

use crate::viz::interaction::{
    actions::registry::ActionRegistry,
    keybindings::load_keybindings,
    sampling::{sample_actions, ActionState},
    tuning::load_interaction_tuning,
};

use bevy::prelude::*;

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
        // Tuning loads in `PreStartup`, not `Startup`: its resources are inserted via
        // deferred commands, and `Startup` consumers (e.g. `spawn_tf_panel`, which reads
        // `TfPanelDockTuning`) must see them already applied. `PreStartup` finishes and
        // flushes before `Startup` begins, so a single earlier load serves every
        // `Startup` reader without per-plugin ordering. It has no dependency on the
        // action-registration sets, so it drops the `InteractionSet` assignment.
        app.add_systems(PreStartup, load_interaction_tuning);
        app.add_systems(Update, sample_actions.in_set(InteractionSet::Sampling));
    }
}
