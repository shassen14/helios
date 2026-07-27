//! Interactive visualization: gizmos drawn from live sim/pipeline state.
//!
//! Added only by the `helios_play` bin, never by `HeliosSimulationPlugin` — the
//! headless test bin shares that body and must stay window- and gizmo-free. All
//! viz runs in `Update` (`FixedUpdate` is reserved for sensors/physics/the
//! autonomy tick) and belongs to `VizSet`.

use crate::{
    prelude::AppState,
    viz::{
        interaction::{registration::register_viz_actions, InteractionSet},
        live::{
            estimate::estimate_update_system,
            map::{ensure_map_visible, map_update_system, toggle_map_visibility},
            path::path_update_system,
            pose::pose_update_system,
        },
    },
};

use bevy::prelude::*;

pub mod interaction;
pub mod live;

/// Schedule anchor for every viz system, so later interaction sets (Part C
/// keybindings) have a stable point to order against. One variant for now.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum VizSet {
    Live,
}

pub struct VizPlugin;

impl Plugin for VizPlugin {
    fn build(&self, app: &mut App) {
        // Viz consumes `ActionState`, so it runs after the sampler. Declaring the
        // edge here — on the downstream set — rather than in `ActionRegistryPlugin`
        // keeps that input infrastructure viz-agnostic; knowledge flows one way.
        app.configure_sets(Update, VizSet::Live.after(InteractionSet::Sampling));

        app.add_systems(
            Startup,
            register_viz_actions.in_set(InteractionSet::Registration),
        );

        app.add_systems(
            Update,
            (
                pose_update_system,
                estimate_update_system,
                path_update_system,
                map_update_system,
            )
                .in_set(VizSet::Live)
                .run_if(in_state(AppState::Running)),
        );

        // mapping
        app.add_systems(
            Update,
            (ensure_map_visible, toggle_map_visibility)
                .in_set(VizSet::Live)
                .run_if(in_state(AppState::Running)),
        );
    }
}
