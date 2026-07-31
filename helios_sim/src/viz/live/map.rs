//! Occupancy-grid gizmo — the agent's 2-D map drawn as ground-plane cells.
//!
//! Like `path.rs`, `MapData` is a *plural* channel whose names belong to the
//! stack, so this pulls every declared producer via [`declared_outputs`] rather
//! than hardcoding a name. Only occupied cells are drawn; a full grid of
//! unknown/free cells would be visual noise and one gizmo per cell gets
//! expensive fast.
//!
//! The map is *not* redrawn from a per-frame scan — the occupancy grid is a
//! rolling-window log-odds map that accumulates belief over time, so the cells
//! shown are everything the agent currently remembers as occupied. (Bevy
//! gizmos are immediate-mode and already cleared each frame; the persistence
//! is in the map data, not here.) Because those cells are placed from the
//! agent's *estimated* pose, a divergent estimate shows up here as cells
//! offset from the true obstacles — the map faithfully draws a wrong belief.
//!
use crate::{
    core::transforms::EnuVector,
    prelude::AutonomyPipelineComponent,
    viz::{
        interaction::{
            actions::{
                handle::{ActionHandle, ActionId},
                registry::ActionRegistry,
            },
            sampling::ActionState,
        },
        live::discovery::{declared_outputs, declares_output},
    },
};

use helios_core::mapping::MapData;

use bevy::{color, prelude::*};
use nalgebra::Vector3;
use std::f32::consts::FRAC_PI_2;

/// Cells at or below this map value (unknown ≈ 127, free < 127, occupied >
/// 127) are skipped; only confidently occupied cells are drawn.
// TODO: pull the occupancy cutoff from viz config once that surface exists.
const OCCUPIED_THRESHOLD: u8 = 127;

/// Occupied-cell color. Muted green-grey, distinct from the amber path line
/// and the red/green/blue pose triads it is drawn among.
// TODO: pull the map color from viz config once that surface exists.
const MAP_COLOR: color::Color = color::Color::Srgba(color::Srgba {
    red: 0.5,
    green: 0.65,
    blue: 0.5,
    alpha: 1.0,
});

pub(crate) fn map_update_system(
    query: Query<(&AutonomyPipelineComponent, &MapVisible)>,
    mut gizmos: Gizmos,
) {
    for (pipeline, is_map_visible) in &query {
        if !is_map_visible.0 {
            continue;
        }

        for map in declared_outputs::<MapData>(&pipeline.0) {
            let MapData::OccupancyGrid2D {
                origin,
                resolution,
                data,
                ..
            } = &map.value
            else {
                continue;
            };

            // `data[(row, col)]`: row 0 = south (+row → +north → +Y ENU),
            // col 0 = west (+col → +east → +X ENU); `origin` is the SW corner.
            for row in 0..data.nrows() {
                for col in 0..data.ncols() {
                    if data[(row, col)] <= OCCUPIED_THRESHOLD {
                        continue;
                    }

                    // Cell *center* in world ENU (the +0.5 centers the square
                    // on the cell). The map is a ground-plane belief, so z = 0;
                    // elevation would be a different MapData variant, not this.
                    let x = origin.translation.x + (col as f64 + 0.5) * resolution;
                    let y = origin.translation.y + (row as f64 + 0.5) * resolution;
                    let center_enu = EnuVector(Vector3::<f64>::new(x, y, 0.0));

                    // ENU center → Bevy point via the sole sanctioned helper.
                    let p = Vec3::from(center_enu);

                    // A cell-sized square laid flat on the ground. `rect` draws
                    // in its local XY plane; rotating +90° about X drops that
                    // plane onto Bevy's XZ ground. The tilt is a rendering
                    // choice (not a frame conversion), so it stays viz-local.
                    let cell = Isometry3d::new(p, Quat::from_rotation_x(FRAC_PI_2));
                    gizmos.rect(cell, Vec2::splat(*resolution as f32), MAP_COLOR);
                }
            }
        }
    }
}

/// Per-agent switch for drawing that agent's occupancy-grid gizmo.
///
/// Deliberately a component, not a resource: visibility is per-agent, so a fleet
/// can show one robot's map, several, or all at once — a single global flag
/// could only ever toggle every map together. It is attached only to agents that
/// actually run a mapper (see `ensure_map_visible`), so its presence means
/// "this agent has a map to show", and `map_update_system` reads it per entity.
#[derive(Component)]
pub struct MapVisible(pub bool);

/// Flip every agent's [`MapVisible`] when the `viz.toggle_map` action fires.
///
/// Reads only the global [`ActionState`] — the human's intent — and applies it
/// to the per-agent state, never touching a key itself. The action's handle
/// cannot change after startup, so it is resolved once and cached in a `Local`.
/// With no selection yet the toggle targets every agent (the "toggle everyone"
/// mode); when selection arrives, the sole change here is a filter on the query.
pub(crate) fn toggle_map_visibility(
    registry: Res<ActionRegistry>,
    state: Res<ActionState>,
    mut agents: Query<&mut MapVisible>,
    mut handle: Local<Option<ActionHandle>>,
) {
    let h = *handle.get_or_insert_with(|| {
        registry
            .handle(ActionId("viz.toggle_map"))
            .expect("registered")
    });

    if state.is_active(h) {
        for mut is_map_visible in &mut agents {
            is_map_visible.0 = !is_map_visible.0;
        }
    }
}

/// Attach [`MapVisible`] to any map-producing agent that lacks it.
///
/// The backfill that keeps viz state out of the shared scene build: agents are
/// spawned by the headless-shared `HeliosSimulationPlugin`, so the visibility
/// component is added here instead, from a `helios_play`-only system. Gated on
/// [`declares_output`]`::<MapData>` — the topology capability test, not
/// [`declared_outputs`], which is also empty before the first map publishes — so
/// the flag lands only on agents that run a mapper and never on one that could
/// not honor it. `Without<MapVisible>` makes it idempotent: each agent is tagged
/// once, then matches nothing. The insert is deferred through `Commands`, so a
/// just-spawned agent's map is simply skipped for one frame and drawn the next.
pub(crate) fn ensure_map_visible(
    mut commands: Commands,
    agents: Query<(Entity, &AutonomyPipelineComponent), Without<MapVisible>>,
) {
    for (e, pipeline) in &agents {
        if !declares_output::<MapData>(&pipeline.0) {
            continue;
        }
        commands.entity(e).insert(MapVisible(true));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::viz::interaction::actions::handle::{ActionMetadata, InputKind};

    /// A registry with `viz.toggle_map` registered, and its handle — enough for
    /// `toggle_map_visibility` to resolve the action it gates on.
    fn toggle_registry() -> (ActionRegistry, ActionHandle) {
        let mut registry = ActionRegistry::default();
        let handle = registry.register(
            ActionId("viz.toggle_map"),
            ActionMetadata {
                label: "Toggle map",
                group: "viz",
                kind: InputKind::Button,
                default_key: KeyCode::KeyM,
            },
        );
        (registry, handle)
    }

    /// An `App` running the toggle over one agent that starts visible.
    fn toggle_app(registry: ActionRegistry, state: ActionState) -> (App, Entity) {
        let mut app = App::new();
        app.insert_resource(registry);
        app.insert_resource(state);
        let agent = app.world_mut().spawn(MapVisible(true)).id();
        app.add_systems(Update, toggle_map_visibility);
        (app, agent)
    }

    #[test]
    fn firing_the_action_flips_visibility() {
        let (registry, handle) = toggle_registry();
        let (mut app, agent) = toggle_app(registry, ActionState::from_active([handle]));

        app.update();

        assert!(!app.world().get::<MapVisible>(agent).unwrap().0);
    }

    #[test]
    fn an_idle_action_leaves_visibility_untouched() {
        let (registry, _handle) = toggle_registry();
        // Nothing active this frame, so the visible agent stays visible.
        let (mut app, agent) = toggle_app(registry, ActionState::default());

        app.update();

        assert!(app.world().get::<MapVisible>(agent).unwrap().0);
    }
}
