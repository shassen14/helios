// Phase-0 affordance: dump every agent's PortBus snapshot when the user
// presses Backslash. Bypasses the toggle-based DebugVisualizationConfig
// path because this is a one-shot action, not a persistent viz mode.
//
// Lives here (and not in a domain plugin) per the rule that all debug
// keybindings are registered from `debugging/`. Will move into the
// action registry alongside other debug actions once the observability
// hook lands in Phase 11.

use bevy::prelude::*;

use helios_runtime::diagnostics::format_bus;

use crate::simulation::plugins::autonomy::AutonomyPipelineComponent;

pub fn dump_bus_on_keypress(
    keys: Res<ButtonInput<KeyCode>>,
    query: Query<(Entity, &AutonomyPipelineComponent)>,
) {
    if !keys.just_pressed(KeyCode::Backslash) {
        return;
    }
    for (entity, pipeline) in &query {
        let snapshot = format_bus(pipeline.0.bus());
        info!(target: "helios_sim::diagnostics", agent = ?entity, "\n{snapshot}");
    }
}
