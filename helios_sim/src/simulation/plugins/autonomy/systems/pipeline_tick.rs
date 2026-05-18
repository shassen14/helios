// helios_sim/src/simulation/plugins/autonomy/systems/pipeline_tick.rs
//
// `clear_pipeline_signals` runs in `SimulationSet::Precomputation` to wipe all
// sensor signal slots before sensor systems write fresh data.
// `run_pipeline_tick` runs in `SimulationSet::Estimation` to execute every node
// in topological order after all sensor data has been written to the bus.

use bevy::prelude::*;

use crate::simulation::core::sim_runtime::SimRuntime;
use crate::simulation::core::transforms::TfTree;
use crate::simulation::plugins::autonomy::components::AutonomyPipelineComponent;

/// Clears all sensor-signal slots on every agent's bus at the start of each frame.
///
/// Runs in `SimulationSet::Precomputation` so sensor systems in
/// `SimulationSet::Sensors` always write into freshly cleared slots.
pub fn clear_pipeline_signals(query: Query<&AutonomyPipelineComponent>) {
    for pipeline_comp in &query {
        pipeline_comp.0.bus().clear_signals();
    }
}

/// Ticks every agent's `AutonomyPipeline` once per `FixedUpdate`.
pub fn run_pipeline_tick(
    query: Query<&AutonomyPipelineComponent>,
    tf_tree: Res<TfTree>,
    time: Res<Time>,
) {
    let dt = time.delta_secs_f64();
    let elapsed = time.elapsed_secs_f64();

    for pipeline_comp in &query {
        let runtime = SimRuntime {
            tf: &tf_tree,
            elapsed_secs: elapsed,
        };
        pipeline_comp.0.tick(&runtime, dt);
    }
}
