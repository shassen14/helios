//! Advances every agent's `AutonomyPipeline` one step per `FixedUpdate`.
//!
//! `run_pipeline_tick` runs in `SimulationSet::BrainTick` and executes every
//! DAG node in topological order — the whole brain, not just the estimator.
//! All bus slots are last-known-good; consumers dedupe by `Stamped::timestamp`
//! internally where exactly-once semantics are required.

use bevy::prelude::*;

use crate::brain_bridge::components::AutonomyPipelineComponent;
use crate::core::sim_runtime::SimRuntime;
use crate::core::transforms::TfTree;

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
