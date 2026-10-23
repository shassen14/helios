//! Advances every agent's `AutonomyPipeline` one step per `FixedUpdate`.
//!
//! `run_pipeline_tick` runs in `SimulationSet::BrainTick` and executes every
//! DAG node in topological order — the whole brain, not just the estimator.
//! All bus slots are last-known-good; consumers dedupe by `Stamped::timestamp`
//! internally where exactly-once semantics are required.

use crate::brain_bridge::components::{AutonomyPipelineComponent, TfServiceComponent};
use crate::core::components::GroundTruthState;

use bevy::prelude::*;
use std::collections::HashSet;

/// Ticks every agent's `AutonomyPipeline` once per `FixedUpdate`.
///
/// The tick's `now` is the instant the agent's truth describes,
/// [`GroundTruthState::at`], the same value its sensors stamped this step's
/// readings with, so the brain and its inputs agree by the data rather than
/// by the order the sets run in. A pipeline whose agent has no truth can't
/// be given an instant, a scene-wiring bug, so it is skipped with a warning,
/// once per agent.
pub fn run_pipeline_tick(
    mut query: Query<(
        Entity,
        &AutonomyPipelineComponent,
        &mut TfServiceComponent,
        Option<&GroundTruthState>,
    )>,
    time: Res<Time>,
    mut warned_missing_truth: Local<HashSet<Entity>>,
) {
    let dt = time.delta_secs_f64();

    for (agent, pipeline_comp, mut service_comp, truth) in &mut query {
        let Some(truth) = truth else {
            if warned_missing_truth.insert(agent) {
                warn!(
                    "Agent {agent:?} has a pipeline but no GroundTruthState, so its \
                     brain has no instant to tick at and never ticks."
                );
            }
            continue;
        };

        // Fold last tick's dual-published edge samples into the estimated tree,
        // then hand every node a read-only view of it. The `&mut` fold and the
        // shared provider borrow cannot overlap, so all nodes this tick query one
        // frozen buffer regardless of execution order — and, wired only from edge
        // channels, that buffer cannot reach the sim's truth.
        service_comp.0.fold(pipeline_comp.0.bus());
        pipeline_comp
            .0
            .tick(truth.at, dt, service_comp.0.as_provider());
    }
}
