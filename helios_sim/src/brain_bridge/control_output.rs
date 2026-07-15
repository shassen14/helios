//! Egress: drains the pipeline's control output to the actuation component.
//!
//! `publish_pipeline_control` runs in `SimulationSet::BrainOutput` and copies
//! the pipeline's latest `ControlOutput` into `ControlOutputComponent`, which
//! the vehicle adapter consumes in `Actuation`. The controller itself is a DAG
//! node inside `BrainTick`; this is only the bus→ECS bridge.

use crate::{
    prelude::*,
    {
        brain_bridge::components::AutonomyPipelineComponent,
        core::components::ControlOutputComponent,
    },
};

/// Copies the pipeline's latest `ControlOutput` (written by the controller
/// node inside the DAG during `SimulationSet::BrainTick`) into
/// `ControlOutputComponent` so the vehicle adapter in `SimulationSet::Actuation`
/// can read it.
pub fn publish_pipeline_control(
    mut query: Query<(&AutonomyPipelineComponent, &mut ControlOutputComponent)>,
) {
    let _span = tracing::trace_span!("sim.control.publish").entered();
    for (pipeline, mut output) in &mut query {
        if let Some(stamped) = pipeline.0.read_control() {
            output.0 = stamped.value.clone();
        }
    }
}
