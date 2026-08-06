//! Egress: drains the pipeline's actuator terminal to the actuation component.
//!
//! `publish_pipeline_actuators` runs in `SimulationSet::BrainOutput` and copies
//! the pipeline's latest `ActuatorCommand` — the per-actuator setpoints the
//! allocator produces — into `ActuatorCommandComponent`, which the vehicle
//! plugin applies to physics in `Actuation`. The allocator itself is a DAG node
//! inside `BrainTick`; this is only the bus→ECS bridge.

use crate::{
    brain_bridge::components::AutonomyPipelineComponent,
    core::components::ActuatorCommandComponent, prelude::*,
};

/// Copies the pipeline's latest actuator command (written by the allocator
/// node inside the DAG during `SimulationSet::BrainTick`) into
/// `ActuatorCommandComponent` so the vehicle plugin in `SimulationSet::Actuation`
/// can apply it.
pub fn publish_pipeline_actuators(
    mut query: Query<(&AutonomyPipelineComponent, &mut ActuatorCommandComponent)>,
) {
    let _span = tracing::trace_span!("sim.actuation.publish").entered();
    for (pipeline, mut output) in &mut query {
        if let Some(stamped) = pipeline.0.read_actuators() {
            output.0 = stamped.value.clone();
        }
    }
}
