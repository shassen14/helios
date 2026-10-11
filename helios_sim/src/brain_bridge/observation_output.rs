//! Egress: collects what every agent's pipeline recorded this tick into one
//! batch, each observation tagged with its agent.
//!
//! Sinks read the batch; only the drain writes it.

use crate::prelude::{AgentIdComponent, AutonomyPipelineComponent};

use helios_runtime::observe::observation::{AgentObservation, Observation};

use bevy::prelude::*;

/// One tick's observations from every agent, in agent query order, and per
/// agent in the order its pipeline drained them.
///
/// A resource the drain clears itself, not Bevy `Messages`: messages are
/// cleared once per frame, but the drain runs in `FixedUpdate`, which runs
/// zero or several times a frame, so a frame could lose ticks or see the same
/// tick twice.
#[derive(Debug, Default, Resource)]
pub struct ObservationBatch(Vec<AgentObservation>);

impl ObservationBatch {
    pub fn observations(&self) -> &[AgentObservation] {
        &self.0
    }
}

/// Replaces the batch with what every pipeline recorded since the last drain.
///
/// Runs in `BrainTick`, chained after `run_pipeline_tick`, so the batch holds
/// the tick that just ran. It drains every agent every tick, watched or not:
/// a pipeline's buffers are emptied only by a drain.
pub fn drain_observations(
    pipelines: Query<(&AutonomyPipelineComponent, &AgentIdComponent)>,
    mut batch: ResMut<ObservationBatch>,
    // Reused every tick, so its allocation is kept; `drain(..)` empties it.
    mut scratch: Local<Vec<Observation>>,
) {
    batch.0.clear();

    for (pipeline, agent) in &pipelines {
        pipeline.0.drain_into(&mut scratch);

        let tagged = scratch.drain(..).map(|observation| AgentObservation {
            agent: agent.0.clone(),
            observation,
        });

        batch.0.extend(tagged);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_runtime::{observe::observation::ObservedValue, pipeline::PipelineBuilder};

    use bevy::ecs::system::RunSystemOnce;

    const CAR: &str = "car";

    fn world_with_batch(batch: ObservationBatch) -> World {
        let mut world = World::new();
        world.insert_resource(batch);
        world
    }

    fn drain(world: &mut World) {
        world
            .run_system_once(drain_observations)
            .expect("drain_observations runs");
    }

    #[test]
    fn last_ticks_batch_is_cleared_even_with_no_agents() {
        let stale = AgentObservation {
            agent: AgentId::new(CAR),
            observation: Observation {
                node: "estimator".into(),
                leaf: "aiding.gps.nis".into(),
                timestamp: MonotonicTime(0.0),
                value: ObservedValue::Scalar(1.0),
            },
        };
        let mut world = world_with_batch(ObservationBatch(vec![stale]));

        drain(&mut world);

        assert!(world
            .resource::<ObservationBatch>()
            .observations()
            .is_empty());
    }

    #[test]
    fn an_agent_watching_nothing_leaves_the_batch_empty() {
        let mut world = world_with_batch(ObservationBatch::default());
        let pipeline = PipelineBuilder::new()
            .build()
            .expect("an empty pipeline builds");
        world.spawn((
            AutonomyPipelineComponent(pipeline),
            AgentIdComponent(AgentId::new(CAR)),
        ));

        drain(&mut world);
        drain(&mut world);

        assert!(world
            .resource::<ObservationBatch>()
            .observations()
            .is_empty());
    }
}
