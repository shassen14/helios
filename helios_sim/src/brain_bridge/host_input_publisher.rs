//! The single path every host *input* uses to publish onto its agent's bus.
//!
//! A host input is any value the host injects into the brain that is not a
//! sensor reading: a mission goal, a teleop command, a behavior-tree directive.
//! Choosing the channel name and the value stays with each input's own system —
//! those differ per input. Everything after that is identical: wrap the value
//! in a `Stamped` envelope with the host producer id, build the named internal
//! channel key, write, and turn a dropped write into one loud warning. That
//! shared tail lives here as [`HostInputPublisher`], so there is exactly one
//! place that constructs the envelope and one place that handles a missing
//! channel.
//!
//! This is the host-input twin of `SensorPublisher`, with three deliberate
//! differences: the value is a single item, not a batch; it rides an
//! `InternalChannel` (a DAG-node input), not a `SensorChannel`; and its
//! timestamp is supplied by the caller rather than read off the payload, since
//! a host input has no intrinsic reading time.

use crate::prelude::AutonomyPipelineComponent;

use helios_core::data::MonotonicTime;
use helios_runtime::{port::InternalChannel, Health, Stamped, HOST_PRODUCER_ID};

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use std::collections::HashSet;

/// Bundles what every host-input system needs to publish: the query that
/// resolves an agent entity to its `AutonomyPipeline`, and a per-system set of
/// channels already warned about. An input system takes this one param and its
/// publish tail collapses to a single [`publish`](Self::publish) call.
///
/// `warned_channels` is a [`Local`], so each system that takes a
/// `HostInputPublisher` gets its own set. That is deliberate: a given host-input
/// channel is written by exactly one system, so per-system dedup warns once per
/// broken channel — the same guarantee a global set would give, without a
/// shared-mutable resource serializing the input systems against each other.
#[derive(SystemParam)]
pub struct HostInputPublisher<'w, 's> {
    pipelines: Query<'w, 's, &'static AutonomyPipelineComponent>,
    warned_channels: Local<'s, HashSet<String>>,
}

impl HostInputPublisher<'_, '_> {
    /// Publishes a single host input to `channel` on `agent`'s pipeline bus,
    /// stamped at `timestamp`.
    ///
    /// - An agent with no pipeline (not yet spawned during scene build, or a
    ///   brainless prop) is a silent no-op — legal, not an error.
    /// - `timestamp` comes from the caller, not the value: a host input has no
    ///   intrinsic reading time, so the system that emits it decides "when"
    ///   (typically the current tick clock).
    /// - The value rides an `InternalChannel` keyed by `channel`, the same slot
    ///   a DAG node declares as an input — so the consumer (a planner reading
    ///   its goal, say) and the host agree by this one name.
    /// - A write to an unwired channel warns exactly once per channel name,
    ///   naming it: the node's configured input channel and the host's declared
    ///   channel disagree, and the input is being dropped.
    pub fn publish<T: Send + Sync + 'static>(
        &mut self,
        agent: Entity,
        channel: &str,
        value: T,
        timestamp: MonotonicTime,
    ) {
        let Ok(pipeline) = self.pipelines.get(agent) else {
            return;
        };

        let stamped_message = Stamped {
            value,
            timestamp,
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        let write_result = pipeline
            .0
            .bus()
            .write(InternalChannel::named::<T>(channel).into(), stamped_message);

        if write_result.is_err() && self.warned_channels.insert(channel.to_string()) {
            warn!(
                "Channel '{}' has no slot in the autonomy DAG; the host input is being \
                 dropped.",
                channel
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::planning::types::PlannerGoal;

    use bevy::ecs::system::SystemState;
    use nalgebra::Vector2;

    /// An agent without an `AutonomyPipelineComponent` — a prop, or an agent
    /// mid-scene-build — must be a silent no-op, not a panic. This is the only
    /// branch reachable without a fully assembled pipeline; asserting a goal
    /// actually reaches a planner is an end-to-end concern that lives in
    /// `helios_test`, not here.
    #[test]
    fn publish_to_agent_without_pipeline_is_a_noop() {
        let mut world = World::new();
        let agent = world.spawn_empty().id();

        let mut state: SystemState<HostInputPublisher> = SystemState::new(&mut world);
        let mut publisher = state.get_mut(&mut world).unwrap();

        publisher.publish(
            agent,
            "mission",
            PlannerGoal::WorldPosition2D(Vector2::new(1.0, 2.0)),
            MonotonicTime(0.0),
        );
    }
}
