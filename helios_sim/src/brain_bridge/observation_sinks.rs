//! Egress: hands each tick's observation batch to every registered sink, and
//! tells each agent's pipeline what the sinks asked to have recorded.
//!
//! A sink is registered once, while the app is built, with
//! [`add_observation_sink`](ObservationSinkAppExt::add_observation_sink). Its
//! request joins one request for the whole host, which is resolved against
//! every agent at scene build. After every drain, each sink receives the
//! batch.

use crate::{
    brain_bridge::ObservationBatch,
    prelude::{AgentIdComponent, AppState, AutonomyPipelineComponent, SimulationSet},
};

use helios_runtime::observe::{
    resolve::WatchResolver,
    sink::{ObservationSink, WatchRequest},
};

use bevy::prelude::*;

/// Every registered sink's request, joined into one.
///
/// Exists from the start, so with no sink registered it asks for nothing and
/// nothing is watched.
#[derive(Debug, Default, Resource)]
pub struct HostWatchRequest(WatchRequest);

/// A sink the host feeds every tick, one per sink type.
///
/// Held by its type, so whoever registered a sink reads it back with
/// `Res<RegisteredSink<TheirSink>>`.
#[derive(Resource)]
pub struct RegisteredSink<S>(pub S)
where
    S: ObservationSink + Send + Sync + 'static;

/// Registers observation sinks on an [`App`].
///
/// A method on `App` rather than a plugin: a plugin's `build` only borrows
/// the plugin, so it couldn't move an owned sink into the world.
pub trait ObservationSinkAppExt {
    /// Adds `sink`'s request to the host's, stores the sink, and feeds it
    /// every tick's batch. Call it from a plugin's `build`.
    ///
    /// # Panics
    ///
    /// If a sink of the same type is already registered: the second would
    /// replace the first while its request stayed in the host's.
    fn add_observation_sink<S>(&mut self, sink: S) -> &mut Self
    where
        S: ObservationSink + Send + Sync + 'static;
}

impl ObservationSinkAppExt for App {
    fn add_observation_sink<S>(&mut self, sink: S) -> &mut Self
    where
        S: ObservationSink + Send + Sync + 'static,
    {
        if self.world().contains_resource::<RegisteredSink<S>>() {
            panic!(
                "observation sink {} registered twice; a host holds one sink per type",
                std::any::type_name::<S>()
            );
        }

        // Asked before the sink moves into the world, and never again.
        let request = sink.request();

        // `init_resource` only inserts when missing, so this and
        // `BrainBridgePlugin` may run in either order.
        self.init_resource::<HostWatchRequest>();
        self.world_mut()
            .resource_mut::<HostWatchRequest>()
            .0
            .add(request);

        self.insert_resource(RegisteredSink(sink));

        // `Validation` runs after `BrainTick`, so the batch holds the tick
        // that just ran.
        self.add_systems(
            FixedUpdate,
            feed_sink::<S>
                .in_set(SimulationSet::Validation)
                .run_if(in_state(AppState::Running)),
        );

        self
    }
}

/// Hands the tick's batch to the sink of type `S`.
fn feed_sink<S>(batch: Res<ObservationBatch>, mut sink: ResMut<RegisteredSink<S>>)
where
    S: ObservationSink + Send + Sync + 'static,
{
    sink.0.receive(batch.observations());
}

/// Tells every agent's pipeline to record what the sinks requested.
///
/// Runs once at scene build, after the pipelines are spawned.
pub fn watch_requested_observables(
    request: Res<HostWatchRequest>,
    mut pipelines: Query<(&mut AutonomyPipelineComponent, &AgentIdComponent)>,
) {
    watch_requested(&request.0, &mut pipelines);
}

/// Resolves `request` against every agent and sets each pipeline's watch set.
///
/// Separate from the system so that a system run mid-run can watch a new
/// request the same way.
fn watch_requested(
    request: &WatchRequest,
    pipelines: &mut Query<(&mut AutonomyPipelineComponent, &AgentIdComponent)>,
) {
    let mut resolver = WatchResolver::new(request.clone());

    for (mut pipeline, agent) in pipelines.iter_mut() {
        // The set is owned, so the borrow `observables` takes ends here,
        // before `watch` needs the pipeline mutably.
        let set = resolver.watch_set_for(&agent.0, pipeline.0.observables());

        // Can't fail for a set resolved from the pipeline's own observables;
        // if it does, the pipeline keeps watching what it watched before.
        if let Err(errors) = pipeline.0.watch(&set) {
            for error in errors {
                warn!(
                    "Agent '{}' could not watch a requested observable: {}",
                    agent.0, error
                );
            }
        }
    }

    // A path no agent reports records nothing, so whatever reads it would
    // see no samples rather than an error.
    for path in resolver.into_unresolved() {
        warn!(
            "Requested observation path '{}' matches nothing any agent reports; \
             it will record nothing.",
            path
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::brain_bridge::drain_observations;

    use helios_core::{
        prelude::{AgentId, FrameId, MonotonicTime, TfProvider},
        spatial::transforms::ErasedTransform,
    };
    use helios_runtime::{
        observe::{observation::AgentObservation, path::agent_path},
        port::{Determinism, MockNodePortDescriptor, PortBus, PortDescriptor},
        PipelineBuilder, PipelineNode, TickContext,
    };

    use bevy::ecs::system::RunSystemOnce;

    const CAR: &str = "car";
    const ESTIMATOR: &str = "estimator";
    const NIS: &str = "aiding.gps.nis";
    const DROPPED: &str = "aiding.gps.dropped";
    const NOW: f64 = 1.0;
    const STEP: f64 = 0.01;

    /// A node that, each run, emits every leaf it declares, valued and
    /// stamped with the tick's `now`.
    struct Emitting {
        descriptor: PortDescriptor,
    }

    impl PipelineNode for Emitting {
        fn name(&self) -> &str {
            ESTIMATOR
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _tf: &dyn TfProvider, tick: TickContext) {
            for observable in self.descriptor.observables() {
                tick.emit(observable.leaf_name(), tick.now, tick.now.0);
            }
        }
    }

    struct NoTransforms;

    impl TfProvider for NoTransforms {
        fn get_transform(
            &self,
            _from: FrameId,
            _to: FrameId,
            _at: MonotonicTime,
        ) -> Option<ErasedTransform> {
            None
        }
    }

    /// Asks for `request` and keeps every batch it receives.
    struct Recording {
        request: WatchRequest,
        received: Vec<AgentObservation>,
    }

    impl Recording {
        fn asking_for(paths: &[String]) -> Self {
            Self {
                request: WatchRequest::Paths(paths.iter().cloned().collect()),
                received: Vec::new(),
            }
        }
    }

    impl ObservationSink for Recording {
        fn request(&self) -> WatchRequest {
            self.request.clone()
        }

        fn receive(&mut self, batch: &[AgentObservation]) {
            self.received.extend_from_slice(batch);
        }
    }

    /// An app with the request and batch the bridge plugin creates, and one
    /// agent whose estimator declares NIS and the drop count.
    fn app_with_car() -> App {
        let mut app = App::new();
        app.init_resource::<HostWatchRequest>()
            .init_resource::<ObservationBatch>();

        let descriptor = MockNodePortDescriptor::new()
            .observable(NIS, Determinism::Reproducible)
            .observable(DROPPED, Determinism::Reproducible)
            .build();
        let pipeline = PipelineBuilder::new()
            .add_node(Box::new(Emitting { descriptor }))
            .build()
            .expect("one node builds");
        app.world_mut().spawn((
            AutonomyPipelineComponent(pipeline),
            AgentIdComponent(AgentId::new(CAR)),
        ));
        app
    }

    /// Watches what was requested, then ticks every pipeline once and drains
    /// them, as scene build and one `FixedUpdate` would.
    fn watch_tick_and_drain(world: &mut World) {
        world
            .run_system_once(watch_requested_observables)
            .expect("watch_requested_observables runs");
        world
            .run_system_once(|pipelines: Query<&AutonomyPipelineComponent>| {
                for pipeline in &pipelines {
                    pipeline.0.tick(MonotonicTime(NOW), STEP, &NoTransforms);
                }
            })
            .expect("tick runs");
        world
            .run_system_once(drain_observations)
            .expect("drain_observations runs");
    }

    #[test]
    fn a_requested_path_reaches_the_sink_tagged_with_its_agent() {
        let mut app = app_with_car();
        app.add_observation_sink(Recording::asking_for(&[agent_path(CAR, ESTIMATOR, NIS)]));

        let world = app.world_mut();
        watch_tick_and_drain(world);
        world
            .run_system_once(feed_sink::<Recording>)
            .expect("feed_sink runs");

        let received = &world.resource::<RegisteredSink<Recording>>().0.received;
        let sources: Vec<(&str, &str, &str)> = received
            .iter()
            .map(|o| {
                (
                    o.agent.as_str(),
                    o.observation.node.as_ref(),
                    o.observation.leaf.as_ref(),
                )
            })
            .collect();
        assert_eq!(sources, [(CAR, ESTIMATOR, NIS)]);
    }

    #[test]
    fn with_no_sink_nothing_is_watched() {
        let mut app = app_with_car();

        let world = app.world_mut();
        watch_tick_and_drain(world);

        assert!(world
            .resource::<ObservationBatch>()
            .observations()
            .is_empty());
    }

    #[test]
    #[should_panic(expected = "registered twice")]
    fn registering_a_sink_type_twice_panics() {
        let mut app = App::new();
        app.add_observation_sink(Recording::asking_for(&[]))
            .add_observation_sink(Recording::asking_for(&[]));
    }
}
