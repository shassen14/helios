use helios_core::prelude::{AgentId, FrameId, SensorReading};
use helios_runtime::port::SensorChannel;
use helios_runtime::{pipeline::AutonomyPipeline, tf_service::TfService, BodyChannel, Provenance};

use bevy::prelude::{Component, Entity};

/// Holds the fully-assembled autonomy pipeline for an agent.
/// Written at spawn time by `spawn_autonomy_pipeline`. Ticked every frame
/// by `run_pipeline_tick` in `SimulationSet::BrainTick`.
#[derive(Component)]
pub struct AutonomyPipelineComponent(pub AutonomyPipeline);

/// Holds the agent's [`TfService`] — the drain-fold-expose unit that turns the
/// pipeline's dual-published transform edges into a queryable estimated tree.
///
/// Written at spawn time alongside [`AutonomyPipelineComponent`], and folded
/// once per tick by `run_pipeline_tick` (before the pipeline ticks) so nodes
/// query a frozen buffer. Its provider is the *estimated* chain, wired only from
/// edge channels — nodes can no longer reach the sim's truth tree, which is the
/// point of the split.
#[derive(Component)]
pub struct TfServiceComponent(pub TfService);

/// The agent's canonical coordinate-frame identity — the typed [`AgentId`] built
/// once from `agent_config.name()` (e.g. `"rover_1"`), *not* the composite `Name`
/// (`"rover_1/base_link"`). Stamped at shell-spawn time (`spawn_agent_shells`, in
/// `SceneBuildSet::CreateRequests`) so it is present before any frame producer
/// runs — the vehicle topology, every sensor spawner, the odom frame, and the
/// pipeline assembler all read *this* value rather than each re-deriving an
/// `AgentId` from a name string in scope.
///
/// Single-sourcing the scope this way is what makes the aiding-drop bug
/// unrepresentable: there is one `AgentId` on the entity, so no producer can
/// silently stamp a frame under a divergent identity (the class of failure that
/// once dropped a GPS aiding update and let the estimate drift — gh #65). The
/// `Name` label stays purely cosmetic.
///
/// Present on *every* agent from `CreateRequests` onward, so a query for
/// `(&AgentIdComponent, &AutonomyPipelineComponent)` still never sees a pipeline
/// without its identity, and a `PipelineBuildFailed` agent is still nameable in a
/// report.
///
/// This is also the external identity used in reports, assertion targets, and
/// (later) determinism hashing — distinct from the Bevy `Entity`, which is an
/// internal handle: generational, run-unstable, and meaningless on hardware.
#[derive(Component)]
pub struct AgentIdComponent(pub AgentId);

/// Records that this agent's autonomy pipeline could not be assembled, and why.
///
/// The counterpart to [`AutonomyPipelineComponent`]: exactly one of the two
/// lands on every agent. Without it, a failed build is invisible — the agent
/// simply lacks a pipeline, and every downstream system that queries for one
/// finds nothing and does nothing, so the run proceeds as if the agent had no
/// autonomy to begin with.
///
/// The host decides what a failure means. This crate only records it, because
/// the right response differs by host: a test harness fails the run, an
/// interactive session might want the scene anyway to inspect it. Recording
/// per agent rather than aborting on the first also lets a multi-agent
/// scenario report every broken stack in one pass.
#[derive(Component)]
pub struct PipelineBuildFailed {
    /// Every error `build_pipeline` reported, already rendered for display.
    pub errors: Vec<String>,
}

/// The channel a sensor publishes on, described as the body publishes it:
/// typed by the sensor's payload, from the installation the sensor belongs to,
/// simulated, and mounted at the sensor's frame.
///
/// Attached to every sensor entity at spawn, where the payload type, the
/// installation, the channel name from the fit and the sensor's frame are all
/// known. Read twice:
/// `spawn_autonomy_pipeline` collects each agent's into the body it builds the
/// pipeline against, and [`SensorPublisher`](super::SensorPublisher) writes to
/// its key. One description serves both, so the channel the build checked is
/// the channel written.
#[derive(Component, Clone)]
pub struct SensorPublishChannel(BodyChannel);

impl SensorPublishChannel {
    /// Channel `name` of installation `unit`, carrying batches of `P` readings
    /// as [`SensorPublisher`](super::SensorPublisher) writes them, measured in
    /// `mount`, the frame the sensor's entity tracks.
    pub fn new<P: Send + Sync + 'static>(name: &str, unit: &str, mount: FrameId) -> Self {
        Self(
            BodyChannel::new(
                SensorChannel::named::<Vec<SensorReading<P>>>(name),
                unit,
                Provenance::Simulated,
            )
            .with_mount(mount),
        )
    }

    pub fn description(&self) -> &BodyChannel {
        &self.0
    }
}

/// Declares the bus channel names the host writes this agent's mission goal to.
/// The host-input counterpart to [`SensorPublishChannel`]: that one names where a
/// sensor *writes* its readings; this one names where the host writes the goal —
/// the channels the agent's planners read their goal from.
///
/// Plural — and a `Vec` — because a stack may run several planners on distinct
/// goal channels; one authored or clicked goal fans out to all of them. The set
/// is deduplicated and in a stable order at spawn time, so two planners sharing
/// the default channel collapse to one entry. Stamped alongside
/// [`AutonomyPipelineComponent`] by `spawn_autonomy_pipeline`, and read by
/// `forward_goal_events` to publish the goal.
#[derive(Component)]
pub struct MissionGoalChannels(pub Vec<String>);

#[derive(Component)]
pub struct TeleopControlled;

/// Marker on a virtual "odom frame" entity that links it back to its agent.
///
/// The odom entity carries `TrackedFrame` + `Transform` + `GlobalTransform`.
/// Its `Transform` is updated every tick by `update_odom_frames` to match
/// the pipeline's current pose estimate.
#[derive(Component)]
pub struct OdomFrameOf(pub Entity);
