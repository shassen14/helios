use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

use helios_core::{
    frames::FrameAwareState,
    prelude::{ControlOutput, PlannerGoal},
};

use crate::{
    pipeline::rate_gate::RateTimer,
    port::{ChannelKey, PortBus},
    prelude::{
        AgentRuntime, Health, NodeId, PipelineBuildError, PipelineNode, Stamped, TickContext,
    },
};

// Constant Sentinal Values
pub const MISSION_GOAL_INSTANCE: &str = "mission";
pub const HOST_PRODUCER_ID: NodeId = NodeId::MAX;

pub struct NodePipelineBuilder {
    nodes: Vec<Box<dyn PipelineNode>>,
    sensor_signal_keys: Vec<ChannelKey>,
    host_state_keys: Vec<ChannelKey>,
}

impl Default for NodePipelineBuilder {
    fn default() -> Self {
        NodePipelineBuilder::new()
    }
}

impl NodePipelineBuilder {
    pub fn new() -> Self {
        NodePipelineBuilder {
            nodes: vec![],
            sensor_signal_keys: vec![],
            host_state_keys: vec![],
        }
    }

    pub fn add_node(mut self, node: Box<dyn PipelineNode>) -> Self {
        self.nodes.push(node);
        self
    }

    pub fn with_sensor_signals(mut self, keys: Vec<ChannelKey>) -> Self {
        self.sensor_signal_keys = keys;
        self
    }

    pub fn with_host_states(mut self, keys: Vec<ChannelKey>) -> Self {
        self.host_state_keys = keys;
        self
    }

    pub fn build(self) -> Result<NodePipeline, Vec<PipelineBuildError>> {
        let mut errors: Vec<PipelineBuildError> = vec![];
        let mut producer_of: HashMap<ChannelKey, String> = HashMap::new();

        for node in &self.nodes {
            // get the outputs to check if there are multiple node producers
            let outputs = &node.port_descriptor().outputs;
            let node_name = node.name();

            // each output has the same node name
            for output in outputs {
                // if the channel_key gets updated, then we have multiple producers
                // and we collect the error to relay to the user
                if let Some(prev_name) = producer_of.insert(output.clone(), node_name.to_string()) {
                    let error = PipelineBuildError::MultipleProducers {
                        channel: output.clone(),
                        first_node: prev_name,
                        second_node: node_name.to_string(),
                    };
                    errors.push(error);
                }
            }
        }

        // Filling out the produced channels which require no input
        // which should be just host/sensor signals
        let mut produced: HashSet<ChannelKey> = HashSet::new();
        produced.extend(self.sensor_signal_keys.iter().cloned());
        produced.extend(self.host_state_keys.iter().cloned());

        let mut remaining: Vec<Box<dyn PipelineNode>> = self.nodes;
        let mut levels: Vec<Vec<(NodeId, Box<dyn PipelineNode>)>> = Vec::new();
        let mut next_id: NodeId = 0;

        while !remaining.is_empty() {
            let (ready, still_waiting): (Vec<_>, Vec<_>) =
                remaining.into_iter().partition(|node| {
                    node.port_descriptor()
                        .required_inputs
                        .iter()
                        .all(|channel| produced.contains(channel))
                });

            remaining = still_waiting;

            // if ready is empty, the graph cannot advance further
            // since the remaining nodes don't have their inputs satisfied
            // or a cycle has occurred
            if ready.is_empty() {
                // build the outputs as though we continued creating the graph
                let mut pending_outputs: HashSet<ChannelKey> = HashSet::new();
                for node in &remaining {
                    for channel in &node.port_descriptor().outputs {
                        pending_outputs.insert(channel.clone());
                    }
                }

                // Are there any remaining node whose all required input is accounted
                // for whether it's in already in the produces bus or from another
                // remaining node. If so, there is a cycle because all inputs are
                // given but we cannot progress further
                let is_cycle_detected = remaining.iter().any(|node| {
                    node.port_descriptor()
                        .required_inputs
                        .iter()
                        .all(|channel| {
                            produced.contains(channel) || pending_outputs.contains(channel)
                        })
                });

                if is_cycle_detected {
                    errors.push(PipelineBuildError::Cycle);
                }

                // if nothing produced the channel for required_inputs, then
                // we have a missing input
                for node in &remaining {
                    for channel in &node.port_descriptor().required_inputs {
                        if !produced.contains(channel) && !pending_outputs.contains(channel) {
                            let error = PipelineBuildError::UnsatisfiedInput {
                                node_name: node.name().to_string(),
                                channel: channel.clone(),
                            };
                            errors.push(error);
                        }
                    }
                }

                break;
            }

            // add each node's outputs as part of the produced data
            // we abstract each node which has a vec of outputs and flatten them to 1D
            produced.extend(
                ready
                    .iter()
                    .flat_map(|node| node.port_descriptor().outputs.iter())
                    .cloned(),
            );

            // create the level and push the ready nodes to this level
            let mut level: Vec<(NodeId, Box<dyn PipelineNode>)> = Vec::with_capacity(ready.len());
            for node in ready {
                level.push((next_id, node));
                next_id += 1;
            }

            // push the level to the levels graph
            levels.push(level);
        }

        if !errors.is_empty() {
            return Err(errors);
        }

        // now we have a guaranteed DAG, may not necessarily
        // be completely useful in terms of autonomy but we
        // know that there is no config time errors
        let descriptor_iter = levels
            .iter()
            .flat_map(|level| level.iter().map(|(_, node)| node.port_descriptor()));

        let signals = self.sensor_signal_keys;

        // create bus with descriptors and signals
        let bus = PortBus::new(descriptor_iter, signals);

        let mut rate_timers: Vec<RateTimer> = Vec::with_capacity(next_id as usize);

        for level in &levels {
            for (_, node) in level {
                rate_timers.push(RateTimer::new(node.port_descriptor().rate));
            }
        }

        Ok(NodePipeline {
            levels,
            bus,
            rate_timers,
        })
    }
}

pub struct NodePipeline {
    levels: Vec<Vec<(NodeId, Box<dyn PipelineNode>)>>,
    bus: PortBus,
    rate_timers: Vec<RateTimer>,
}

impl NodePipeline {
    pub fn bus(&self) -> &PortBus {
        &self.bus
    }

    pub fn tick(&self, runtime: &dyn AgentRuntime, dt: f64) {
        let now = runtime.now();
        self.bus.set_tick_time(now.0);

        // level by level, execute each node if the it meets the rate
        for level in &self.levels {
            for (node_id, node) in level {
                if self.rate_timers[*node_id as usize].should_fire_and_advance(dt) {
                    node.execute(
                        &self.bus,
                        runtime,
                        TickContext {
                            now,
                            dt,
                            node_id: *node_id,
                        },
                    );
                }
            }
        }
    }

    pub fn inject_mission_goal(&self, goal: PlannerGoal, runtime: &dyn AgentRuntime) {
        let now = runtime.now();
        let goal_stamped = Stamped {
            value: goal,
            timestamp: now,
            health: Health::Ok,
            producer: HOST_PRODUCER_ID,
        };

        let _ = self.bus.write(
            ChannelKey::named::<PlannerGoal>(MISSION_GOAL_INSTANCE),
            goal_stamped,
        );
    }

    pub fn read_state(&self) -> Option<Arc<Stamped<FrameAwareState>>> {
        self.bus.read(ChannelKey::of::<FrameAwareState>())
    }

    pub fn read_control(&self) -> Option<Arc<Stamped<ControlOutput>>> {
        self.bus.read(ChannelKey::of::<ControlOutput>())
    }
}
