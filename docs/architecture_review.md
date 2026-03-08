# Helios Architecture Review

> Comprehensive systems-level analysis of the Helios robotics simulation platform.
> Covers structural weaknesses, scalability risks, ECS anti-patterns, recommended
> improvements, and a next-generation architecture for multi-host deployment
> (simulation, hardware, HIL, digital twin).

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Structural Weaknesses](#structural-weaknesses)
3. [ECS Anti-Patterns and Issues](#ecs-anti-patterns-and-issues)
4. [Scalability Risks](#scalability-risks)
5. [Module Boundary Problems](#module-boundary-problems)
6. [Missing Abstractions](#missing-abstractions)
7. [Recommended Improvements](#recommended-improvements)
8. [Next-Generation Architecture](#next-generation-architecture)
9. [Next Steps — Phased Roadmap](#next-steps--phased-roadmap)
10. [Long-Term Vision](#long-term-vision)

---

## System Overview

Helios is a modular robotics/autonomy simulation platform built in Rust. It is
split into two crates with a strict one-way dependency:

```
helios_core   (pure algorithms, no framework deps)
     ^
     |
helios_sim    (Bevy 0.16+ / Avian3D simulation host)
```

### helios_core

- Pure algorithmic library: EKF, UKF, PID, LQR, dynamics models, sensor models,
  mapping, SLAM trait stubs.
- Framework-agnostic. Could deploy to real hardware.
- Coordinate conventions: ENU world, FLU body. All values SI, f64.
- Key traits: `StateEstimator`, `EstimationDynamics`, `Measurement`, `Controller`,
  `Mapper`, `SlamSystem`.
- `FrameAwareState` provides a layout-indexed state vector with covariance.

### helios_sim

- Bevy ECS host wrapping core algorithms in components and systems.
- Avian3D rigid-body physics.
- TOML-driven configuration with a catalog/prefab resolution system.
- `AutonomyRegistry` maps config string keys to factory closures — spawning
  systems never name concrete algorithm types.
- `TopicBus` provides a generic pub/sub message bus.
- `TfTree` maintains a transform graph bridging Bevy transforms to ENU/FLU.
- Phased spawn pipeline (`SceneBuildSet`, 10 ordered phases) and a runtime
  data-flow schedule (`SimulationSet`, 7 ordered phases plus physics).

### Current Scale

- ~10,900 lines of Rust across both crates.
- 1 vehicle type (Ackermann), 4 sensor types (IMU, GPS, magnetometer, 2D LiDAR).
- 2 filter implementations (EKF, UKF), 3 controller types (PID, LQR, Feedforward PID).
- 1 mapper (occupancy grid), SLAM trait defined but no concrete implementation.
- Foxglove WebSocket bridge for scalar telemetry.

---

## Structural Weaknesses

### SW-1: TopicBus serializes all parallel systems

Every sensor, estimator, and debug system takes `ResMut<TopicBus>` or
`Res<TopicBus>`. In Bevy's scheduler, any `ResMut<T>` prevents all other
systems touching `T` from running concurrently.

Affected systems (all serialized against each other):

- `imu_sensor_system`
- `gps_sensor_system`
- `magnetometer_sensor_system`
- `ground_truth_publish_system`
- `tf_publish_system`
- `world_model_output_publisher`
- Foxglove bridge system

This is the single biggest scalability wall in the current architecture.

### SW-2: Dual messaging path — TopicBus AND Bevy Events

Sensor systems publish to **both** paths:

```rust
// In imu_sensor_system:
topic_bus.publish(&imu.topic_name, pure_message.clone());        // TopicBus path
measurement_writer.write(BevyMeasurementMessage(pure_message));  // Event path
```

- The estimator reads from `EventReader<BevyMeasurementMessage>`.
- Foxglove/debugging reads from `TopicBus`.
- Two independent data paths that can diverge.
- Every new sensor must remember to write to both.

### SW-3: SpawnAgentConfigRequest is a god component

`SpawnAgentConfigRequest(pub AgentConfig)` carries the entire agent config —
vehicle params, all sensors, full autonomy stack, poses — as one component.
Every spawn phase queries the same monolithic blob.

- No compile-time visibility into which phase consumed which data.
- `AgentConfig` is deep-cloned into every build context.
- After `cleanup_spawn_requests` removes it, nothing enforces that no system
  reads it afterward.

### SW-4: WorldModelComponent enum forces exhaustive matching

```rust
pub enum WorldModelComponent {
    Separate { estimator: Box<dyn StateEstimator>, mapper: Box<dyn Mapper> },
    CombinedSlam { system: Box<dyn SlamSystem> },
}
```

Every consumer (`controller_compute_system`, `update_odom_frames`,
`world_model_output_publisher`, debugging) must `match` both variants.
Adding a third variant requires modifying every consumer.

### SW-5: Manual axis swaps outside transforms.rs

`core/mod.rs` lines 32 and 39:

```rust
ground_truth.angular_velocity =
    Vector3::new(ang_vel.x as f64, -ang_vel.z as f64, ang_vel.y as f64);
```

These violate the coordinate-frame law. Should use `bevy_vector_to_enu_vector()`.

### SW-6: `.unwrap()` in EKF runtime code path

`ekf.rs` lines 141 and 148:

```rust
model.predict_measurement(&self.state, message, context.tf.unwrap())
```

Violates the no-unwrap rule. A missing TF provider crashes the simulation.

---

## ECS Anti-Patterns and Issues

### ECS-1: Commands used for per-tick component mutation

`controller_compute_system` inserts `ControlOutputComponent` via
`commands.entity(entity).insert(...)` every tick. This queues a deferred command
instead of mutating in-place. Same issue in `drive_ackermann_cars` with
`ExternalForce`/`ExternalTorque`.

**Fix:** Query for `&mut ControlOutputComponent` directly. Insert the component
once during scene building; mutate it in-place at runtime.

### ECS-2: No per-agent parallelism

`world_model_event_processor` collects all measurement events into a single
`Vec`, sorts globally by timestamp, then loops over agents. With N agents
producing M sensors at R Hz, this is N×M×R operations/sec in a single serial
system.

**Fix:** Group events by agent entity, then process each agent independently.
Future `par_iter_mut` over agents with zero API changes.

### ECS-3: Monolithic debugging plugin

`debugging/systems.rs` is 636 lines containing 8+ visualization systems, UI
code, keybinding handling, and a sensor data cache. Should be split into
per-concern files (pose gizmos, covariance ellipsoid, trail, occupancy grid,
TF frames, legend UI).

### ECS-4: TfTree rebuilt from scratch twice per tick

`tf_tree_builder_system` clears all maps and re-queries every `TrackedFrame`
entity. Runs in both `Precomputation` and `StateSync`. No change detection.

**Fix:** Use `Changed<GlobalTransform>` filter for incremental updates. Only
clear/rebuild on structural changes (entity spawn/despawn).

---

## Scalability Risks

### SR-1: TopicBus contention (see SW-1)

At 100 agents × 5 sensors × 100 Hz = 50,000 messages/sec, serial TopicBus
access becomes the dominant bottleneck.

### SR-2: O(n) measurement model iteration in EKF

`ekf.rs` line 134: `for model in self.measurement_models.values()` iterates all
registered models on every measurement. The message already carries
`sensor_handle` — should be an O(1) HashMap lookup.

### SR-3: SceneBuildSet doesn't scale to 100+ component types

The 10-phase manual chain works when you can reason about ordering in your head.
At 100 component types with cross-dependencies, it becomes unmanageable.

### SR-4: No process-level experiment parallelism

For large experiments (100 runs of the same scenario with different seeds), the
right answer is process-level parallelism. Each run is an independent OS process.
This requires a robust MCAP/logging pipeline — currently absent.

---

## Module Boundary Problems

### MB-1: FrameHandle encodes Bevy Entity bits

`FrameHandle(pub u64)` stores `Entity::to_bits()` via `from_entity()`. This
couples `helios_core` message types to Bevy's entity representation, preventing
deployment on hardware where no Bevy entities exist.

### MB-2: Autonomy orchestration lives in Bevy systems, not in core

The predict → update → control sequencing is implemented in
`world_model_event_processor` (a Bevy system), not as a reusable algorithm
pipeline. Deploying the same stack on hardware requires reimplementing this
orchestration.

### MB-3: FilterContext is simulation-shaped

`FilterContext { tf: Option<&dyn TfProvider> }` is constructed from Bevy's
`TfTree`. On hardware, TF data comes from URDF + sensor calibration, not from
an ECS resource. The calling convention needs to be host-agnostic.

---

## Missing Abstractions

### MA-1: Agent Runtime Contract

No formal interface between algorithms and their host. The same autonomy stack
should run identically in simulation and on hardware. Requires:

- `SensorId` (host-agnostic, not Bevy `Entity` bits)
- `MonotonicTime` (clock-source-agnostic)
- `AgentRuntime` trait (TF lookups, time)
- `AutonomyPipeline` (self-contained predict → update → plan → control)

### MA-2: PoseSource abstraction

Entities get their pose from different sources (physics, estimator, network
mirror, log replay). Currently hardcoded to Avian3D physics. Needed for digital
twin and HIL.

### MA-3: Inter-agent communication

No mechanism for agents to exchange state estimates, map fragments, or plan
intents. Required for multi-robot coordination.

### MA-4: Telemetry decimation

No rate-limiting on TopicBus publishing. Every sensor fires every tick. For
external consumers (Foxglove, loggers), most of this data is redundant.

### MA-5: Filter health diagnostics

`StateEstimator::predict/update` are infallible — no `Result` return, no health
metrics. No way to detect filter divergence, NIS/NES violations, or covariance
blow-up.

---

## Recommended Improvements

### Hot-path / cold-path separation

**Hot path** (per-tick, performance-critical):

```
Sensors  -->  BevyEvent<MeasurementMessage>  -->  Estimator
         -->  EstimatedState component       -->  Controller
         -->  ControlOutput component        -->  Actuator
```

Use Bevy events and component mutation. No TopicBus. Full parallel scheduling.

**Cold path** (telemetry, logging, visualization):

```
Components  -->  telemetry_flush_system  -->  TopicBus  -->  Foxglove / MCAP
```

A single system publishes to TopicBus at configurable rates. No `ResMut<TopicBus>`
in the hot path.

### Component composition over enum dispatch

```rust
// Replace WorldModelComponent enum with independent components:
#[derive(Component)]
pub struct EstimatorComponent(pub Box<dyn StateEstimator>);

#[derive(Component)]
pub struct MapperComponent(pub Box<dyn Mapper>);

#[derive(Component)]
pub struct SlamComponent(pub Box<dyn SlamSystem>);
```

Systems query only what they need. Adding a mapper-only agent requires zero code
changes in unrelated systems.

### Direct sensor routing in EKF

```rust
fn update(&mut self, message: &MeasurementMessage, context: &FilterContext) {
    let Some(model) = self.measurement_models.get(&message.sensor_handle) else {
        return;
    };
    // O(1) dispatch instead of O(n) iteration
}
```

### Per-agent message batching

```rust
fn estimation_system(
    mut agents: Query<(Entity, &mut EstimatorComponent, &mut SensorMailbox)>,
    mut events: EventReader<BevyMeasurementMessage>,
) {
    // Group messages by agent
    let mut per_agent: HashMap<Entity, Vec<_>> = HashMap::new();
    for msg in events.read() {
        per_agent.entry(msg.0.agent_handle.to_entity())
            .or_default()
            .push(msg.0.clone());
    }

    // Process each agent independently (future par_iter candidate)
    for (entity, mut estimator, mut mailbox) in &mut agents {
        let Some(messages) = per_agent.remove(&entity) else { continue };
        for msg in messages { /* predict + update */ }
    }
}
```

### Phase-typed spawn requests

```rust
// Each spawn phase has its own request component
#[derive(Component)] pub struct VehicleSpawnRequest(pub VehicleConfig);
#[derive(Component)] pub struct SensorSpawnRequest(pub HashMap<String, SensorConfig>);
#[derive(Component)] pub struct AutonomySpawnRequest(pub AutonomyStack);

// Each phase consumes and removes its own request
// Compile-time: systems can only access what they're meant to process
```

---

## Next-Generation Architecture

### Crate Dependency Graph

```
helios_core            pure math, traits, data structures
     ^
     |
helios_runtime         agent runtime contract (NEW)
     ^            ^
     |            |
helios_sim        helios_hw
(Bevy/Avian3D)    (tokio + HW drivers)
```

### helios_runtime — The Key Missing Crate

A small (~500 lines), load-bearing crate that defines how an algorithm stack
receives data and produces outputs, independent of the host:

```rust
// helios_runtime/src/lib.rs

/// Host-agnostic sensor identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SensorId(pub u64);

/// Clock-source-agnostic timestamp.
#[derive(Debug, Clone, Copy, PartialOrd, PartialEq)]
pub struct MonotonicTime(pub f64);

/// A timestamped sensor reading from any source.
pub struct SensorReading {
    pub sensor_id: SensorId,
    pub timestamp: MonotonicTime,
    pub data: MeasurementData,
}

/// The contract any host must fulfill to run an autonomy stack.
pub trait AgentRuntime {
    fn get_transform(&self, from: SensorId, to: SensorId) -> Option<Isometry3<f64>>;
    fn now(&self) -> MonotonicTime;
}

/// A self-contained autonomy pipeline. Runs identically in sim and on hardware.
pub struct AutonomyPipeline {
    estimator: Option<Box<dyn StateEstimator>>,
    mapper: Option<Box<dyn Mapper>>,
    planner: Option<Box<dyn Planner>>,
    controller: Option<Box<dyn Controller>>,
}

impl AutonomyPipeline {
    pub fn process_measurement(
        &mut self,
        reading: &SensorReading,
        runtime: &dyn AgentRuntime,
    ) -> Option<ControlOutput> {
        // predict -> update -> plan -> control
    }
}
```

### Host implementations

**helios_sim** (Bevy):

```rust
struct SimRuntime<'a> {
    tf: &'a TfTree,
    time: &'a Time,
    sensor_map: &'a SensorIdMap,
}

impl AgentRuntime for SimRuntime<'_> {
    fn get_transform(&self, from: SensorId, to: SensorId) -> Option<Isometry3<f64>> {
        let from_entity = self.sensor_map.to_entity(from)?;
        let to_entity = self.sensor_map.to_entity(to)?;
        self.tf.get_transform_between(from_entity, to_entity)
    }
    fn now(&self) -> MonotonicTime {
        MonotonicTime(self.time.elapsed_secs_f64())
    }
}
```

**helios_hw** (hardware):

```rust
async fn agent_loop(
    mut pipeline: AutonomyPipeline,
    mut sensor_rx: mpsc::Receiver<SensorReading>,
    runtime: HardwareRuntime,
    actuator_tx: mpsc::Sender<ControlOutput>,
) {
    while let Some(reading) = sensor_rx.recv().await {
        if let Some(output) = pipeline.process_measurement(&reading, &runtime) {
            actuator_tx.send(output).await.ok();
        }
    }
}
```

### Inter-agent communication

```rust
pub struct AgentMessage {
    pub from: AgentId,
    pub to: AgentId,       // or Broadcast
    pub payload: AgentPayload,
    pub timestamp: MonotonicTime,
}

pub enum AgentPayload {
    PoseEstimate(Odometry),
    MapFragment(MapData),
    PlanIntent(PathSegment),
    Custom(Vec<u8>),
}
```

- In `helios_sim`: instant delivery (or configurable latency/drop model).
- In `helios_hw`: UDP multicast or DDS.

### Digital twin / HIL

A single Bevy instance runs the simulation with N virtual agents while
simultaneously receiving real sensor data from M physical robots:

```rust
#[derive(Component)]
pub enum PoseSource {
    Physics,                              // driven by Avian3D
    Estimator,                            // driven by autonomy pipeline
    NetworkMirror { addr: SocketAddr },   // driven by remote robot
    Replay { log: PathBuf },              // driven by MCAP playback
}
```

All downstream systems (visualization, debugging, Foxglove) are agnostic to the
source.

### Proposed module layout

```
helios_core/
  estimation/
    traits.rs                 StateEstimator, FilterHealth
    filters/                  ekf.rs, ukf.rs, particle.rs
  control/
    traits.rs                 Controller, ControlOutput, ControlContext
    pid.rs, lqr.rs, feedforward_pid.rs
  planning/
    traits.rs                 Planner, PathSegment, PlanResult
    a_star.rs, rrt_star.rs
  mapping/
    traits.rs                 Mapper, MapData
    occupancy_grid.rs
  models/
    dynamics/                 EstimationDynamics implementations
    measurement/              Measurement implementations
    perception/               Raycasting sensor models
  frames/                     FrameAwareState, StateVariable
  messages.rs                 MeasurementMessage, Odometry, PointCloud
  types.rs                    SensorId, TfProvider, State, Control

helios_runtime/
  lib.rs                      AgentRuntime, AutonomyPipeline
  sensor.rs                   SensorId, SensorReading, MonotonicTime
  comms.rs                    AgentMessage, AgentPayload

helios_sim/
  config/
    catalog.rs                Prefab resolution
    resolver.rs               TOML loading + validation
    schema/                   One file per config struct type
  core/
    agent.rs                  AgentId, AgentBundle, SensorIdMap
    events.rs                 BevyMeasurementMessage
    scheduling.rs             SimulationSet, SceneBuildSet, schedule config
    transforms/
      conversions.rs          ENU<->Bevy, FLU<->Bevy (pure functions)
      tf_tree.rs              TfTree resource + TfProvider impl
      systems.rs              tf_tree_builder, build_static_maps
    components.rs             GroundTruthState, PoseSource
  plugins/
    sensors/                  One plugin per sensor type
    vehicles/                 One plugin per vehicle type
    estimation/               EstimationPlugin (owns EstimatorComponent)
    mapping/                  MappingPlugin (owns MapperComponent)
    control/                  ControlPlugin
    planning/                 PlanningPlugin
    telemetry/
      topic_bus.rs            TopicBus (cold-path only)
      foxglove/               WebSocket bridge
      metrics.rs              Filter health, timing
    visualization/
      gizmos/                 Per-concern: pose.rs, covariance.rs, trail.rs
      ui/                     Legend, HUD
    world/                    Environment spawning
  registry/                   AutonomyRegistry (unchanged pattern)

helios_hw/
  runtime.rs                  HardwareRuntime impl
  drivers/                    Sensor driver async tasks
  actuators/                  Actuator output tasks
  comms/                      Network transport for AgentMessage
```

### Runtime data-flow schedule

```
FixedUpdate (per tick):

  Precomputation
  |  tf_tree_incremental_update (Changed<GlobalTransform>)
  |
  +---> Sensors          Perception
  |     (parallel)       (parallel)
  |
  +---> Estimation       WorldModeling
  |     (parallel,       (parallel,
  |      separate        separate
  |      components)     components)
  |
  +---> Planning
  |
  +---> Control
  |
  +---> Actuation
  |
  +---> [Avian3D Physics]
  |
  +---> StateSync
  |
  +---> TelemetryPublish  (cold-path TopicBus writes)
```

---

## Next Steps — Phased Roadmap

### Phase 1: Fix foundations ✅ DONE (2026-03-08)

| Item                                                           | Effort | Impact      |
| -------------------------------------------------------------- | ------ | ----------- |
| Fix `.unwrap()` in EKF update (crash risk)                     | 30 min | Safety      ✅ |
| Fix manual axis swaps in `ground_truth_sync_system`            | 15 min | Correctness ✅ |
| Route EKF updates by `sensor_handle` O(1) lookup               | 1 hr   | Performance ✅ |
| Use `&mut ControlOutputComponent` instead of `commands.insert` | 30 min | Performance ✅ |

### Phase 2: Introduce helios_runtime (next milestone)

| Item                                                                           | Effort | Impact       |
| ------------------------------------------------------------------------------ | ------ | ------------ |
| Create `helios_runtime` crate with `SensorId`, `MonotonicTime`, `AgentRuntime` | 2 days | Architecture |
| Define `AutonomyPipeline` — move predict/update/control sequencing out of Bevy | 2 days | Portability  |
| Migrate `FrameHandle` to `SensorId` in core algorithm interfaces               | 1 day  | Decoupling   |
| `helios_sim` implements `AgentRuntime` as adapter over `TfTree` + `Time`       | 1 day  | Integration  |

### Phase 3: Agent-local data isolation

| Item                                                                           | Effort | Impact        |
| ------------------------------------------------------------------------------ | ------ | ------------- |
| Per-entity `SensorMailbox` replaces `ResMut<TopicBus>` in sensor systems       | 2 days | Scalability   |
| TopicBus becomes telemetry-only, single `telemetry_flush_system`               | 1 day  | Parallelism   |
| Split `WorldModelComponent` enum into `EstimatorComponent` + `MapperComponent` | 4 hrs  | Extensibility |
| Per-agent message batching in estimation system                                | 4 hrs  | Multi-agent   |

### Phase 4: helios_hw skeleton

| Item                                                              | Effort | Impact   |
| ----------------------------------------------------------------- | ------ | -------- |
| Create `helios_hw` crate with tokio-based agent event loop        | 2 days | Hardware |
| Implement `HardwareRuntime` (static TF, system clock)             | 1 day  | Hardware |
| Sensor driver framework (async tasks feeding `SensorReading`)     | 2 days | Hardware |
| Actuator output framework (async tasks consuming `ControlOutput`) | 1 day  | Hardware |

### Phase 5: Multi-agent and digital twin

| Item                                                     | Effort | Impact       |
| -------------------------------------------------------- | ------ | ------------ |
| `AgentMessage` pub/sub in `helios_runtime`               | 1 day  | Multi-robot  |
| Sim implementation (instant or latency-delayed delivery) | 1 day  | Simulation   |
| HW implementation (UDP multicast or DDS)                 | 2 days | Hardware     |
| `PoseSource` component + mirror entities in Bevy         | 2 days | Digital twin |
| MCAP logging plugin for offline analysis                 | 3 days | Experiments  |

---

## Long-Term Vision

### The Platform Triangle

```
            helios_core
           /     |     \
          /      |      \
   helios_sim  helios_hw  helios_runtime
      |           |            |
   Research    Deployment   Portability
   + Viz       + Real HW    + Shared
   + Physics   + Network    + Pipeline
```

### Design principles

- **Same algorithm, any host.** An `AutonomyPipeline` configured with a
  particular EKF, dynamics model, and controller runs identically in simulation
  and on hardware. Tuning parameters transfer directly.

- **Component composition over enum dispatch.** Every autonomy module is its
  own ECS component. Systems query only what they need. New module types require
  zero changes to unrelated systems.

- **Events for data flow, components for state.** Measurements flow via Bevy
  events. Estimated state, control output, and ground truth are components
  mutated in-place. No deferred commands in the hot path.

- **TopicBus is telemetry, not architecture.** Nothing in the control loop
  reads from TopicBus. It exists solely for Foxglove, MCAP logging, and
  metrics dashboards.

- **Agent-local data isolation.** Each agent's autonomy state is a
  self-contained component set. Systems can `par_iter` across agents with no
  shared mutable state.

- **Process-level experiment parallelism.** Large-scale experiments (Monte
  Carlo, hyperparameter sweeps) run as independent OS processes. A thin
  orchestration layer launches runs, collects MCAP logs, and aggregates results.

### Success criteria

The architecture is successful when:

1. Adding a new sensor type requires: one `helios_core` model file, one
   `helios_sim` plugin file, one TOML config entry, zero changes elsewhere.
2. The same EKF + PID stack can be tested in simulation, deployed on a
   Raspberry Pi, and visualized as a digital twin — with no code changes.
3. A 100-agent simulation runs at real-time with sub-millisecond per-tick
   overhead from the autonomy pipeline.
4. A new team member can add a particle filter without understanding the
   Bevy scheduler, TopicBus internals, or coordinate frame math.
