# Helios Architecture Principles

> Architectural guidance for a modular robotics and autonomy simulation platform.
> Written for engineers who build and review code in this workspace.

---

## 1. Philosophy

Helios exists to close the gap between algorithm research and physical deployment. Every architectural decision serves one mandate: **code that runs in simulation must run on hardware without modification to the algorithm layer.**

Four pillars support this:

**Modularity over monoliths.** The workspace is split into crates with enforced, one-way dependencies (`helios_core` ← `helios_runtime` ← `helios_sim`). Each crate has a single reason to exist. `helios_core` is a math library. `helios_runtime` is a pipeline orchestrator. `helios_sim` is a Bevy host. A new host (`helios_hw`) plugs in at the same seam `helios_sim` occupies — it implements `AgentRuntime` and feeds the same `AutonomyPipeline`. This is not theoretical; the config system already separates portable agent profiles (`AgentBaseConfig`) from sim-specific spawn data (`AgentConfig`).

**Composability over configuration.** Algorithms are registered as factory closures in the `AutonomyRegistry`, keyed by string identifiers that appear in TOML. Spawning systems never name concrete types. Swapping an EKF for a UKF, or A\* for RRT\*, is a config change, not a code change. The `PipelineBuilder` assembles stages by `PipelineLevel` (Global → Local → Custom), so adding a new planning layer means registering a factory and adding a TOML block.

**Determinism where it matters.** `helios_core` is pure: no I/O, no system clock, no global state. Filters receive time through `FilterContext`, transforms through `TfProvider`, and control inputs through function arguments. Given the same inputs, an EKF produces the same outputs on any machine, in any host. This makes unit testing trivial and replay-based debugging possible.

**Sim-to-Real as a first-class constraint.** Every interface boundary is designed so that `helios_hw` can exist. `AgentRuntime` abstracts time and coordinate lookups. `AutonomyStack` configs live in `configs/catalog/agent_profiles/`, loadable by both sim and hardware hosts. The `EstimationDriver` / `MapDriver` traits allow swapping real estimation for ground-truth passthrough without touching the pipeline. If a design decision makes simulation easier but hardware harder, it's the wrong decision.

---

## 2. Core Architectural Characteristics

### Testability

**What it means here:** Every algorithm in `helios_core` can be tested with a deterministic state vector, a known Jacobian, and an asserted output — no ECS, no physics engine, no frame timing. Unit tests construct an `ExtendedKalmanFilter` with known initial state, feed it a `MeasurementMessage`, and assert the posterior covariance. Integration tests build an `AutonomyPipeline` via `PipelineBuilder`, feed it a sequence of measurements, and verify that estimated state converges.

**Why it matters:** Robotics bugs are expensive. A sign error in a Jacobian can send a vehicle off course. Catching these in a 10ms unit test instead of a 60-second simulation run is the difference between shipping and debugging.

**How it influences design:**
- `helios_core` has zero Bevy dependencies (the optional `bevy_ecs` feature exists solely for the `Entity` type in message routing, gated behind `#[cfg(feature = "bevy")]`).
- `helios_runtime` depends on `helios_core` *without* the `bevy` feature. Its tests exercise the full pipeline with mock `AgentRuntime` implementations.
- `SimulationProfile` and `CapabilitySet` let integration tests isolate subsystems — `EstimationOnly` runs sensors and filters without planning or control, `ControlOnly` feeds a static path and ground-truth state to verify controller convergence.

### Agility / Extensibility

**What it means here:** Adding a new algorithm — a particle filter, a lattice planner, a model-predictive controller — requires implementing one trait, registering one factory, and writing one TOML block. No existing code changes.

**Why it matters:** Autonomy R&D moves fast. If adding a new filter requires modifying the pipeline, the estimation system, and three spawning systems, researchers will work around the architecture instead of within it.

**How it influences design:**
- The `AutonomyRegistry` maps string keys to `Arc<dyn Fn(BuildContext) -> Result<Box<dyn Trait>>>` closures. Concrete types are invisible to the orchestration layer.
- `PipelineLevel` (Global, Local, Custom) allows hierarchical planning and mapping stages to be composed from config. A drone with both a global mission planner and a local obstacle avoider is a two-entry TOML list.
- New `MeasurementData` variants can be added to the enum without modifying existing `Measurement` implementations — each model's `get_measurement_vector` returns `None` for variants it doesn't handle.

### Deployability

**What it means here:** `cargo build --release` produces a single binary with all algorithms, configs, and assets embedded or referenced by path. No ROS master, no roslaunch, no catkin workspace, no docker-compose.

**Why it matters:** Dependency trees kill deployment velocity. A ROS2 workspace with 30 packages, each with its own CMakeLists.txt, takes longer to build and is harder to cross-compile for ARM targets than a single Cargo workspace.

**How it influences design:**
- The three-crate structure is a Cargo workspace, not three repositories. `cargo test --workspace` runs everything.
- All config lives in `configs/` at the workspace root, loaded via `--config-root` CLI argument. No environment variables, no ROS parameter server.
- `helios_hw` will be another binary target in the same workspace, sharing `helios_core` and `helios_runtime` as library dependencies.

### Performance

**What it means here:** The simulation must support real-time operation at 200+ Hz fixed timestep for control loops, with sub-millisecond overhead per agent for the estimation-control hot path.

**Why it matters:** If the simulation can't run faster than real-time, you can't do Monte Carlo testing. If the hot path allocates, you get GC-like latency spikes that mask real timing issues.

**How it influences design:**
- `helios_core` is allocation-free during steady-state operation. EKF/UKF work on fixed-size `nalgebra` matrices. The `dhat-heap` profiler confirms zero `helios_core` allocations during simulation.
- The Hot Path (Sensor → EKF → Control) uses zero-copy in-memory mechanisms: Bevy Events in sim, direct channels in hw. No serialization, no network I/O.
- `SimulationSet` ordering guarantees that estimation completes before planning reads state, and planning completes before control reads the path — no locks, no mutexes, just ECS schedule ordering.
- `TopicBus` uses ring buffers (`VecDeque<StampedMessage<T>>`) with fixed capacity — old messages are dropped, not reallocated.

### Scalability

**What it means here:** The architecture must support 100+ simultaneous agents without lock contention or linear-in-agent-count overhead in shared systems.

**Why it matters:** Swarm research, traffic simulation, and adversarial testing all require many agents. If agent count is bounded by a global mutex, the architecture fails these use cases.

**How it influences design:**
- Each agent owns its own `AutonomyPipeline` instance. There is no shared mutable state between agents' estimation or control systems.
- The `TfTree` is rebuilt per-tick in `SimulationSet::Precomputation` and read immutably by all subsequent systems. This is a single-writer-many-reader pattern with no runtime synchronization.
- `TopicBus` topics are per-agent (`/{agent}/sensors/{name}`), so publishing to one agent's IMU topic doesn't contend with another agent's GPS topic.
- Future multi-agent coordination (`helios_swarm`) will use `helios_communications` (Zenoh) for inter-agent messaging — the Cold Path, explicitly outside the per-agent hot loop.

### Interoperability

**What it means here:** The system can export real-time telemetry to standard robotics tooling (Foxglove, custom dashboards) and accept commands from external systems without modifying internal data structures.

**Why it matters:** Researchers need visualization. Operators need dashboards. Hardware teams need to bridge ROS bags for validation. A closed system that can't talk to anything is a toy.

**How it influences design:**
- `helios_communications` (Zenoh) handles the Cold Path: serializing `Odometry`, `PointCloud`, and other types for external consumption.
- The `TopicBus` serves as the internal boundary — systems publish to topics, and bridge plugins (e.g., the Foxglove WebSocket bridge) read from topics and forward externally.
- All core data structures implement `serde::Serialize` and `serde::Deserialize`, making them wire-format-agnostic.

---

## 3. Architecture Decision Principles

### 3.1 Coupling vs. Cohesion

**Tradeoff:** High cohesion within a crate means related logic lives together. Low coupling between crates means changes don't cascade.

**Direction for Helios:** Minimize coupling at crate boundaries. Maximize cohesion within crates. The three-crate contract is the primary coupling firewall.

**Concrete example:** `helios_core` defines `StateEstimator`, `EstimationDynamics`, and `Measurement` traits — all related to estimation — in the same crate. But `helios_sim`'s `AutonomyRegistry`, which instantiates these, lives in a different crate. The coupling between them is a single trait boundary (`Box<dyn StateEstimator>`). If the EKF's internal representation changes, only `helios_core` tests break. The registry doesn't care.

**Rule:** If two modules change for the same reason, they belong in the same crate. If they change for different reasons (algorithm math vs. ECS scheduling vs. physics integration), they belong in different crates.

### 3.2 Data Flow vs. Control Flow (Hot Path / Cold Path)

**Tradeoff:** Unified data flow is simpler to reason about but introduces latency. Separate paths are faster but harder to debug.

**Direction for Helios:** Two explicit paths. The Hot Path is in-memory, zero-copy, and synchronous within a tick. The Cold Path is asynchronous, serialized, and network-capable.

**Hot Path (in-memory, per-tick):**
```
Sensor System → MeasurementMessage → EstimationCore.process_measurement()
  → FrameAwareState → ControlCore.step_controllers() → ControlOutput
  → VehicleOutputAdapter.adapt() → VehicleCommand → Avian3D Forces
```
Every step in this chain is a direct function call or a Bevy Event read within a single `FixedUpdate` tick. No serialization. No channel. No heap allocation.

**Cold Path (networked, async):**
```
TopicBus.publish("/{agent}/odometry/estimated", odometry)
  → FoxgloveBridge reads topic → WebSocket → Foxglove Studio
  → helios_communications (Zenoh) → external subscriber
```
This path tolerates latency. Messages can be batched, dropped, or replayed.

**Rule:** Never put Zenoh, serde serialization, or any I/O in the Hot Path. If you need to observe Hot Path data externally, publish a copy to the Cold Path after the Hot Path completes (in `SimulationSet::Validation`).

### 3.3 State Management: Stateless Algorithms vs. Stateful ECS Components

**Tradeoff:** ECS components are great for spatial queries and parallel iteration. But putting algorithm state in ECS components couples the algorithm to the host.

**Direction for Helios:** Algorithm state lives in `AutonomyPipeline` (owned per-agent, stored as a Bevy Component in sim). ECS components hold only host-specific state (physics handles, render transforms, sensor config).

**Concrete example:** The `ExtendedKalmanFilter` owns its `FrameAwareState` (state vector + covariance). This is *not* an ECS component — it's a field inside `EstimationCore`, inside `AutonomyPipeline`. The Bevy system that ticks estimation calls `pipeline.process_measurement(msg, &runtime)` and reads the result. The ECS never directly touches the state vector.

Contrast this with physics state: `Transform`, `ExternalForce`, `LinearVelocity` are Avian3D components, managed by the physics engine. These are host-specific and belong in ECS.

**Rule:** If state is consumed by `helios_core` or `helios_runtime` algorithms, it lives in the pipeline. If state is consumed by the physics engine or renderer, it lives in ECS components.

### 3.4 Contract Design: Trait Bounds and the AgentRuntime Interface

**Tradeoff:** Rich trait interfaces are expressive but hard to implement for new hosts. Minimal interfaces are easy to implement but push complexity to the caller.

**Direction for Helios:** Minimal, host-agnostic trait surfaces. `AgentRuntime` has three methods: `now()`, `get_transform()`, `world_pose()`. That's it. A hardware implementation can satisfy this with a system clock and a static URDF lookup table. A simulation implementation wraps `TfTree` and `Time<Virtual>`.

**Concrete example:** `FilterContext` holds an `Option<&dyn TfProvider>`. Measurement models call `tf.get_transform(sensor_frame, body_frame)` to transform sensor readings into the body frame. The model doesn't know whether the transform came from a Bevy `TfTree`, a ROS TF2 listener, or a hardcoded calibration matrix. It doesn't care.

**Rule:** Traits in `helios_core` and `helios_runtime` must be implementable without any Bevy, Avian3D, or OS-specific dependency. If a trait method would require `bevy::ecs::World`, it belongs in `helios_sim`.

### 3.5 Abstraction Boundaries: The Coordinate Frame Law

**Tradeoff:** Allowing ad-hoc coordinate conversions is flexible but error-prone. Centralizing conversions adds indirection but eliminates an entire class of bugs.

**Direction for Helios:** All frame conversions happen at exactly one boundary, through typed newtypes and dedicated conversion functions.

**Concrete example:** `helios_core` operates in ENU (world) and FLU (body/sensor). `helios_sim` uses Bevy's Y-up/-Z-forward convention. The typed newtypes (`EnuWorldPose`, `FluLocalPose`, `EnuVector`, `FluVector`) in `helios_sim/src/simulation/core/transforms/` enforce conversions via `From` impls in `bevy_bridge.rs`. A developer cannot accidentally pass a Bevy `Transform` where an ENU pose is expected — the type system prevents it.

**Rule:** Never perform manual axis swaps (`v.y = physics.z`) anywhere in the codebase. If you need a new conversion, add a `From` impl in `bevy_bridge.rs`. If you find yourself swapping axes in a system function, you have a bug.

### 3.6 Evolutionary Architecture: The AutonomyRegistry as Extension Point

**Tradeoff:** Static dispatch (generics/monomorphization) is faster. Dynamic dispatch (`Box<dyn Trait>`) is flexible. A registry adds indirection but enables runtime composition.

**Direction for Helios:** Dynamic dispatch via the `AutonomyRegistry` for algorithm selection. Static dispatch within algorithm internals (e.g., nalgebra operations).

**Concrete example:** `AutonomyRegistry` stores factories as `Arc<dyn Fn(BuildContext) -> Result<Box<dyn Trait>>>`. When a scenario TOML says `dynamics.type = "IntegratedImu"`, the spawning system looks up `"IntegratedImu"` in `registry.dynamics`, calls the factory, and gets back a `Box<dyn EstimationDynamics>`. The spawning system has no `match` statement, no concrete type name, no `use helios_core::models::estimation::dynamics::integrated_imu::IntegratedImuModel`.

**Why this works:** The dynamic dispatch cost is paid once at spawn time (scene building). During simulation, the pipeline holds `Box<dyn StateEstimator>` and calls `predict`/`update` through vtable dispatch — a single indirect call per tick, invisible compared to the matrix math inside.

**Rule:** Never `match` on algorithm type strings in spawning systems. If you're writing `if dynamics_type == "IntegratedImu"`, you should be registering a factory instead.

### 3.7 Error Handling: Fail-Safe Degradation over Fail-Fast Panics

**Tradeoff:** Panicking on errors is explicit and easy to debug. Graceful degradation keeps the system running but can mask bugs.

**Direction for Helios:** Panic at startup on invalid configuration (catch errors early). Log and skip at runtime (never crash a running agent).

**Concrete example:** If `AutonomyRegistry` can't find a factory for a requested dynamics model during scene building, it logs `error!` and skips spawning that agent. The simulation continues with the remaining agents. This is intentional — in a 100-agent swarm test, one bad config shouldn't kill the entire run.

At runtime, if `s.try_inverse()` fails during an EKF update (singular innovation matrix), the update is skipped and covariance health is checked. The filter continues with the prior estimate. This is numerically correct — a skipped update is better than a NaN state.

**Rule:** `unwrap()` and `expect()` are forbidden in runtime code paths. They are acceptable in startup/config-loading code where a missing field genuinely means the configuration is invalid and the user needs to fix it.

### 3.8 The 3-Layer Actuation Model: Decoupling Abstraction Levels

**Tradeoff:** A single monolithic controller that outputs motor torques is simple but couples high-level planning to low-level vehicle dynamics. Layered actuation is more complex but allows swapping vehicles under the same planner.

**Direction for Helios:** Three explicit layers with clean interfaces between them.

```
Layer 1 (Core):    Controller.compute() → ControlOutput::BodyVelocity { linear, angular }
Layer 2 (Adapter): AckermannOutputAdapter.adapt() → VehicleCommand { throttle, steering_torque }
Layer 3 (Actuator): apply_ackermann_forces() → ExternalForce + ExternalTorque on Avian3D body
```

**Why three layers:** A Pure Pursuit controller outputs `ControlOutput::BodyVelocity` — an abstract command that says "go forward at 5 m/s, turn left at 0.3 rad/s." This same controller works for an Ackermann car, a differential-drive robot, and a drone (with different adapters). The adapter is "firmware" — it owns the PID loops that translate velocity commands into vehicle-specific actuator inputs. The actuator layer is "dumb" — it applies forces to the physics engine, no logic.

**Rule:** Controllers in `helios_core` must never reference vehicle geometry (wheelbase, rotor arm length). Adapters in `helios_sim` must never reference planning goals. If a controller needs to know the turning radius, the adapter should expose it as a constraint, not the controller should query vehicle parameters.

---

## 4. System Structure & Boundaries

### Dependency Graph

**Current workspace (3 crates):**

```
helios_core          (zero external runtime deps, pure math)
    ↑
helios_runtime       (depends on helios_core without bevy feature)
    ↑
helios_sim           (depends on helios_core with bevy feature + helios_runtime)
```

**Planned crates:**

```
helios_hw            (depends on helios_core + helios_runtime, no bevy)
helios_communications (depends on helios_core for message types)
helios_tools         (depends on helios_core for data structures)
helios_research      (depends on helios_sim or helios_hw as a test harness)
helios_ai            (depends on helios_core for trait impls)
helios_swarm         (depends on helios_runtime + helios_communications)
```

### Crate Responsibilities and Rules

#### helios_core

**Owns:** Trait definitions, data structures, algorithm implementations, math.

**Allowed:** nalgebra, serde, rand, num-traits, thiserror, dyn-clone. The optional `bevy_ecs` feature for `Entity` only.

**Forbidden:** Bevy app/render/schedule, Avian3D, tokio, std::time, std::fs, std::net, any I/O. No side effects. No global state. No `println!` outside tests.

**Test strategy:** Unit tests with deterministic inputs. Property-based tests for numerical stability. No integration tests — those live in `helios_runtime` and `helios_sim`.

#### helios_runtime

**Owns:** `AutonomyPipeline`, `PipelineBuilder`, `AgentRuntime` trait, config schemas (`AutonomyStack`, `AgentBaseConfig`, `EstimatorConfig`), pipeline validation, `EstimationDriver`/`MapDriver` polymorphism.

**Allowed:** helios_core (without `bevy` feature), nalgebra, serde, log, toml (for config parsing).

**Forbidden:** Bevy, Avian3D, any rendering or physics types. No `Entity`. No `Component`. No `System`. No `App`. If you're importing from `bevy::*`, you're in the wrong crate.

**Test strategy:** Integration tests that build pipelines from config, feed measurement sequences, and assert state convergence. Mock `AgentRuntime` implementations that return canned transforms and timestamps.

#### helios_sim

**Owns:** Bevy application, ECS plugins, physics integration, rendering, sensor simulation (raycasting, noise injection), `AutonomyRegistry`, scene building, `TfTree`, `TopicBus`, `SimRuntime`, `SimulationProfile`/`CapabilitySet`, coordinate frame conversions.

**Allowed:** Everything. This is the integration point.

**Forbidden:** Algorithm math. If a system function contains a Jacobian, a matrix inverse, or a dynamics equation, that code belongs in `helios_core`. Systems orchestrate; they don't compute.

**Test strategy:** Scenario-level integration tests. Headless runs with `--duration-secs` and assertion checks on final state. Profile-specific tests (e.g., `ControlOnly` profile to verify path tracking convergence).

#### helios_hw (planned)

**Owns:** Hardware `AgentRuntime` implementation, device drivers (IMU, GPS, motor controllers), real-time scheduling.

**Allowed:** helios_core, helios_runtime, Linux HAL libraries, async runtime (tokio).

**Forbidden:** Bevy, Avian3D, anything rendering-related.

**Key constraint:** Must implement `AgentRuntime` with real system clock (`std::time::Instant`) and real TF lookups (from URDF + encoder readings). The `AutonomyPipeline` it runs is identical to the one in `helios_sim`.

#### helios_communications (planned)

**Owns:** Zenoh session management, topic mapping (internal `TopicBus` names → Zenoh key expressions), serialization bridges, Foxglove WebSocket server.

**Allowed:** helios_core (for message types), zenoh, serde, tokio.

**Forbidden:** Bevy, helios_runtime internals. This crate reads from and writes to the `TopicBus` — it doesn't participate in the pipeline.

#### helios_tools (planned)

**Owns:** Offline log analysis (MCAP, Parquet), scenario validation, replay.

**Allowed:** helios_core, helios_runtime (for config parsing), file I/O.

**Forbidden:** Real-time constraints. This crate processes data after the fact.

#### helios_research

**Binary:** `helios_sim/src/bin/helios_research.rs` — the main simulation entry point. Used for headless profiling runs, scenario testing, and data collection.

**Future:** Monte Carlo harness, parameter sweep infrastructure, statistical analysis of pipeline outputs. Will become a standalone crate wrapping `helios_sim` headless runs and `helios_hw`.

#### helios_ai (planned)

**Owns:** ML/DL model inference, training pipeline integration.

**Key constraint:** AI models implement `helios_core` traits (`Planner`, `Controller`, `Mapper`). A neural network planner is just another `Box<dyn Planner>` registered in the `AutonomyRegistry`. The pipeline doesn't know or care that it's running inference instead of A\*.

#### helios_swarm (planned)

**Owns:** Multi-agent coordination protocols, consensus algorithms, task allocation.

**Key constraint:** Inter-agent communication goes through `helios_communications` (Zenoh), not through shared memory or ECS queries. Each agent runs its own `AutonomyPipeline`. Swarm coordination is an overlay, not a replacement for per-agent autonomy.

### Side-Effect Boundary Summary

| Crate | File I/O | Network I/O | System Clock | Heap Alloc (hot path) | Global Mutable State |
|-------|----------|-------------|-------------|----------------------|---------------------|
| helios_core | No | No | No | No | No |
| helios_runtime | No | No | No | Minimal | No |
| helios_sim | Yes (config, assets) | Yes (Foxglove WS) | Yes (Bevy Time) | Yes (scene build) | No (ECS Resources only) |
| helios_hw | Yes (device files) | Yes (Zenoh) | Yes (system clock) | Minimal | No |
| helios_communications | No | Yes (Zenoh) | Yes | Yes (serialization) | No |

---

## 5. Tradeoffs

### Performance vs. Modularity

**The tension:** Dynamic dispatch via `Box<dyn StateEstimator>` costs one vtable indirection per `predict`/`update` call. Monomorphization would eliminate this. But monomorphization requires the pipeline to be generic over the estimator type, which infects every caller with type parameters and makes the `AutonomyRegistry` impossible.

**Our choice:** Dynamic dispatch at the pipeline level, static dispatch within algorithms. The vtable cost of calling `estimator.predict()` is approximately 1-2 nanoseconds. The matrix multiplication inside `predict()` is 100-1000 nanoseconds. The indirection is noise. The registry's flexibility is not.

**Where we don't pay the cost:** Inside `ExtendedKalmanFilter`, all nalgebra operations use compile-time-sized types where possible (`Matrix6`, `Vector6`). The optimizer inlines and vectorizes these. We get the best of both worlds: flexible composition at the pipeline boundary, maximum performance inside the algorithm.

### Realism vs. Simulation Speed

**The tension:** Full multi-body tire physics with deformable contact patches is more realistic. Rigid-body dynamics with simplified force models run 100x faster.

**Our choice:** Avian3D rigid-body dynamics with force-based vehicle models. An Ackermann car applies `ExternalForce` and `ExternalTorque` proportional to throttle and steering — no tire slip model, no suspension simulation, no aerodynamics. This is sufficient for testing estimation, planning, and control algorithms. It is *not* sufficient for testing tire-wear strategies or suspension tuning.

**When to revisit:** If Helios is used for vehicle dynamics research (not just autonomy algorithm development), a higher-fidelity physics model would be warranted. This would likely mean replacing Avian3D with a specialized vehicle dynamics solver, but the three-layer actuation model means only Layer 3 changes.

### Opinionated Pipeline vs. Infinite DAG Flexibility

**The tension:** ROS lets you wire arbitrary nodes into arbitrary graphs. This is maximally flexible and maximally chaotic — debugging a 50-node ROS graph with circular dependencies is a known pain point. Helios enforces a fixed pipeline structure: Sense → Estimate → Map → Plan → Control → Actuate.

**Our choice:** The opinionated pipeline. `AutonomyPipeline` has exactly three sub-cores: `EstimationCore`, `MappingCore`, `ControlCore`. Stages within each core are ordered by `PipelineLevel` (Global → Local → Custom). You can add stages, but you can't reorder the top-level flow.

**What we give up:** You can't easily build a pipeline where control output feeds back into estimation within the same tick (e.g., for model-predictive estimation). You can't build circular dependencies. This is a feature, not a limitation — circular dependencies in autonomy pipelines are a source of subtle, timing-dependent bugs.

**Escape hatch:** The `Custom(String)` pipeline level and the `Raw`/`RawActuators` control output variants exist for cases that don't fit the standard pattern. If someone truly needs a non-standard pipeline topology, they can implement `EstimationDriver` or `MapDriver` with custom logic. But they should justify the deviation.

### Per-Agent Ownership vs. Shared State

**The tension:** Sharing a single occupancy grid across all agents is memory-efficient. Per-agent maps are wasteful but eliminate synchronization.

**Our choice:** Per-agent pipeline ownership. Each agent's `AutonomyPipeline` owns its own `EstimationCore`, `MappingCore`, and `ControlCore`. Maps are per-agent. State estimates are per-agent. There is no shared mutable state between agents during the hot path.

**What we give up:** If 100 agents all build the same global occupancy grid from the same sensor data, we store 100 copies. This is acceptable for current scale. At 10,000 agents, a shared read-only map layer (written once per tick, read by all agents) would be worth the synchronization complexity.

### TOML Configuration vs. Code Configuration

**The tension:** TOML is readable, diffable, and accessible to non-programmers. But it can't express conditional logic, loops, or type-safe validation at write time.

**Our choice:** TOML for all tunable parameters (noise values, update rates, model selections, sensor placements). Rust code for structural validation (`validate_autonomy_config` in `helios_runtime`). The config system uses `figment` for layered resolution and `serde` for deserialization, so invalid configs fail at load time with clear error messages.

**Hard rule:** No noise value, update rate, or model type string is ever hardcoded in Rust source. If it changes between runs, it's in TOML.

---

## 6. Evolution Strategy

### Phase 1: Single-Agent Simulation (Current)

The system supports one or more agents in a single Bevy simulation, each with its own `AutonomyPipeline`, driven by `SimRuntime`. All communication is in-process via `TopicBus` and Bevy Events.

**Stability criteria:** A single Ackermann car can drive a planned path with EKF-fused IMU+GPS, with less than 1m position error over 1km, in a headless 60-second run.

### Phase 2: Multi-Agent Simulation

Add `helios_communications` (Zenoh) as the Cold Path. Agents still run in the same Bevy process, but telemetry and inter-agent messages flow through Zenoh topics.

**Key change:** The `TopicBus` gains a Zenoh bridge plugin. `helios_swarm` adds coordination protocols (consensus, task allocation) that communicate through Zenoh key expressions. The Hot Path is untouched — each agent's `AutonomyPipeline` still runs in-process.

**Stability criteria:** 50 agents in a single simulation, each publishing odometry at 100 Hz to Zenoh, with Foxglove Studio receiving all streams in real-time without frame drops.

### Phase 3: Hardware Deployment

Add `helios_hw` as an alternative host. It implements `AgentRuntime` with real hardware drivers and system clock. The same `AgentBaseConfig` TOML that defines an agent's autonomy stack in simulation defines it on hardware.

**Key change:** `helios_hw` loads `configs/catalog/agent_profiles/` and builds an `AutonomyPipeline` via `PipelineBuilder`, identical to what `helios_sim` does. The difference is the `AgentRuntime` implementation: `SimRuntime` wraps `TfTree` + Bevy time; `HardwareRuntime` wraps URDF transforms + `std::time::Instant`.

**Stability criteria:** The same EKF config that converges in simulation converges on a Raspberry Pi with real IMU and GPS data, within 10% of the simulated error bounds.

### Phase 4: Distributed Simulation

Multiple Bevy instances on different machines, each running a subset of agents, communicating through Zenoh. This enables large-scale swarm simulation beyond what a single machine can handle.

**Key change:** The Zenoh bridge becomes bidirectional — agents in Process A publish sensor data that agents in Process B can subscribe to (e.g., for cooperative perception). The pipeline architecture doesn't change; only the transport layer scales out.

**Stability criteria:** 500 agents across 10 machines, with inter-agent message latency below 10ms on a local network.

### Phase 5: AI Integration

Add `helios_ai` for ML/DL-based algorithms. Neural network planners, learned controllers, and perception models implement `helios_core` traits and register in the `AutonomyRegistry` like any other algorithm.

**Key change:** `helios_ai` brings inference runtime dependencies (ONNX Runtime, TensorRT, or similar). These are optional — not everyone needs ML. The `AutonomyRegistry` doesn't care whether a `Box<dyn Planner>` runs A\* or a neural network.

**Key constraint for sim-to-real:** If a model trains on simulation data, the coordinate frames (ENU/FLU), units (SI), and data formats (`MeasurementMessage`, `PointCloud`) must match between training and deployment. This is why the Coordinate Frame Law and SI unit conventions are non-negotiable.

---

## 7. Anti-Goals

These are things this architecture deliberately does not optimize for. If a proposed change serves one of these goals at the expense of the core characteristics, reject it.

**"Using ECS systems for pure mathematics."**
Bevy systems orchestrate. They call `pipeline.process_measurement()` and read the result. They do not contain Jacobians, matrix inversions, quaternion operations, or dynamics equations. If math is in a system function, it's in the wrong place.

**"Hardcoding vehicle geometry in high-level planners."**
A planner should not know whether it's planning for an Ackermann car or a quadcopter. It receives a `MapData` and a `FrameAwareState` and produces a `Path`. Vehicle constraints (turning radius, max velocity) are expressed as planner parameters in TOML, not as hardcoded constants.

**"Rebuilding a monolithic ROS1-style node graph internally."**
The `AutonomyPipeline` is a fixed-structure pipeline, not a general-purpose computation graph. Stages execute in a defined order (Sense → Estimate → Map → Plan → Control → Actuate). There is no runtime graph reconfiguration, no message-passing between arbitrary nodes, no `roslaunch`-style XML wiring.

**"Running a network middleware in the high-frequency control loop."**
Zenoh, gRPC, WebSockets, and any other network protocol are Cold Path only. The Hot Path — from sensor reading to actuator command — is in-memory function calls within a single tick. Introducing network latency or serialization overhead in this path is a correctness bug, not just a performance issue. Estimation filters assume measurements arrive in causal order within a tick; network jitter violates this assumption.

**"Supporting arbitrary plugin architectures with uncontrolled loading order."**
Every system belongs to exactly one `SimulationSet` or `SceneBuildSet` variant. There is no `.before()` / `.after()` spaghetti. If a new system doesn't fit an existing set, the correct response is usually to rethink the system's responsibility, not to add ordering constraints.

**"Optimizing for minimal binary size."**
A single binary that includes all algorithms, all vehicle models, and all sensor simulations is acceptable. Helios is a research and development platform, not a microcontroller firmware. Binary size is not a constraint. Compile time is a secondary concern to correctness and modularity.

**"Making helios_core generic over float precision."**
All core algorithms use `f64`. Period. `f32` introduces numerical instability in EKF covariance updates, quaternion normalization, and integrator accumulation. The 2x memory cost is irrelevant for the state vector sizes we handle (typically 15-30 elements). If a future GPU-based particle filter needs `f32`, it uses `f32` internally and converts at the trait boundary.

**"Providing a GUI for configuration."**
Configuration is TOML files in `configs/`. They are version-controlled, diffable, and reviewable in pull requests. A GUI that generates TOML would be a `helios_tools` concern, not an architectural requirement.

---

## 8. Guiding Principles

A short list for code reviews and design discussions. If a change violates one of these, it needs explicit justification.

1. **Algorithms are pure functions of their inputs.** A filter's output depends on state, measurement, and time — not on which host is running it, what frame the rendering engine uses, or whether a network connection is active.

2. **The dependency arrow points one way: core ← runtime ← sim.** If you find yourself importing `helios_sim` from `helios_runtime`, or `helios_runtime` from `helios_core`, stop. Restructure.

3. **Frame conversions happen at the boundary, nowhere else.** Use the typed newtypes (`EnuWorldPose`, `FluLocalPose`, etc.) and `bevy_bridge.rs` `From` impls. If you're writing `y = z` or `-z = y` in a system function, you have a bug.

4. **Systems orchestrate; they don't compute.** A Bevy system should call a method on `AutonomyPipeline` or read a result from it. It should not contain a `for` loop over matrix elements.

5. **Config drives composition.** Adding a new algorithm means implementing a trait, registering a factory, and writing a TOML block. Zero changes to existing code.

6. **Hot Path stays hot.** No serialization, no I/O, no heap allocation in the Sensor → Estimate → Control path. If you need to observe it, publish a copy to the Cold Path after the fact.

7. **Fail safe at runtime, fail fast at startup.** Invalid config panics during scene building. Invalid sensor data is logged and skipped during simulation. A running pipeline never panics.

8. **Per-agent ownership, no shared mutable state.** Each agent's pipeline is independent. Cross-agent communication goes through the Cold Path (`helios_communications`).

9. **SI units always, degrees only in TOML.** Radians, meters, seconds, kilograms in all Rust code. Convert degrees to radians immediately on config load.

10. **Every system belongs to a set.** `SimulationSet` for runtime systems, `SceneBuildSet` for scene construction. No orphan systems. No `.before()` / `.after()` ordering — the set defines the order.

11. **No concrete types in spawning systems.** Spawning systems resolve algorithms through the `AutonomyRegistry`. If you're writing `IntegratedImuModel::new()` in a spawning system, use a factory instead.

12. **Test at the right layer.** Math correctness in `helios_core` unit tests. Pipeline integration in `helios_runtime` tests. End-to-end behavior in `helios_sim` scenario tests. Don't test Jacobians through a full Bevy simulation.
