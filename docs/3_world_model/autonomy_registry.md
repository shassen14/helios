# AutonomyRegistry

**Location:** `helios_sim/src/simulation/registry/`

The `AutonomyRegistry` maps config-string keys to factory closures. Spawning systems
never name concrete types — they call `registry.build_*(key, ctx)` and receive a
`Box<dyn Trait>`. Adding an algorithm requires zero changes to spawning systems.

---

## Factory Return Type: `Result<Box<T>, String>`

Every factory returns `Result`, not `Option`. Two distinct failures are surfaced:

| Failure | Where it originates | Log level |
|---|---|---|
| Unregistered key | `build_*` method (`HashMap::get` miss) | `error!` in spawning system |
| Factory prerequisite missing | Inside the factory closure | `error!` in spawning system |

A bad TOML config produces a clear `error!` at startup, not a silent no-op.

**Estimator failure** → `error!` + `continue` (agent gets no world model).
**Mapper failure** → `error!` + fallback to `NoneMapper` (estimation still runs).
**SLAM failure** → `error!` (no component inserted).

---

## Registering a New Algorithm

### 1. Implement the trait in `helios_core`

```rust
// helios_core/src/estimation/filters/my_filter.rs
pub struct MyFilter { ... }
impl StateEstimator for MyFilter { ... }
```

### 2. Add a factory in the appropriate registry file

| Algorithm type | Registry file | Trait |
|---|---|---|
| State estimator | `registry/estimators.rs` | `StateEstimator` |
| Dynamics model | `registry/dynamics.rs` | `EstimationDynamics` |
| Mapper | `registry/mappers.rs` | `Mapper` |
| SLAM system | `registry/slam.rs` | `SlamSystem` |
| Controller | `registry/controllers.rs` | `Controller` |
| Planner | `registry/planners.rs` | `Planner` |
| Adapter | `registry/adapters.rs` | `AckermannOutputAdapter` |

```rust
// In registry/estimators.rs
fn build_my_filter(ctx: EstimatorBuildContext) -> Result<Box<dyn StateEstimator>, String> {
    // Validate prerequisites — return Err if something required is absent
    let EstimatorConfig::MyFilter(cfg) = ctx.estimator_cfg else {
        return Err("build_my_filter received wrong config variant".to_string());
    };
    Ok(Box::new(MyFilter::new(cfg)))
}
```

### 3. Register it in the plugin's `build()`

```rust
impl Plugin for DefaultEstimatorsPlugin {
    fn build(&self, app: &mut App) {
        app.world_mut()
            .resource_mut::<AutonomyRegistry>()
            .register_estimator("MyFilter", build_my_filter); // key matches TOML `kind`
    }
}
```

### 4. Add the TOML config variant (if needed)

```toml
[autonomy_stack.world_model.estimator]
kind = "MyFilter"
# ...filter-specific params
```

That's it. No spawning systems change.

---

## Registering a New Sensor

Sensors have two parts: an ECS plugin (sim side) and a measurement model (core side).

### 1. Implement the measurement model in `helios_core`

```rust
// helios_core/src/models/estimation/measurement/my_sensor.rs
pub struct MySensorModel { ... }
impl Measurement for MySensorModel { ... }
```

### 2. Create a sensor plugin in `helios_sim`

Location: `helios_sim/src/simulation/plugins/sensors/my_sensor.rs`

Follow the `ImuPlugin` pattern:

```rust
pub struct MySensorPlugin;
impl Plugin for MySensorPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
               OnEnter(AppState::SceneBuilding),
               spawn_my_sensor.in_set(SceneBuildSet::ProcessSensors),
           )
           .add_systems(FixedUpdate, run_my_sensor.in_set(SimulationSet::Sensors));
    }
}
```

**Spawning system** (`SceneBuildSet::ProcessSensors`):
- Create a child entity under the agent.
- Attach `MeasurementModel(Arc::new(MySensorModel { ... }))` and `TrackedFrame`.
- Read all config from TOML — no hardcoded noise values.

**Runtime system** (`SimulationSet::Sensors`):
- Read `GroundTruthState`, apply sensor physics and noise.
- Emit `BevyMeasurementMessage`.

### 3. Register the plugin

Add to `HeliosSimulationPlugin::build()` in `helios_sim/src/lib.rs`:

```rust
app.add_plugins(MySensorPlugin);
```

The `EstimationPlugin` spawner automatically picks up the new sensor's `MeasurementModel`
when building the estimator's measurement model map — no registry change needed for sensors.

---

## Open Problem: Downstream Consumers in Chained Pipelines

The registry builds algorithm objects cleanly in isolation, but breaks down when a downstream
consumer (controller, planner) needs to read estimated state produced by an upstream algorithm
it doesn't own.

**Concrete example:** A controller needs a pose to compute a steering command. Which estimate
does it use?

```
EKF estimator ──┐
                ├── EstimatorComponent  ←─?─  Controller
SLAM system   ──┘
```

In `EstimatorComponent::Separate`, the EKF is the answer. In `EstimatorComponent::CombinedSlam`,
the SLAM system is. If both are eventually possible simultaneously (e.g., EKF for velocity,
SLAM for global position), the question has no clean answer at the type level, and the controller
factory has no way to express which it needs. **This is unsolved.** The approaches below are
candidates, not decisions.

---

### Candidate Approaches

#### 1. TopicBus as the contract boundary (most Helios-native)

World model systems already publish to `/{agent}/odometry/estimated`. Controllers subscribe
from TopicBus using the agent name as the key. No direct ownership coupling between algorithms.

The "which estimate wins" question is answered by world model systems: whichever system writes
to the canonical topic is authoritative. If both EKF and SLAM are active, only one writes to
`/odometry/estimated`; the other writes to a secondary topic.

**Upside:** Zero shared ownership. Controller factories don't need to know what's upstream.
**Downside:** TopicBus is one-frame-stale by design (publish → next frame read). A controller
running in the same `FixedUpdate` tick as estimation would always see the previous frame's state.
Tight latency budgets break here.

---

#### 2. `CurrentBestEstimate` ECS component

Add a `CurrentBestEstimate(FrameAwareState)` component, written every frame by the world model
system after its update, read by the controller system. No ownership transfer. One writer per
agent, many readers.

```
WorldModelSystem  →  CurrentBestEstimate  →  ControllerSystem
```

The `WorldModelSet` runs before `ControllerSet`, so the component is always fresh within a tick.

**Upside:** Same-tick reads. Simple ECS pattern. Controller factories need no knowledge of
which estimator is upstream.
**Downside:** Adds a staging component. The world model system must decide what "best" means
when multiple estimates are available (which is the whole unsolved question — just moved).

---

#### 3. Typed estimate consumers in `ControllerBuildContext`

Give the controller factory an `estimate_source: EstimateSource` enum in its build context,
configured from TOML. The enum selects which component/topic to poll.

```rust
pub enum EstimateSource { Estimator, Slam, TopicBus(String) }
```

The controller closure captures this and knows where to look at runtime.

**Upside:** Explicit, config-driven. No implicit "which one wins" logic.
**Downside:** Requires the controller trait to express this preference, which pulls topology
knowledge into algorithm code. Also requires the controller to handle the case where its
preferred source doesn't exist on the agent.

---

#### 4. Enforce a single authoritative estimate per agent

Constrain the architecture: exactly one algorithm per agent produces the "navigation estimate"
used by all downstream consumers. SLAM replaces the estimator when `CombinedSlam` is active.
The controller always reads from one place, regardless of which algorithm produced it.

The `CurrentBestEstimate` component (option 2) is the natural expression of this constraint —
the world model system is responsible for populating it correctly for its variant.

**Upside:** Eliminates the ambiguity at the design level rather than routing around it.
**Downside:** Precludes legitimate multi-estimate use cases (e.g., EKF velocity + SLAM position
fused in the controller). This is probably too restrictive for a research sim.

---

### Notes

- The `Controller` trait and `ControllerFactory` now exist in the registry. Controllers currently
  read estimated state from `EstimatorComponent` via the pipeline's `get_state()` method. The
  multi-estimate routing question remains open for future SLAM + EKF coexistence scenarios.
- Options 2 and 4 (staging component + single authoritative source) are compatible and together
  form a coherent design: one `CurrentBestEstimate` writer per agent, determined by world model
  variant, consumed uniformly by all downstream systems.

---

## Build Contexts

Each factory receives an owned context with everything it needs:

| Context | Key fields |
|---|---|
| `DynamicsBuildContext` | `agent_entity`, `agent_config`, `gravity_magnitude` |
| `EstimatorBuildContext` | above + `estimator_cfg`, `measurement_models`, `dynamics_factories` snapshot |
| `MapperBuildContext` | `agent_entity`, `mapper_cfg` |
| `SlamBuildContext` | `agent_entity`, `slam_cfg`, `agent_config`, `gravity_magnitude`, `measurement_models`, `dynamics_factories` |

Dynamics are accessed inside estimator factories via `ctx.dynamics_factories` (a snapshot
taken once before scene building). This is an `Arc`-clone of the registry's dynamics map,
so all registered dynamics are always available regardless of plugin registration order.
