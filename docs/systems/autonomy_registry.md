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

The `WorldModelPlugin` spawner automatically picks up the new sensor's `MeasurementModel`
when building the estimator's measurement model map — no registry change needed for sensors.

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
