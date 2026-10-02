# Helios Documentation — Start Here

> Recommended reading order for new contributors is top-to-bottom.

---

## Top-Level References

| File                               | Description                                                                           |
| ---------------------------------- | ------------------------------------------------------------------------------------- |
| [architecture.md](architecture.md)                       | High-level system overview and three-crate architecture rationale              |
| [pipeline_vision.md](pipeline_vision.md)                 | Full intended pipeline stage-by-stage vision and multi-agent coordination      |
| [pipeline_scope.md](pipeline_scope.md)                   | DAG pipeline contract, in/out-of-scope architectures, pluggable boundary      |
| [pipeline_communication.md](pipeline_communication.md)   | Communication layers (port bus, Zenoh), distributed single-agent deployment   |
| [standards.md](standards.md)                             | Project-wide coding, coordinate frame, and units standards (read before code) |
| [config_design/](config_design/README.md)                | Active design thread: config vocabulary, channel wiring, composition/overrides |

---

## 1 — Core Engine

Foundation layer: simulation loop, physics integration, coordinate transforms, and config loading.

| File                                                                     | Description                                                                               |
| ------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------- |
| [1_core_engine/simulation_engine.md](1_core_engine/simulation_engine.md) | Bevy app structure, `SimulationSet` ordering, `AppState` lifecycle                        |
| [1_core_engine/physics.md](1_core_engine/physics.md)                     | Avian3D integration, rigid body setup, collision layers                                   |
| [1_core_engine/transforms.md](1_core_engine/transforms.md)               | `transforms.rs` helpers — ENU↔Bevy conversions (mandatory reading before touching frames) |
| [1_core_engine/configuration.md](1_core_engine/configuration.md)         | TOML config loading, catalog resolver, `SpawnAgentConfigRequest` flow                     |

---

## 2 — Perception

Sensor simulation and asset creation.

| File                                                             | Description                                                                            |
| ---------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| [2_perception/blender_assets.md](2_perception/blender_assets.md) | Blender asset standards: scale, coordinate system, collision meshes, semantic metadata |

---

## 3 — World Model

State estimation, mapping, and the algorithm registry.

| File                                                                     | Description                                                                      |
| ------------------------------------------------------------------------ | -------------------------------------------------------------------------------- |
| [3_world_model/autonomy_registry.md](3_world_model/autonomy_registry.md) | `AutonomyRegistry` — how algorithms are registered, discovered, and instantiated |

---

## 4 — Action

Planning and vehicle actuation.

| File                                                               | Description                                                                |
| ------------------------------------------------------------------ | -------------------------------------------------------------------------- |
| [4_action/planning.md](4_action/planning.md)                       | Planner trait, pipeline levels, goal injection                             |
| [4_action/ackermann_actuation.md](4_action/ackermann_actuation.md) | Three-layer Ackermann control pipeline, adapter pattern, TOML tuning guide |

---

## 5 — Telemetry & Tools

Data flow from simulation to observability tools, and profiling workflows.

| File                                                                       | Description                                                                           |
| -------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| [5_telemetry_tools/topic_bus.md](5_telemetry_tools/topic_bus.md)           | `TopicBus` — **deprecated**, see `pipeline_communication.md` for replacement          |
| [5_telemetry_tools/profiling_cpu.md](5_telemetry_tools/profiling_cpu.md)   | CPU profiling with samply — build, record, and read the profile in the browser        |

