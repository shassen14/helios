# Helios Engineering Standards

The rules every crate follows. Read this before writing code.

---

## 1. Three crates, one direction

| Crate | Holds | Never holds |
|---|---|---|
| `helios_core` | Algorithms (estimation, planning, control, mapping), their traits, sensor forward models, data types | Bevy, Avian, ECS, the pipeline |
| `helios_runtime` | The autonomy pipeline: a DAG of nodes over a typed channel bus, the node registry, autonomy config | Bevy, Avian, algorithm math |
| `helios_sim` | The Bevy + Avian3D host: scenes, physics, sensor simulation, rendering, TOML loading | Algorithm math, autonomy config structs |

- Dependencies point one way: `helios_core` ← `helios_runtime` ← `helios_sim`.
- `bevy` and `avian3d` are never required dependencies of `helios_core` or
  `helios_runtime`. The same pipeline is meant to run on hardware, without the
  sim.
- Bevy systems orchestrate; `helios_core` computes. No filter math,
  kinematics, or Jacobians inside a system.

## 2. Coordinate frames

Two coordinate conventions exist, and data is converted when it crosses
between them.

| | Convention | Axes | Used by |
|---|---|---|---|
| World | ENU | +X east, +Y north, +Z up | `helios_core`, `helios_runtime` |
| Body / sensor | FLU | +X forward, +Y left, +Z up | `helios_core`, `helios_runtime` |
| Bevy / Avian | Y-up | +X right, +Y up, −Z forward | rendering, physics, gizmos |

All are right-handed.

- **Every conversion goes through `helios_sim/src/core/transforms/`**, the
  `ToBevy` / `FromBevy` traits. A manual axis swap anywhere else is a bug,
  including copying one component of a Bevy vector into an ENU slot.
- Core types carry their frame in the type (`Point<Enu>`, `FreeVector<Flu>`),
  so mixing frames is a compile error rather than a wrong number.

## 3. Units

SI everywhere at runtime: meters, seconds, kilograms, radians, m/s, rad/s,
µT. Values are `f64`.

**One exception:** TOML may use degrees where a human reads them (fields of
view, headings, steering limits, lidar ring elevations). They are converted
to radians on load. Degrees are never stored at runtime.

## 4. Sensors: truth model versus filter model

Two families describe a sensor, and they must stay separate:

- **Forward models** (`helios_core::sensors`): what the sensor actually does,
  including noise, bias, and saturation. The sim uses these to generate
  readings.
- **Measurement models** (`helios_core::estimation::measurement`): what the
  estimator *believes* the sensor does.

Simulating a sensor with a measurement model is an inverse crime: the filter
is graded against its own assumptions and only fails on hardware. Measurement
models never appear in `helios_sim`.

Sensors that need the world (lidar, radar, sonar, camera) work in two phases:
core generates the query, the host runs it against its scene, and core packs
the result.

## 5. Configuration

All TOML lives in `configs/`, never inside a crate.

```
configs/
  entities/    physical things: vehicles, sensors, world objects
  runtime/     portable autonomy config (catalog prefabs, agent profiles)
  sim/         sim-only config (worlds, agents, scenarios)
  observation/ observability presets
```

- **Composition by reference only** (`from = "..."`). No inheritance, no
  `extends`, no overriding a referenced prefab. Scenario `[overrides]` blocks
  are the one place leaf values are overridden.
- **One value, one place.** Everything else refers to it.
- **Prefabs are reusable algorithm settings.** What is specific to one agent,
  such as which channels a node reads, lives in the agent profile.
- **Channel names come from config.** Sim and hardware code publish to the
  names the agent profile declares; they never invent them.
- **No magic values in Rust.** Noise, rates, and model choices live in TOML;
  any other constant is named.

## 6. Code

- **Errors:** startup may panic on bad config, which fails fast. Runtime code
  never calls `.unwrap()` or `.expect()`; it warns and skips instead.
- **Channel writes:** the result of a bus write is never discarded. An
  unknown channel means the pipeline was never wired to it, so warn loudly
  and name the channel.
- **Randomness:** never `thread_rng()` or `OsRng`. Every random source is
  seeded from the scenario, so runs are reproducible.
- **System sets:** every sim system belongs to exactly one `SimulationSet` or
  `SceneBuildSet` variant. No reliance on implicit ordering.
- **Shared strings** (channel names, node kinds, action names) are defined
  once, in the module that owns the concept, and imported everywhere else.
- **Imports,** in three groups separated by a blank line: this crate; other
  helios crates; std and third-party.
- **Files are short** and split by responsibility. Within a file, each type's
  definition is followed by its `impl` blocks; free functions come next; tests
  come last.
- **Comments describe the code as it is.** They do not cite plans, phases, or
  documents.
