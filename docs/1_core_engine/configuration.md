# Configuration & Asset Loading

Helios uses a two-layer TOML configuration system:

- **Catalog** — a library of reusable prefabs (sensors, vehicles, agents, algorithm components).
- **Scenario** — a top-level file that assembles agents from catalog pieces and sets world parameters.

A resolver runs at startup to recursively expand `from` references and deep-merge overrides before deserializing into typed Rust structs.

---

## Directory Layout

All TOML configuration lives in `configs/` at the workspace root. Never inside a crate.

```
configs/
├── scenarios/              ← Top-level simulation scenarios
│   ├── 00_tutorial_showcase.toml
│   ├── simple_car_scenario.toml
│   └── isolation/          ← Single-subsystem test scenarios
│       ├── control_only.toml
│       ├── estimation_only.toml
│       ├── mapping_only.toml
│       ├── path_tracking.toml
│       └── planning_only.toml
├── catalog/
│   ├── agent_profiles/     ← Portable autonomy stacks (helios_hw safe)
│   │   ├── basic_car.toml
│   │   ├── default_car.toml
│   │   └── pro_car.toml
│   ├── agents/             ← Sim-only prefabs: vehicle + sensors + profile
│   │   ├── ackermann_car.toml
│   │   ├── ackermann_car_basic.toml
│   │   └── ackermann_car_pro.toml
│   ├── components/         ← Algorithm parameter sets
│   │   ├── estimators/     ← ekf_basic.toml, ekf_advanced.toml, ekf_pro.toml
│   │   ├── controllers/    ← pid_path_follower.toml, pid_aggressive.toml
│   │   ├── mappers/        ← occupancy_grid_2d.toml, none.toml
│   │   └── planners/       ← a_star.toml, a_star_global.toml, rrt_star.toml
│   ├── sensor_suites/      ← Named sensor bundles
│   │   ├── ins_basic.toml
│   │   ├── ins_standard.toml
│   │   └── ins_pro.toml
│   ├── sensors/            ← Individual sensor definitions
│   │   ├── imu_basic.toml, imu_standard.toml, imu_pro.toml
│   │   ├── gps_basic.toml, gps_standard.toml, gps_pro.toml
│   │   ├── magnetometer_basic.toml, magnetometer_pro.toml
│   │   └── lidar2d.toml
│   ├── vehicles/           ← Vehicle physics prefabs
│   │   ├── ackermann_base.toml
│   │   └── ackermann_truck.toml
│   └── objects/            ← World object type definitions
│       ├── building_office.toml
│       ├── stop_sign.toml
│       ├── traffic_cone.toml
│       └── tree_oak.toml
└── fixtures/               ← Baked test data (paths, maps)
    ├── paths/
    └── maps/
```

**Catalog key format**: path relative to `catalog/`, slashes and file extension stripped, segments joined with dots.
- `catalog/sensors/imu_pro.toml` → key `sensors.imu_pro`
- `catalog/components/estimators/ekf_pro.toml` → key `components.estimators.ekf_pro`

---

## Loading Pipeline

Loading runs during `AppState::AssetLoading`, in the `AssetLoadSet::Config` system set.

```
1. load_catalog_from_disk()
   Walk {config_root}/catalog/ recursively.
   For each .toml file: parse with Figment → raw Value → insert into PrefabCatalog.

2. load_and_resolve_scenario()
   Load scenario TOML → RawScenarioConfig (agents kept as Vec<Value>).
   For each agent Value:
     resolve_agent_value() → fully resolved Value (all `from` refs expanded)
     deserialize → AgentConfig
   Assemble ScenarioConfig resource → insert into ECS.
```

`ScenarioConfig` is then available to all subsequent spawning systems.

---

## The Resolver

**Source**: `helios_sim/src/simulation/config/resolver.rs`

The resolver walks a raw `Value` tree and expands any dict that contains a `from` key.

**Algorithm (recursive)**:

1. If the current dict has `from = "namespace.key"`:
   - Look up `catalog["namespace.key"]`; error if missing.
   - Recursively resolve the referenced prefab.
   - Deep-merge the current dict's remaining fields on top of the resolved base.
   - Strip the `from` key from the merged result.
2. Recurse into every child field (dicts and arrays).
3. Scalars pass through unchanged.

**Deep-merge rules**:
- If both base and override are dicts and the override has **no** `from` key → merge field-by-field recursively.
- If the override dict **has** a `from` key → replace the entire field (no merge).
- Otherwise the override value replaces the base value.

---

## `from` Reference Syntax

```toml
# Inline (single-line)
vehicle = { from = "vehicles.ackermann_base" }

# Table form (allows overrides in adjacent lines)
[agents.vehicle]
from = "vehicles.ackermann_base"
mass = 900.0              # override one field after expanding base
```

`from` values are always dot-separated catalog keys. Nested `from` references are fully resolved before merging.

---

## Scenario File Structure

**Source**: `helios_sim/src/simulation/config/structs/scenario.rs`

```toml
[simulation]
seed             = 42           # optional u64 RNG seed
duration_seconds = 120.0
frequency_hz     = 400.0        # default 400 Hz
log_topics       = ["pose", "ekf_state"]
profile          = "low_power"  # optional SimulationProfile name
mock_path        = "fixtures/paths/circle_r5.toml"   # optional
mock_map         = "fixtures/maps/empty_20x20.toml"  # optional

[world]
[world.terrains]
# see terrain config below

[world.atmosphere]
gravity            = [0.0, -9.81, 0.0]  # Bevy Y-up
sun_elevation      = 45.0               # degrees
sun_azimuth        = 180.0              # degrees from North
ambient_lux        = 8000.0
fog_density        = 0.0

[[world.objects]]
prefab   = "objects.stop_sign"
position = [10.0, 5.0, 0.0]             # ENU world frame [East, North, Up]
orientation_degrees = [0.0, 0.0, 45.0]  # [roll, pitch, yaw]
scale    = [1.0, 1.0, 1.0]

[debug]
show_pose_gimbals   = true
show_covariance     = false
show_point_cloud    = false
show_velocity       = false
show_error_line     = false
show_path_trail     = true
show_occupancy_grid = true
show_tf_frames      = false
show_planned_path   = true
show_legend         = true

[metrics]
output_path = "results/run_001.csv"     # optional

[[agents]]
# ... (see agent config below)
```

### Terrain Config

```toml
[[world.terrains]]
mesh                = "terrain/ground.glb"          # visual mesh
collider            = "terrain/ground_collision.glb" # optional separate collision mesh
medium              = "air"                          # "air" | "water" | "vacuum"
position            = [0.0, 0.0, 0.0]               # ENU
orientation_degrees = [0.0, 0.0, 0.0]
```

If `collider` is omitted, the physics engine uses the visual mesh as a trimesh collider.

### World Object Prefab

Defined in `catalog/objects/`:

```toml
# catalog/objects/stop_sign.toml
label    = "stop_sign"
class_id = 10

visual_mesh = "objects/stop_sign.glb"
bounding_box = [0.6, 2.6, 0.6]          # [width, height, depth] meters

[collider]
shape       = "capsule"                  # "box" | "sphere" | "capsule" | "cylinder"
radius      = 0.04
half_height = 1.0
```

---

## Agent Config

An agent combines a portable software stack (`AgentBaseConfig`) with simulation-specific fields.

**Source**: `helios_sim/src/simulation/config/structs/scenario.rs` (`AgentConfig`),
`helios_runtime/src/config/agent.rs` (`AgentBaseConfig`)

```toml
[[agents]]
from = "agents.ackermann_car_pro"   # optional: expand a prefab first

name = "Agent-01"
starting_pose = { translation = [0.0, 0.0, 1.0], rotation = [0.0, 0.0, 90.0] }
goal_pose     = { translation = [100.0, 50.0, 0.5] }

[agents.vehicle]
from = "vehicles.ackermann_base"

[agents.sensors]
imu  = { from = "sensors.imu_pro" }
gps  = { from = "sensors.gps_standard" }
lidar = { from = "sensors.lidar2d" }

[agents.autonomy_stack]
from = "agent_profiles.pro_car"
```

`starting_pose` and `goal_pose` are in **ENU** world frame. `rotation` is Euler angles in **degrees** `[roll, pitch, yaw]`.

### Agent Profile (Portable)

Stored in `catalog/agent_profiles/`. Contains only the autonomy stack — no vehicle or sensors.
Can be loaded by `helios_hw` without any `helios_sim` dependency.

```toml
# catalog/agent_profiles/pro_car.toml
[world_model]
type = "Separate"
estimator = { from = "components.estimators.ekf_pro" }
mapper    = { from = "components.mappers.none" }

[planners]
local_path = { from = "components.planners.a_star" }

[controllers]
path_follower = { from = "components.controllers.pid_path_follower" }
```

### Agent Prefab (Sim-Specific)

Stored in `catalog/agents/`. Contains vehicle, sensors, poses, and a profile reference.

```toml
# catalog/agents/ackermann_car_pro.toml
name = "ProCar"
starting_pose = { translation = [0.0, 0.5, 1.0], rotation = [0.0, 0.0, 0.0] }
goal_pose     = { translation = [0.0, 0.5, 0.0] }

vehicle          = { from = "vehicles.ackermann_base" }
sensors          = { from = "sensor_suites.ins_pro" }
autonomy_stack   = { from = "agent_profiles.pro_car" }
```

---

## Vehicle Config

**Source**: `helios_sim/src/simulation/config/structs/vehicle.rs`

```toml
# catalog/vehicles/ackermann_base.toml
kind            = "Ackermann"
wheelbase       = 2.5             # meters
max_steering_angle = 35.0         # degrees
max_steering_rate  = 140.0        # degrees/sec

[physics]
mass             = 1500.0         # kg
friction         = 0.7            # Avian3D friction coefficient
linear_damping   = 0.5            # passive drag
angular_damping  = 1.5            # yaw decay

[actuator]
max_force   = 22000.0             # N
max_torque  = 10000.0             # N·m
max_speed   = 20.0                # m/s

[adapter]
kind = "Default"                  # "Default" (open-loop) or "DualSisoPid"
```

**Adapter variants**:

```toml
# DualSisoPid adapter — closed-loop velocity + steering control
[adapter]
kind = "DualSisoPid"

[adapter.longitudinal]
kp = 1.0
ki = 0.1
kd = 0.05
integral_clamp = 10.0

[adapter.lateral]
kp = 2.0
ki = 0.0
kd = 0.1
```

---

## Sensor Configs

**Source**: `helios_sim/src/simulation/config/structs/sensors.rs`

Sensors are named in a `HashMap<String, SensorConfig>`. All `transform` fields are in the **body FLU** frame (Forward=+X, Left=+Y, Up=+Z). Noise values are standard deviations.

### IMU

```toml
# catalog/sensors/imu_pro.toml
kind = "Imu"
type = "SixDof"      # "SixDof" (accel + gyro) or "NineDof" (+ magnetometer)
name = "imu_pro"
rate = 400.0         # Hz
transform = { translation = [0.0, 0.0, 0.0], rotation = [0, 0, 0] }

accel_noise_stddev = [0.001, 0.001, 0.001]   # m/s² per axis
gyro_noise_stddev  = [0.0001, 0.0001, 0.0001] # rad/s per axis
```

### GPS

```toml
# catalog/sensors/gps_standard.toml
kind = "Gps"
name = "gps_standard"
rate = 10.0
transform = { translation = [0.0, 0.0, 0.5] }

noise_stddev = [1.0, 1.0, 2.0]   # [East, North, Up] in meters
```

### Magnetometer

```toml
kind = "Magnetometer"
name = "mag_pro"
rate = 100.0
transform = { translation = [0.0, 0.0, 0.0] }

noise_stddev = [0.1, 0.1, 0.1]   # µT per axis (body frame)
```

### LiDAR 2D

```toml
# catalog/sensors/lidar2d.toml
kind = "Lidar"
type = "Lidar2D"
rate = 10.0
transform = { translation = [1.5, 0.0, 0.4] }

max_range          = 30.0     # meters
horizontal_fov     = 270.0    # degrees
horizontal_beams   = 540
range_noise_stddev = 0.02     # meters
debug_visuals      = false
```

### LiDAR 3D

```toml
kind = "Lidar"
type = "Lidar3D"
rate = 10.0
transform = { translation = [1.5, 0.0, 0.4] }

max_range          = 100.0
horizontal_fov     = 360.0
horizontal_beams   = 1800
vertical_fov       = 30.0
vertical_beams     = 32
range_noise_stddev = 0.03
debug_visuals      = false
```

### Sensor Suite Bundle

```toml
# catalog/sensor_suites/ins_pro.toml
[imu_pro]
from = "sensors.imu_pro"

[gps_pro]
from = "sensors.gps_pro"

[magnetometer_pro]
from = "sensors.magnetometer_pro"
```

The sensor suite is deserialized directly as `HashMap<String, SensorConfig>`. Each key becomes the sensor's identifier in the agent's sensor map.

---

## Autonomy Stack Config

**Source**: `helios_runtime/src/config/`

### `AutonomyStack`

```toml
[autonomy_stack.world_model]
type = "Separate"                              # or "CombinedSlam"
estimator = { from = "components.estimators.ekf_pro" }
mapper    = { from = "components.mappers.occupancy_grid_2d" }

[autonomy_stack.planners]
local_path = { from = "components.planners.a_star" }

[autonomy_stack.controllers]
path_follower = { from = "components.controllers.pid_path_follower" }
```

`world_model` is a tagged enum on the `type` field:
- `Separate` — independent estimator + mapper.
- `CombinedSlam` — a single SLAM system.

### Estimator Config

**Source**: `helios_runtime/src/config/estimator.rs`

```toml
# catalog/components/estimators/ekf_pro.toml
kind = "Ekf"          # "Ekf" or "Ukf"

[config.dynamics]
type = "IntegratedImu"  # "IntegratedImu" | "AckermannOdometry" | "Quadcopter"

accel_noise_stddev     = 0.2
gyro_noise_stddev      = 0.01
accel_bias_instability = 0.05
gyro_bias_instability  = 0.005
```

`kind` selects the filter type (`tag = "kind", content = "config"`). `dynamics.type` selects the process model.

**Dynamics process noise fields**:

| Dynamics | Fields |
|----------|--------|
| `IntegratedImu` | `accel_noise_stddev`, `gyro_noise_stddev`, `accel_bias_instability`, `gyro_bias_instability` |
| `AckermannOdometry` | `velocity_stddev`, `yaw_rate_stddev` |
| `Quadcopter` | `force_stddev`, `torque_stddev` |

### Mapper Config

**Source**: `helios_runtime/src/config/mapper.rs`

```toml
# catalog/components/mappers/occupancy_grid_2d.toml
kind = "OccupancyGrid2D"
rate       = 5.0       # Hz
resolution = 0.2       # meters/cell
width_m    = 40.0      # rolling window East extent
height_m   = 40.0      # rolling window North extent
pose_source = "GroundTruth"  # or "Estimated"
```

`kind = "None"` disables mapping.

### Planner Config

**Source**: `helios_runtime/src/config/planner.rs`

```toml
# catalog/components/planners/a_star.toml
kind = "AStar"
rate = 2.0             # Hz

arrival_tolerance_m   = 1.5    # meters to goal (default 1.5)
occupancy_threshold   = 180    # 0–255 cell value to treat as obstacle (default 180)
max_search_depth      = 50000  # node expansion limit (default 50 000)
enable_path_smoothing = true
replan_on_path_deviation = true
deviation_tolerance_m = 3.0    # meters off-path before replanning (default 3.0)
level = "local"                # planner pipeline level key
```

### Controller Config

**Source**: `helios_runtime/src/config/controller.rs`

```toml
# PID controller
kind = "Pid"
rate = 50.0
kp = 1.0
ki = 0.1
kd = 0.05

# LQR controller
kind = "Lqr"
gain_matrix  = [1.0, 0.0, 0.0, 1.0]  # flat row-major
state_dim    = 2
control_dim  = 2
u_min = [-1.0, -0.5]
u_max = [1.0, 0.5]

# Feedforward + PID
kind = "FeedforwardPid"
dynamics_key = "ackermann"
kp = [1.0, 0.5]
ki = [0.1, 0.0]
kd = [0.05, 0.0]
u_min = [-1.0, -0.5]
u_max = [1.0, 0.5]
controlled_indices = [0, 1]
```

---

## Coordinate Frames & Units in Config

| Value | Frame | Unit in TOML | Unit at Runtime |
|-------|-------|--------------|-----------------|
| Agent/world poses | ENU (East=+X, North=+Y, Up=+Z) | meters / degrees | meters / radians (quaternion) |
| Sensor transforms | FLU body (Forward=+X, Left=+Y, Up=+Z) | meters / degrees | meters / radians (quaternion) |
| GPS noise | ENU | meters σ | meters σ |
| IMU noise | Body axes | m/s² σ, rad/s σ | same |
| Magnetometer noise | Body axes | µT σ | µT σ |
| Gravity | Bevy Y-up (-Z forward) | m/s² | m/s² |
| LiDAR FOV | — | degrees | converted to radians at construction |
| Steering limits | — | degrees | converted to radians at construction |

**Rule**: All angular values in TOML are degrees. Conversion to radians happens immediately on load inside the spawning system or `From` impl. No degrees anywhere at runtime.

---

## Serde Patterns

### Tagged Enum (`#[serde(tag = "kind")]`)

Selects a variant by the value of one field. The remaining fields are inlined into the variant.

```toml
kind = "Ackermann"
wheelbase = 2.5
```

### Content Enum (`#[serde(tag = "kind", content = "config")]`)

Variant data lives in a separate `config` subtable. Used for `EstimatorConfig` and `ControllerConfig`.

```toml
kind = "Ekf"
[config.dynamics]
type = "IntegratedImu"
```

### Flatten (`#[serde(flatten)]`)

Inlines the fields of a struct into the parent table. `AgentConfig` flattens `AgentBaseConfig`, so `name` and `autonomy_stack` appear at the top level of an `[[agents]]` entry.

### Custom Deserializers (`serde_helpers`)

`Pose` uses two custom de/serializers:
- `vec3_f64_from_f32_array` — deserializes `[f32; 3]` → `Vector3<f64>`
- `quat_f64_from_euler_deg_f32` — deserializes Euler degrees `[roll, pitch, yaw]` → `UnitQuaternion<f64>`

### Default Values

Two patterns coexist:
1. `#[serde(default = "fn_name")]` — calls a function for non-`Default` types.
2. `#[serde(default)]` — uses `Default::default()` (empty `HashMap`, `false`, `0.0`, etc.).

Missing optional fields use their defaults silently. Missing required fields cause a deserialization error and abort the scenario load.

---

## Config Struct Locations

| Struct | File | Crate |
|--------|------|-------|
| `AgentBaseConfig` | `helios_runtime/src/config/agent.rs` | `helios_runtime` |
| `AutonomyStack` | `helios_runtime/src/config/autonomy.rs` | `helios_runtime` |
| `EstimatorConfig` | `helios_runtime/src/config/estimator.rs` | `helios_runtime` |
| `ControllerConfig` | `helios_runtime/src/config/controller.rs` | `helios_runtime` |
| `PlannerConfig` | `helios_runtime/src/config/planner.rs` | `helios_runtime` |
| `MapperConfig` | `helios_runtime/src/config/mapper.rs` | `helios_runtime` |
| `SlamConfig` | `helios_runtime/src/config/slam.rs` | `helios_runtime` |
| `ScenarioConfig` | `helios_sim/src/simulation/config/structs/scenario.rs` | `helios_sim` |
| `AgentConfig` | `helios_sim/src/simulation/config/structs/scenario.rs` | `helios_sim` |
| `Vehicle` | `helios_sim/src/simulation/config/structs/vehicle.rs` | `helios_sim` |
| `SensorConfig` | `helios_sim/src/simulation/config/structs/sensors.rs` | `helios_sim` |
| `Pose` | `helios_sim/src/simulation/config/structs/pose.rs` | `helios_sim` |
| `TerrainConfig` / `AtmosphereConfig` | `helios_sim/src/simulation/config/structs/terrain.rs` | `helios_sim` |
| `WorldObjectPlacement` / `WorldObjectPrefab` | `helios_sim/src/simulation/config/structs/world_object.rs` | `helios_sim` |
| `PrefabCatalog` | `helios_sim/src/simulation/config/catalog.rs` | `helios_sim` |
| Resolver logic | `helios_sim/src/simulation/config/resolver.rs` | `helios_sim` |
