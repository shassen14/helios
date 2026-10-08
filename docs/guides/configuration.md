# Configuration

Every run is described by TOML files under `configs/`. No configuration
lives inside a crate, and tunable values (noise, rates, gains, model choices)
live here rather than in Rust.

---

## 1. Layout

```
configs/
  entities/           physical things, one file each
    vehicles/           bodies: topology, plant, actuation
    sensors/            sensor devices: kind, rate, noise, bias
    sensor_fits/        a vehicle's sensors: each unit's device, mount, channel(s)
    objects/            world objects: mesh, class, mass
  runtime/            portable autonomy (helios_runtime's vocabulary)
    catalog/            reusable algorithm prefabs: estimators, planners,
                        mappers, controllers, semantic classes
    profiles/
      agent_profiles/   one agent's whole autonomy stack
  sim/                simulator only (helios_sim's vocabulary)
    catalog/
      agents/           sim agents: a vehicle + a sensor fit + a profile
      worlds/           world layouts: placed objects
    scenarios/          what to run: world, agents, simulation settings
    interaction/        viewer tuning (colors, sizes)
    keybindings/        viewer keys
  test/               test runs and test-only scenarios
  fixtures/           maps and paths used by tests
```

Each file belongs to one crate's vocabulary. `runtime/` holds nothing about
physics or rendering, so a hardware host can load an agent profile without
the simulator.

## 2. References: `from`

Every file under `entities/`, `runtime/catalog/`, `runtime/profiles/` and
`sim/catalog/` is a **prefab**, named by its path with dots and without
`.toml`:

| File | Key |
|---|---|
| `configs/entities/sensors/lidar2d.toml` | `entities.sensors.lidar2d` |
| `configs/runtime/catalog/estimators/ekf_pro.toml` | `runtime.catalog.estimators.ekf_pro` |

A table containing `from = "<key>"` is replaced by that prefab:

```toml
vehicle = { from = "entities.vehicles.raycast_car" }

[sensors]
from = "entities.sensor_fits.raycast_car_ins_pro_lidar"
```

Keys written beside `from` are merged on top of the prefab. Scenarios use
this to give each agent instance its name and poses.

**Rules:**

- **Compose, don't override.** A prefab or profile refers to other prefabs
  whole. A different setup is a new file, not an override of an old one.
  Scenarios set per-instance values (names, poses); they do not restructure
  what they reference.
- **One value, one place.** Everything else refers to it.
- **Prefabs hold reusable settings.** What is specific to one agent, such as
  which channels a node reads, belongs in its agent profile. (The occupancy
  grid prefab still names its input channel; that is the known exception.)

A scenario with broken references fails to load and lists every failure at
once. Most sim-side tables reject unknown keys; most runtime prefabs do not
yet, so a misspelled key there is silently ignored. Check names against an
existing prefab.

## 3. Scenarios

A scenario is the file passed to `--scenario`:

```toml
[simulation]
duration_seconds = 180.0
seed = 2024                       # master seed; every random source derives from it

[world.layout]
from = "sim.catalog.worlds.proving_ground"

[world.semantic_classes]
from = "runtime.catalog.semantic_classes.default"

[world.atmosphere]
gravity_enu = [0.0, 0.0, -9.81]   # m/s²
sun_elevation = 45.0              # degrees
sun_azimuth = 180.0
ambient_lux = 1500.0

[[agents]]
from = "sim.catalog.agents.raycast_car"
name = "ProvingCar"
starting_pose = { translation = [-20.0, 0.0, 1.0], rotation = [0.0, 0.0, 0.0] }
goal_pose = { translation = [20.0, 0.0, 0.5] }
```

Add an `[[agents]]` entry per agent.

## 4. Worlds and objects

A world layout places object prefabs:

```toml
name = "proving_ground"

[[objects]]
name     = "north_wall"
prefab   = "entities.objects.wall_4m"
position = [0.0, 30.0, 0.0]
scale    = [15.0, 1.0, 1.0]
```

| Placement field | Meaning | Default |
|---|---|---|
| `name` | unique within the layout | required |
| `prefab` | an object prefab key | required |
| `position` | ENU meters of the object's origin, which sits on its base | required |
| `orientation_degrees` | [roll, pitch, yaw]; yaw 90 turns the object's front from east to north | `[0, 0, 0]` |
| `scale` | per axis, along the object's own axes | `[1, 1, 1]` |
| `body` | `"static"` (never moves) or `"dynamic"` (physics moves it; scale must be 1) | `"static"` |

An object prefab says what the thing is. Its shape comes from the mesh:

```toml
mesh    = "objects/crate_1m.glb"   # relative to helios_sim/assets/
class   = "crate"                  # a name from the semantic class catalog
mass_kg = 20.0                     # required for a dynamic placement
collides = true                    # default
```

Making the mesh is covered in
[blender_assets.md](blender_assets.md).

## 5. Agents

An agent is split in two, so the autonomy can run on hardware unchanged:

| | Sim agent (`sim/catalog/agents/`) | Agent profile (`runtime/profiles/agent_profiles/`) |
|---|---|---|
| Holds | the vehicle, its sensor fit, default poses, and which profile to run | the autonomy stack: its `[nodes]` (estimators, mappers, planners, path followers, controllers, allocators, teleop, …) and the seam sections that join them |
| Read by | the simulator | the simulator today, a hardware host later |

```toml
# sim/catalog/agents/raycast_car.toml
name = "RaycastCar"
starting_pose  = { translation = [0.0, 0.5, 1.0], rotation = [0.0, 0.0, 0.0] }
goal_pose      = { translation = [0.0, 0.5, 0.0] }
vehicle        = { from = "entities.vehicles.raycast_car" }
autonomy_stack = { from = "runtime.profiles.agent_profiles.raycast_car" }

# Last, so the keys above are not read as sensor entries.
[sensors]
from = "entities.sensor_fits.raycast_car_ins_pro_lidar"   # IMU + GPS + magnetometer + 2 lidars
```

Every node in a profile is a `[nodes.<name>]` table. The key is the node's
name; the table names its `kind` and that kind's settings, either inline or
through `from` to a catalog prefab:

```toml
[nodes.primary]
from = "runtime.catalog.estimators.ekf_advanced"

[nodes.front_lidar_deproject]
kind   = "Deproject"
input  = "sensor.lidar.front"
output = "sensor.lidar.front.points"
```

Where several nodes feed one channel, a seam section says which nodes join
it and how:

| Section | Says |
|---|---|
| `[estimate]` | which estimator the rest of the stack reads (`source`) |
| `[reference]` | which nodes feed the reference the controllers track (`base`, `preferred`) |
| `[command.<fold>]` | which controllers' commands are summed into one command an allocator reads (`type`, `required`, `optional`) |
| `[actuators]` | which allocators' partial commands make up what the body applies (`members`) |
| `[tf]` | sizing for the estimated transform buffer; optional |

A seam names nodes by their table key. `raycast_car.toml` uses all four
seams and is the worked example.

### Sensors and channels

A sensor is described in two layers, so each value lives in one place.

A **device** (`entities/sensors/`) holds what is true of every unit of one
model: its kind, rate, noise and bias, and for a lidar its beam pattern.

```toml
# entities/sensors/lidar2d.toml
kind = "Lidar"
rate = 10.0                                  # Hz
range_min = 0.15                             # nearer returns are misses
max_range = 50.0
azimuth_fov = 360.0                          # degrees
azimuth_beams = 360
ring_elevations = [0.0]                      # degrees; one entry per ring
range_noise_stddev = 0.03
angular_noise_stddev = 0.1
```

A **sensor fit** (`entities/sensor_fits/`) lists the units on one vehicle.
Each table is one **installation**: which device it is, where it is mounted,
and the channel(s) it publishes on. The table key names the sensor and
seeds its noise, so two units of one device draw independent noise.

```toml
# entities/sensor_fits/raycast_car_ins_pro_lidar.toml (excerpt)
[imu_pro]
device = { from = "entities.sensors.imu_pro" }
transform = { translation = [-1.0, 0.0, 0.0], rotation = [0.0, 0.0, 0.0] }  # body FLU
accel_channel = "sensor.imu.accel"
gyro_channel = "sensor.imu.gyro"

[lidar]
device = { from = "entities.sensors.lidar2d" }
transform = { translation = [1.5, 0.0, 0.5], rotation = [0.0, 0.0, 0.0] }
channel = "sensor.lidar.front"
```

- `transform` is required.
- The channel fields depend on the device kind. An IMU takes `accel_channel`
  and `gyro_channel`; every other kind takes `channel`. A missing field, or
  one the kind doesn't have, fails the load and names the field.
- A fit's mounts are measured on one vehicle, so a fit is named after its
  vehicle. Swapping devices in a fit (`raycast_car_ins_pro` →
  `raycast_car_ins_basic`) leaves the sensors where they are.

Channel names are written in config, never invented in code. A node in the
profile reads a sensor by naming that channel, and the agent fails to build
if a node reads a sensor channel no sensor on the body publishes. So adding
or removing a sensor means changing the sensor fit and the profile together.

## 6. Frames and units

- **World poses** (scenario poses, object placements) are ENU: x east,
  y north, z up, in meters.
- **Mounts** (an installation's `transform`, vehicle mounts) are body FLU: x forward,
  y left, z up, in meters.
- **Rotations** are `[roll, pitch, yaw]` in degrees. Degrees are allowed in
  config wherever a human reads the value (headings, fields of view, ring
  elevations) and are converted to radians on load. Everything else is SI.

## 7. Command line

`helios_play` and the other sim binaries take:

| Flag | Default | Meaning |
|---|---|---|
| `--scenario`, `-s` | `configs/sim/scenarios/01_proving_ground.toml` | the scenario to run |
| `--config-root` | `configs` | where prefab keys are resolved |
| `--headless` | off | no window; exits after `duration_seconds` |
| `--speed` | real time in a window; as fast as possible headless | simulated-time multiplier |
| `--seed` | the scenario's seed | overrides `[simulation] seed` |
