# How Helios Fits Together

Helios splits a robot into two halves:

- **the brain:** the autonomy that estimates, maps, plans and controls;
- **the host:** whatever the brain is running on, which supplies sensor data
  and carries out commands.

Today the host is a simulator. The brain is written so that a hardware host
can replace it without changing a line of autonomy code. Most of the design
follows from keeping that promise.

---

## 1. Three crates

```
helios_core  ←  helios_runtime  ←  helios_sim
 algorithms      the pipeline        the simulator host
```

| Crate | Knows about | Never knows about |
|---|---|---|
| `helios_core` | math: filters, search, control laws, sensor models, typed frames and units | how algorithms are wired, any engine |
| `helios_runtime` | wiring: nodes, channels, config, building and running a pipeline | Bevy, Avian, rendering, physics |
| `helios_sim` | the world: physics bodies, sensor simulation, scenes, the viewer | algorithm math |

Dependencies point one way. Neither `helios_core` nor `helios_runtime`
depends on Bevy or Avian, so both can be built for a vehicle computer.

## 2. The pipeline

An agent's brain is an **autonomy pipeline**: a graph of nodes that pass data
over named, typed channels.

```
sensor channels ─► preprocessing ─► estimator ─► mapper ─► planner
                                        │                     │
                                        └────► path follower ◄┘
                                                    │
                                    controllers ─► allocators ─► actuator commands
```

- **A node** wraps one algorithm from `helios_core`. It declares the channels
  it reads and writes and how often it runs. Each tick, every node that is due
  runs in dependency order.
- **A channel** is identified by its data type and its name, such as an IMU
  reading on `sensor.imu.accel`. A channel holds the latest value. Readers
  that find nothing yet (a cold start, a sensor that dropped out) skip their
  tick rather than fail.
- **The graph is built from config.** The agent profile lists the nodes and
  names their channels, and a registry turns each `kind` string into a node.
  Building checks the wiring before anything runs: each channel has one
  producer, no cycles, and every sensor channel a node reads is actually
  published by the body.
- **Frames come from a transform service.** Nodes ask for the transform
  between named frames (`odom`, `base_link`, a sensor's frame) at a given
  time, rather than reading poses off channels.

Algorithms stay unaware of the graph, and the graph stays unaware of where its
data comes from.

## 3. The host boundary

Everything that crosses between host and brain is a channel read or write:

| Direction | What crosses |
|---|---|
| Host → brain | sensor readings, on the channel names the profile declares; the clock; the transforms the host knows (sensor mounts, and ground truth in sim) |
| Brain → host | actuator commands, such as a wheel torque or a steering angle |

The simulator's side, for each agent, every physics step:

1. Simulate each sensor and publish its readings.
2. Tick the agent's pipeline.
3. Read the actuator commands and apply them to the physics body.

A hardware host does the same with drivers in place of step 1 and motors in
place of step 3.

## 4. Sensors: truth versus belief

A sensor is described twice, in two places that never mix:

- **Forward models** (`helios_core::sensors`) are what the sensor really does,
  including noise, bias, range limits and its view of its own vehicle. The
  simulator uses them to produce readings.
- **Measurement models** (`helios_core::estimation::measurement`) are what
  the estimator believes the sensor does.

If the simulator produced readings with the estimator's own measurement
model, the estimator would be tested against its own assumptions and would
only meet its real errors on hardware. Keeping the two separate means
simulation exposes those errors first.

Sensors that look at the world, such as lidar, need the host's scene. Core
generates the rays, the host casts them against its physics world, and core
turns the hits into a reading. A lidar publishes what it measured, a grid of
ranges per beam. Turning that into a point cloud is a node in the brain, the
same as it would be for a real lidar's driver output.

## 5. Frames and units

- The brain works in ENU world coordinates (x east, y north, z up) and FLU
  body coordinates (x forward, y left, z up). Bevy is y-up. All conversion
  happens in one module of the simulator.
- Core types carry their frame (`Point<Enu>`, `FreeVector<Flu>`), so mixing
  frames fails to compile.
- Everything is SI at runtime. Config may use degrees where people read them.

## 6. Configuration

Everything an agent is, is config: the vehicle, sensors and mounts in entity
files; the autonomy in an agent profile; the world as placed objects; the run
in a scenario. Files refer to each other by name rather than copying, and the
portable parts (`configs/runtime/`) contain nothing simulator-specific. See
[guides/configuration.md](guides/configuration.md).

## 7. Testing

- **Unit tests** live beside the code in each crate.
- **`helios_test`** runs whole scenarios: assertions over values on the
  pipeline's channels, run metrics, and Monte Carlo batches over seeds.
- **Determinism:** each scenario has a master seed, and every random source
  derives from it, so a failing run can be replayed.

## 8. Built and planned

| Built | Planned |
|---|---|
| One car with suspension and tire models; IMU, GPS, magnetometer, 2D and 3D lidar | More vehicle types; cameras, radar and sonar |
| EKF estimation, 2D occupancy grid, A\*, pure pursuit, speed and steering control, keyboard driving with hand-back | A 3D world model, and planning that accounts for the vehicle's size and terrain |
| Single-threaded pipeline, one per agent | Nodes at different rates on parallel lanes |
| Simulator host | A hardware host loading the same agent profiles |
| Agents each with their own pipeline and frames | Multi-agent scenarios, and agents communicating over a network |
