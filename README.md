# Helios

A robotics autonomy framework in Rust, with a simulator to develop it in.

The autonomy, meaning estimation, mapping, planning and control, is plain Rust
with no simulator or engine dependency. A Bevy + Avian3D host simulates the
vehicle, its sensors and the world around it, and feeds the autonomy through
the same channels a hardware driver would. The goal is that a stack that
drives in simulation runs unchanged on a real vehicle.

<!-- Screenshot: the proving ground with the map and point cloud on.
![The proving ground: a car mapping crates and walls with its lidar](docs/images/proving_ground.png)
-->

## Quick start

Needs Rust (the toolchain version is pinned in `rust-toolchain.toml`).

```sh
cargo run --bin helios_play
```

This opens the proving ground: a car estimates its pose from an IMU, GPS and
magnetometer, maps the area with its lidar, plans a path around obstacles and
drives to a goal 40 m away.

| Key | Does |
|---|---|
| `W` `A` `S` `D` | Orbit and pitch the camera |
| `=` / `-` | Zoom |
| Click / `Esc` | Select an agent or object / deselect |
| Arrow keys | Drive the car yourself; let go and autonomy takes back over |
| `M` | Occupancy map |
| `L` | Lidar point cloud |
| `T` / `G` | Coordinate frames in the scene / as a panel (`O` flips the panel's layout) |
| `C` / `B` | Colliders / bounding boxes |

Other runs:

```sh
cargo run --bin helios_play -- --headless                  # no window
cargo run --bin helios_play -- --scenario <path.toml>      # another scenario
cargo run --bin helios_test_sim -- --run configs/test/runs/proving_ground.toml
cargo test                                                  # unit tests
```

`helios_test_sim` runs a scenario without a window, checks the assertions in
the run file, and writes a report with run metrics such as where the car
ended up. `--monte-carlo N` repeats a run over N seeds and aggregates them.

## Crates

| Crate | What it is |
|---|---|
| `helios_core` | The algorithms and their data types: estimators, planners, controllers, mappers, sensor models. No engine dependency. |
| `helios_runtime` | The autonomy pipeline: algorithms wrapped as nodes, connected by named, typed channels, built from config. No engine dependency. |
| `helios_sim` | The simulator: Bevy for rendering and the app, Avian3D for physics. Simulates bodies, sensors and worlds, and hosts a pipeline per agent. |
| `helios_test` | Scenario runs with assertions, metrics and Monte Carlo batches. |

Dependencies point one way: `helios_core` ← `helios_runtime` ← `helios_sim`.

## What exists today

- **Vehicle:** a car with spring-damper suspension and a friction-circle tire
  model, driven by wheel torque and a steering angle.
- **Sensors:** IMU, GPS, magnetometer, and planar and multi-ring lidar, each
  with configurable noise and bias.
- **Estimation:** an extended Kalman filter predicting from the IMU, corrected
  by GPS and magnetometer.
- **Mapping and planning:** a 2D occupancy grid and A\*.
- **Control:** pure-pursuit path following, speed feedback with road-load
  feedforward, and steering from a bicycle model. Keyboard driving takes over
  and hands back cleanly.
- **Worlds:** objects modeled in Blender, placed by TOML world files.

## Documentation

Start at [docs/README.md](docs/README.md): an overview of how the pieces fit,
guides for configuration, assets and profiling, and the engineering
standards.

## License

MIT.
