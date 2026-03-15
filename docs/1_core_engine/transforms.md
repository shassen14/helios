# Transform System

**Source:** `helios_sim/src/simulation/core/transforms.rs`

This module is the single gatekeeper between Helios's robotics coordinate frames (ENU world, FLU body) and Bevy's native coordinate system. All conversions happen here. Manual axis swaps anywhere else are a bug.

---

## Coordinate Frames

| Frame | Convention | Used For |
|-------|-----------|----------|
| **ENU** (world) | East=+X, North=+Y, Up=+Z — Right-Handed | Agent start poses, goal poses, world-relative positions |
| **FLU** (body/sensor) | Forward=+X, Left=+Y, Up=+Z — Right-Handed | Sensor offsets from vehicle body, ray directions |
| **Bevy world** | +X, Y-Up, -Z Forward — Right-Handed | Physics engine, rendering |
| **Bevy local** | Same axes, child-relative | Sensor child `Transform` components |

The rotation from ENU to Bevy world is a **−90° rotation around the X-axis**:

```
ENU X (East)   → Bevy +X
ENU Y (North)  → Bevy -Z
ENU Z (Up)     → Bevy +Y
```

The rotation from FLU to Bevy local chains FLU→ENU (+90° around Z) then ENU→Bevy (−90° around X):

```
FLU +X (Forward) → Bevy -Z
FLU +Y (Left)    → Bevy -X
FLU +Z (Up)      → Bevy +Y
```

---

## Two Conversion Contexts

There are exactly two contexts where conversions occur. Mixing them is a latent bug.

### Context 1 — World Frame (ENU ↔ Bevy world)

Use for any pose expressed in the global navigation frame:
- Agent `starting_pose` and `goal_pose` from TOML
- Ground truth state position/velocity
- Occupancy grid world origin

### Context 2 — Body/Sensor Frame (FLU ↔ Bevy local)

Use for any pose expressed relative to the vehicle body:
- Sensor `transform` field from TOML (offset + orientation relative to vehicle)
- Ray directions generated in the sensor model before firing into the physics world

The `Pose` config type exposes both contexts:
- `pose.to_bevy_transform()` — ENU world pose → Bevy world `Transform`
- `pose.to_bevy_local_transform()` — FLU body pose → Bevy local `Transform`

---

## API Reference

### TF Tree Infrastructure

| Type / Function | Description |
|-----------------|-------------|
| `TrackedFrame` | Bevy `Component`. Mark any entity whose pose should be queryable by the TF system. |
| `TfTree` | Bevy `Resource`. Per-frame snapshot of all tracked poses. Rebuilt once per tick by `tf_tree_incremental_update_system`. |
| `TfFramePose` | Snapshot struct published to `TopicBus` after each physics step for Foxglove trajectory plots. |

#### `TfTree` methods

| Method | Signature | Description |
|--------|-----------|-------------|
| `lookup_by_name` | `(&self, frame_name: &str) -> Option<Isometry3<f64>>` | World pose by human-readable name (e.g. `"truck/imu"`). |
| `lookup_by_entity` | `(&self, entity: Entity) -> Option<Isometry3<f64>>` | World pose by Bevy `Entity`. Used internally via `TfProvider`. |
| `lookup_local_by_entity` | `(&self, entity: Entity) -> Option<Isometry3<f64>>` | Parent-relative pose by `Entity`. ENU for root frames, FLU for sensor children. |
| `get_transform_by_name` | `(&self, from: &str, to: &str) -> Option<Isometry3<f64>>` | Transform from frame A to frame B by name. Result = `T_W_A⁻¹ * T_W_B`. |
| `iter_frames` | `(&self) -> impl Iterator<Item=(Entity, world_iso, local_iso, Option<Entity>)>` | Iterate all tracked frames. |

`TfTree` implements `helios_core::TfProvider`, so it can be passed directly into any pipeline or sensor that requires a `&dyn TfProvider`.

#### TF Systems

| System | Schedule | Trigger | Description |
|--------|----------|---------|-------------|
| `tf_tree_structural_system` | `SimulationSet::Precomputation` | `Added<TrackedFrame>` or `RemovedComponents<TrackedFrame>` | Handles entities being added/removed from the TF graph. No-op on most ticks. |
| `tf_tree_incremental_update_system` | `SimulationSet::Precomputation` **and** `SimulationSet::StateSync` | `Changed<GlobalTransform>` | Incrementally updates world and local poses. Runs after Avian3D physics so all consumers see current state. |
| `build_static_tf_maps` | `OnEnter(AppState::Running)` | Once | Builds the `name_to_entity` and `entity_to_name` maps from all `Name` + `TrackedFrame` entities. |

### World Frame Conversions (ENU ↔ Bevy)

| Function | Signature | Use Case |
|----------|-----------|----------|
| `enu_vector_to_bevy_vector` | `(v: &Vector3<f64>) -> BevyVec3` | ENU position/velocity vector → Bevy world vector |
| `bevy_vector_to_enu_vector` | `(v: &BevyVec3) -> Vector3<f64>` | Bevy world vector → ENU vector |
| `enu_quat_to_bevy_quat` | `(q: &UnitQuaternion<f64>) -> BevyQuat` | ENU orientation → Bevy world orientation |
| `bevy_quat_to_enu_quat` | `(q: &BevyQuat) -> UnitQuaternion<f64>` | Bevy orientation → ENU orientation |
| `enu_iso_to_bevy_transform` | `(pose: &Isometry3<f64>) -> BevyTransform` | Full ENU pose → Bevy `Transform`. Use for spawning agents. |
| `bevy_transform_to_enu_iso` | `(t: &BevyTransform) -> Isometry3<f64>` | Bevy `Transform` → ENU `Isometry3`. Use when reading physics state. |
| `bevy_global_transform_to_enu_iso` | `(gt: &GlobalTransform) -> Isometry3<f64>` | Bevy `GlobalTransform` → ENU `Isometry3`. Used by the TF tree rebuild. |

`enu_vector_as_bevy_point` and `bevy_point_as_enu_vector` are aliases for the vector variants — same math, different naming for clarity at call sites that deal with translation.

### Body/Sensor Frame Conversions (FLU ↔ Bevy local)

| Function | Signature | Use Case |
|----------|-----------|----------|
| `flu_vector_to_bevy_local_vector` | `(v: &Vector3<f64>) -> BevyVec3` | FLU direction (e.g. LiDAR ray) → Bevy local direction |
| `bevy_local_vector_to_flu_vector` | `(v: &BevyVec3) -> Vector3<f64>` | Bevy local direction → FLU direction |
| `flu_quat_to_bevy_local_quat` | `(q: &UnitQuaternion<f64>) -> BevyQuat` | FLU orientation → Bevy local orientation |
| `flu_iso_to_bevy_local_transform` | `(pose: &Isometry3<f64>) -> BevyTransform` | Full FLU pose → Bevy child `Transform`. Use for all sensor spawners. |

### Raw Isometry Utilities (no frame conversion)

| Function | Signature | Description |
|----------|-----------|-------------|
| `bevy_transform_to_nalgebra_isometry` | `(t: &BevyTransform) -> Isometry3<f64>` | Direct component copy, no axis remap. For relative transform math inside Bevy space. |
| `bevy_global_transform_to_nalgebra_isometry` | `(gt: &GlobalTransform) -> Isometry3<f64>` | Unwraps `GlobalTransform` then calls above. |

---

## Axis Conversion Table

### ENU ↔ Bevy World

| ENU | Bevy World |
|-----|-----------|
| +X (East) | +X |
| +Y (North) | -Z |
| +Z (Up) | +Y |

### FLU ↔ Bevy Local

| FLU | Bevy Local |
|-----|-----------|
| +X (Forward) | -Z |
| +Y (Left) | -X |
| +Z (Up) | +Y |

---

## Usage Examples

### Spawning an agent at an ENU pose

```rust
// In a SceneBuildSet::ProcessVehicle system:
let bevy_transform = enu_iso_to_bevy_transform(&agent_config.starting_pose.to_isometry());
commands.spawn((
    bevy_transform,
    RigidBody::Dynamic,
    TrackedFrame,
    Name::new(agent_config.name.clone()),
));
```

### Spawning a sensor as a child entity

```rust
// In a SceneBuildSet::ProcessSensors system:
// sensor_config.transform is a FLU Pose (body-relative)
let local_transform = flu_iso_to_bevy_local_transform(&sensor_config.transform.to_isometry());
commands.spawn((
    local_transform,
    TrackedFrame,
    Name::new(format!("{}/imu", agent_name)),
)).set_parent(vehicle_entity);
```

### Reading the TF tree in a sensor system

```rust
// In a SimulationSet::Sensors system:
fn lidar_system(tf_tree: Res<TfTree>, query: Query<(Entity, &LidarConfig)>) {
    for (entity, config) in &query {
        let Some(world_pose) = tf_tree.lookup_by_entity(entity) else {
            warn!("LiDAR entity missing from TF tree");
            continue;
        };
        // world_pose is Isometry3<f64> in ENU
    }
}
```

### Computing a sensor-to-sensor transform

```rust
// Compute the transform from camera frame to lidar frame:
let t = tf_tree.get_transform_by_name("truck/camera", "truck/lidar")
    .expect("both frames must be tracked");
// t = T_W_camera⁻¹ * T_W_lidar
// Applying t to a point in camera frame gives that point in lidar frame.
```

### Passing the TF tree to a helios_core pipeline

```rust
// TfTree implements TfProvider, so it can be passed directly:
pipeline.process_measurement(&measurement, &*tf_tree);
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Using `enu_iso_to_bevy_transform` for sensor child transforms | Sensor appears in wrong orientation/position | Use `flu_iso_to_bevy_local_transform` for all sensor spawners |
| Using `flu_iso_to_bevy_local_transform` for agent start poses | Agent spawns at wrong world position | Use `enu_iso_to_bevy_transform` for world-frame poses |
| Manual axis swap (`v.y = physics.z`) outside this file | Correct locally, breaks when convention changes | Call the appropriate helper function |
| Forgetting `TrackedFrame` on a sensor entity | `tf_tree.lookup_by_entity` returns `None`; sensor silently skips | Add `TrackedFrame` component at spawn time |
| Calling `bevy_transform_to_nalgebra_isometry` for ground truth | No frame conversion; physics state is in Bevy coords | Use `bevy_global_transform_to_enu_iso` instead |
