# Helios Coding & Engineering Standards

This document defines the technical rules and conventions for the Helios project. Following these standards ensures consistency, preventing bugs related to coordinate frames and units.

---

## 1. Coordinate Systems (The "Law of the Land")

We maintain two distinct coordinate universes. Data **must** be converted when crossing the boundary between them.

### A. The Robotics Frame (Core Logic)

All algorithms in `helios_core` and logical calculations in `helios_sim` use this frame.

- **World Standard:** **ENU** (East-North-Up).
  - `+X`: East
  - `+Y`: North
  - `+Z`: Up (Gravity acts along `-Z`)
- **Body/Sensor Standard:** **FLU** (Forward-Left-Up).
  - `+X`: Forward (Direction of motion)
  - `+Y`: Left
  - `+Z`: Up
- **Handedness:** Right-Handed.

### B. The Simulation Frame (Bevy Engine)

All rendering, physics (`Avian3D`), and gizmos use this frame.

- **Standard:** **Y-Up, -Z Forward**.
  - `+X`: Right
  - `+Y`: Up (Gravity acts along `-Y`)
  - `-Z`: Forward
- **Handedness:** Right-Handed.

### C. The Boundary

- **Conversion:** All conversions must happen via the helper functions in `helios_sim/src/simulation/core/transforms.rs`.
  - `enu_iso_to_bevy_transform(...)`
  - `robot_body_vec_to_bevy_local_vec(...)`
- **Rule:** Never perform raw coordinate swaps (e.g., `y = z`) manually in system logic. Always use the transform helpers.

---

## 2. Units of Measurement

We strictly adhere to **SI Units** for all internal logic and calculations.

| Quantity            | Unit                                 | Type  |
| :------------------ | :----------------------------------- | :---- |
| Distance / Position | **Meters (m)**                       | `f64` |
| Velocity            | **Meters per Second (m/s)**          | `f64` |
| Acceleration        | **Meters per Second Squared (m/s²)** | `f64` |
| Mass                | **Kilograms (kg)**                   | `f64` |
| Time                | **Seconds (s)**                      | `f64` |
| Angular Velocity    | **Radians per Second (rad/s)**       | `f64` |
| Orientation/Angle   | **Radians (rad)**                    | `f64` |
| Magnetic Field      | **Micro-Tesla (µT)**                 | `f64` |

### **The "Degrees" Exception**

- **User-Facing Configuration:** Inputs in `scenario.toml` (e.g., `horizontal_fov`, `max_steering_angle`, `starting_heading`) should be in **Degrees** for human readability.
- **Implementation:** These values must be converted to **Radians** immediately upon loading (in the `From` impl or the Spawning system) and stored as Radians in all runtime components.

---

## 3. Sensor Data Contracts

All sensors must output data wrapped in the `MeasurementData` enum found in `helios_core/src/messages.rs`.

- **IMU (`Imu6Dof` / `Imu9Dof`):**
  - **Frame:** Sensor Local (FLU).
  - **Acceleration:** Must be **Proper Acceleration** (includes gravity component, e.g., `+9.81` on Z when stationary).
- **GPS (`GpsPosition`):**
  - **Frame:** World (ENU).
  - **Reference:** Position of the antenna, not the vehicle center.
- **LiDAR (`PointCloud`):**
  - **Frame:** Sensor Local (FLU).
  - **Content:** Contains **only successful hits**. Misses (max-range rays) are excluded.
  - **Orientation:** `+X` is forward (center of scan).
- **Magnetometer (`Magnetometer`):**
  - **Frame:** Sensor Local (FLU).
  - **Reference:** Local magnetic field vector.

---

## 4. Coding Patterns

### Traits for Extensibility

- Use **Trait Objects** (`Box<dyn Trait>`) to define swappable logic (e.g., `EstimationDynamics`, `Measurement`, `Planner`).
- Avoid hard-coding specific algorithms in Bevy systems. The system should drive the _interface_, not the implementation.

### Error Handling

- **Logic Errors:** If a model receives incompatible data (e.g., GPS model receiving LiDAR data), it should return `None` or `Option::None`.
- **Runtime Warnings:** If a system cannot perform its task due to missing data (e.g., transform not found), use `warn!` and skip the entity for that frame. **Do not panic.**
- **Startup Panics:** It is acceptable to `panic!` or `expect()` during the **Spawning** phase if configuration is invalid (e.g., missing required file, invalid matrix dimensions). Fail fast at startup.

### System Sets & Ordering

- Always assign systems to a specific variant of `SimulationSet` (e.g., `.in_set(SimulationSet::Sensors)`).
- Do not rely on implicit system ordering.
