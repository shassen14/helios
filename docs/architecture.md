# Helios Architecture

This document describes the high-level design, data flow, and structural philosophy of the Helios robotics simulation platform.

## 1. High-Level Concept

Helios is a modular, data-driven simulation platform designed to bridge the gap between high-fidelity physics simulation and autonomous robotics algorithms.

It treats the simulation loop as a closed feedback system:

1.  **Sense:** Physics simulation generates raw sensor data.
2.  **Think:** Algorithms fuse data, build maps, and plan paths.
3.  **Act:** Controllers generate forces/torques to drive the physics engine.

### System Diagram

```mermaid
graph TD
    %% --- THE PHYSICAL WORLD (Simulation) ---
    subgraph "Physics & Environment (Avian3D)"
        Physics[Rigid Body Dynamics]
        Env["Environment (Map & Dynamic Actors)"]
    end

    %% --- SENSING ---
    subgraph "Sensor Suite (helios_sim)"
        IMU[IMU Plugin]
        GPS[GPS Plugin]
        LiDAR[LiDAR Plugin]
        Cam[Camera Plugin]
    end

    %% --- PERCEPTION (NEW) ---
    subgraph "Perception & Tracking Layer"
        Tracker[Object Tracker]
        Detector[Object Detector]
    end

    %% --- THE WORLD MODEL (The "Brain") ---
    subgraph "World Model (helios_core + sim adapter)"
        InputGatherer(Input Gatherer)

        subgraph "Internal Logic (Mutually Exclusive)"
            direction TB
            Separate["Strategy A: Separate Modules<br/>(Estimator + Mapper)"]
            SLAM["Strategy B: Unified System<br/>(SLAM)"]
        end

        Publisher(Output Publisher)
    end

    %% --- ACTION ---
    subgraph "Decision Making"
        Planner[Motion Planner]
        Controller[Controller]
    end

    %% --- DATA FLOW ---

    %% 1. Physics to Sensors
    Physics -->|Ground Truth State| IMU
    Physics -->|Ground Truth State| GPS
    Physics -.->|Raycasting/Rendering| LiDAR
    Env -.->|Raycasting/Rendering| LiDAR
    Physics -.->|Rendering| Cam
    Env -.->|Rendering| Cam

    %% 2. Sensors to Perception & World Model
    IMU -->|MeasurementMessage| InputGatherer
    GPS -->|MeasurementMessage| InputGatherer
    LiDAR -->|MeasurementMessage| InputGatherer

    %% NEW: Perception Flow
    LiDAR -->|Raw Data| Detector
    Cam -->|Raw Data| Detector
    Detector -->|Detections| Tracker
    Tracker -->|MeasurementMessage - TrackedObject| InputGatherer
    Tracker == "/objects/tracked" ==> Planner

    %% 3. Inside World Model
    InputGatherer --> Separate
    InputGatherer --> SLAM
    Separate --> Publisher
    SLAM --> Publisher

    %% 4. World Model Outputs (The API)
    Publisher == "/odometry/estimated" ==> Planner
    Publisher == "/odometry/estimated" ==> Controller
    Publisher == "/map" ==> Planner

    %% 5. Planning & Control
    Planner -->|Path| Controller
    Controller -->|Forces/Torques| Physics
```

---

## 2. Project Structure: Core vs. Sim

The project is split into two distinct crates to enforce a strict separation between **logic** and **implementation**.

### `helios_core` (The Algorithm Toolbox)

- **Role:** Contains pure, framework-agnostic robotics algorithms and math.
- **Dependencies:** Minimal (e.g., `nalgebra`). It **does not** depend on Bevy.
- **Contents:**
  - **Traits:** `StateEstimator`, `Mapper`, `SlamSystem`, `Dynamics`, `Measurement`.
  - **Models:** Concrete implementations of physics models (`IntegratedImuModel`) and sensor models (`GpsModel`).
  - **Data Structures:** Standardized messages like `MeasurementMessage`, `Odometry`, and `PointCloud`.
- **Philosophy:** This code could theoretically run on a real robot.

### `helios_sim` (The Simulation Engine)

- **Role:** The host application that runs the simulation loop, visualization, and physics.
- **Dependencies:** `bevy`, `avian3d`, `helios_core`.
- **Contents:**
  - **Plugins:** Bevy plugins for Sensors, World Modeling, Planning, and Control.
  - **Systems:** ECS systems that orchestrate data flow (e.g., `raycasting_sensor_system`, `world_model_processor`).
  - **Adapters:** Code that translates Bevy types (Transforms) into Core types (Isometries).

---

## 3. The Data Pipeline

Data flows through the system in a strictly ordered pipeline, managed by Bevy's `FixedUpdate` schedule.

### Stage 1: Physics & Ground Truth

- **Engine:** `Avian3D` calculates rigid body dynamics.
- **Sync:** A `StateSync` system reads the final physics state (Pose, Velocity) and converts it to the standardized ENU coordinate frame, storing it in a `GroundTruthState` component.

### Stage 2: Sensing & Perception

- **Producers:** Sensor plugins (`ImuPlugin`, `GpsPlugin`) read the `GroundTruthState`.
- **Raycasting:** The `RaycastingSensorPlugin` performs physics queries for LiDAR/Radar.
- **Output:** All sensors publish standardized `BevyMeasurementMessage` events containing pure `helios_core` data structs.

### Stage 3: The World Model (Estimation & Mapping)

- **Role:** The central "brain" that fuses sensor data.
- **Configuration:** Configured via `scenario.toml` to run in one of two modes:
  - **Separate:** Runs an `Estimator` (EKF/UKF) and a `Mapper` (Occupancy Grid) independently.
  - **Combined:** Runs a `SLAM` system (Factor Graph) that handles both.
- **Orchestration:**
  - An **Event-Driven Processor** runs the high-frequency estimator (e.g., INS filter) as fast as data arrives.
  - A **Fixed-Rate System** runs the computationally expensive Mapper or SLAM optimizer at a lower, user-defined rate (e.g., 5Hz).
- **Output:** Publishes `/odometry/estimated` and `/map` topics.

### Stage 4: Decision Making (Planning & Control)

- **Planner:** Consumes `/map` and `/odometry/estimated` to generate a `/path/desired`.
- **Controller:** Consumes `/path/desired` and `/odometry/estimated` to calculate control inputs (Throttle, Steering).
- **Actuation:** The final system applies these controls as forces/torques to the physics engine, closing the loop.

---

## 4. Key Architectural Patterns

### The "Trait Object" Pattern

We use Rust traits (`Box<dyn Trait>`) to decouple the simulator from specific algorithms.

- The `WorldModelPlugin` does not know it is running an "EKF." It only knows it holds a `Box<dyn StateEstimator>`.
- This allows us to swap algorithms (e.g., switch from EKF to Particle Filter) just by changing a line in the configuration file.

### The "Configuration-Driven" Pattern

- **Source of Truth:** The `scenario.toml` file defines the entire simulation.
- **Spawners:** Bevy systems read this config at startup to compose the entity's components.
- **No Hard-Coding:** Noise levels, update rates, and model choices are always injected from the config.

### The "Coordinate Frame" Boundary

- **Bevy Side:** Uses Left-Handed Y-Up (Graphics standard).
- **Core Side:** Uses Right-Handed Z-Up (Robotics standard: ENU/FLU).
- **The Wall:** We enforce a strict boundary. All data crossing from `sim` to `core` is converted to ENU. All data crossing back for visualization is converted to Bevy-frame.
