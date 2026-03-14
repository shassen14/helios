### **System Design: Configuration & Asset Loading**

**Purpose:**
To allow complex scenarios to be defined in human-readable files, supporting reusability and "prefab" agents.

**Architecture:**

1.  **The `Catalog` Concept:**
    - Instead of one giant file, we support a library of "parts."
    - `configs/catalog/sensors/`: Pre-configured sensors (e.g., `velodyne_vlp16.toml`).
    - `configs/catalog/vehicles/`: Vehicle physics parameters.
    - `configs/catalog/agents/`: Valid robot configurations (Vehicle + Sensors).

2.  **The `Resolver`:**
    - A build-time or startup-time system that reads the `scenario.toml`.
    - When it encounters a string like `sensor = "velodyne_vlp16"`, it looks up that file in the catalog and injects the parameters.
    - This flattened, resolved configuration is what the `helios_sim` plugins actually consume (`SpawnAgentConfigRequest`).

3.  **Strict Typing (`config/structs/`):**
    - Every config file maps 1-to-1 to a Rust struct, split across focused modules:
      - `pose.rs` — `Pose`
      - `vehicle.rs` — `Vehicle`, `AckermannActuatorConfig`
      - `sensors.rs` — `SensorConfig`, `ImuConfig`, `GpsConfig`, `MagnetometerConfig`, `LidarConfig`
      - `autonomy.rs` — `AutonomyStack`, `WorldModelConfig`, `EstimatorConfig`, `EkfDynamicsConfig`, `MapperConfig`, `PlannerConfig`, `ControllerConfig`
      - `scenario.rs` — `ScenarioConfig`, `RawScenarioConfig`, `Simulation`, `World`, `AgentConfig`
    - All types are re-exported from `config::structs` so import paths are unchanged.
    - We use `serde` attributes to enforce unit correctness (e.g., degrees vs radians) at the parsing level, so runtime code never has to guess units.
