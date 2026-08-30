//! Typed representations of the scenario TOML — the vocabulary the `config`
//! loader deserializes and every subsystem reads.
//!
//! One curated facade: every submodule is private and this file re-exports the
//! exact public surface, so a type is always named `config::structs::Thing`,
//! never through a submodule path. Moving a type between files never breaks a
//! caller.

mod autonomy;
mod camera;
mod pose;
mod scenario;
mod sensors;
mod simulation;
mod terrain;
mod vehicle;
mod world_object;

// The whole `helios_runtime` config vocabulary is surfaced here unchanged. The
// glob *is* the curation ("expose all of runtime config under this facade"), not
// an omission — the one place a `*` re-export is intentional.
pub use autonomy::*;

pub use camera::CameraVantage;
pub use pose::Pose;
pub use scenario::{
    AgentConfig, RawScenarioConfig, ScenarioCommon, ScenarioConfig, Simulation, World,
};
pub use sensors::{GpsConfig, ImuConfig, LidarConfig, MagnetometerConfig, SensorConfig};
pub use simulation::MetricsConfig;
pub use terrain::{AtmosphereConfig, MagneticFieldConfig, TerrainConfig};
pub use vehicle::{
    AxleConfig, CollisionConfig, MountConfig, PlantConfig, SuspensionConfig, TireConfig,
    TopologyConfig, Vehicle, WheelConfig, CUBOID, L0_SHIM, RAYCAST_WHEELS, RIGID_BODY_WITH_MOUNT,
};
pub use world_object::{WorldObjectCollider, WorldObjectPlacement, WorldObjectPrefab};
