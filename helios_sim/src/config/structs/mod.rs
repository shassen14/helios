//! Typed representations of the scenario TOML — the vocabulary the `config`
//! loader deserializes and every subsystem reads.
//!
//! One curated facade: every submodule is private and this file re-exports the
//! exact public surface, so a type is always named `config::structs::Thing`,
//! never through a submodule path. Moving a type between files never breaks a
//! caller.

mod autonomy;
mod camera;
mod conditions;
mod pose;
mod scenario;
mod sensors;
mod simulation;
mod vehicle;
mod world_layout;

// The whole `helios_runtime` config vocabulary is surfaced here unchanged. The
// glob *is* the curation ("expose all of runtime config under this facade"), not
// an omission — the one place a `*` re-export is intentional.
pub use autonomy::*;

pub use camera::CameraVantage;
pub use conditions::{AtmosphereConfig, MagneticFieldConfig};
pub use pose::Pose;
pub use scenario::{
    AgentConfig, RawScenarioConfig, ScenarioCommon, ScenarioConfig, Simulation, World,
};
pub use sensors::{
    GpsConfig, ImuConfig, LidarConfig, MagnetometerConfig, SensorDeviceConfig,
    SensorInstallationConfig, ACCEL_CHANNEL_FIELD, CHANNEL_FIELD, GYRO_CHANNEL_FIELD,
};
pub use simulation::MetricsConfig;
pub use vehicle::{
    AxleConfig, BoxVisual, CollisionConfig, MountConfig, PlantConfig, SuspensionConfig, TireConfig,
    TopologyConfig, Vehicle, VisualConfig, WheelConfig, WheelVisual, CUBOID, RAYCAST_WHEELS,
    RIGID_BODY_WITH_MOUNT, WHEELED_PRIMITIVES,
};
pub use world_layout::{BodyKind, CatalogRef, ObjectPlacement, ObjectPrefab, WorldLayout};
