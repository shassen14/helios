use bevy::prelude::*;
use helios_core::control::actuators::ActuatorCommand;
use helios_core::interchange::perception::semantic_class::SemanticClass;
use helios_core::prelude::PlannerGoal;
use nalgebra::{Isometry3, Vector3};
use serde::Serialize;

// --- Actuator Command Component ---

/// The pipeline's latest actuator-terminal output — the per-actuator setpoints
/// (`ActuatorCommand`) the allocator node produces. Written by
/// `SimulationSet::BrainOutput`; read by the vehicle plugin in
/// `SimulationSet::Actuation`, which applies each setpoint to physics.
#[derive(Component)]
pub struct ActuatorCommandComponent(pub ActuatorCommand);

/// Selects which state estimate the controller reads. Toggled by the HUD's T key.
#[derive(Component, Clone, Debug, PartialEq, Default)]
pub enum ControllerStateSource {
    #[default]
    GroundTruth,
    Estimated,
}

// --- Agent State Components ---

/// The perfect, physics-driven ground truth state of an agent.
/// Written by the StateSync system; read by sensors, the vehicle plugin, and
/// the ground-truth (oracle) publisher.
#[derive(Component, Clone, Debug, Serialize)]
pub struct GroundTruthState {
    pub pose: Isometry3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub linear_acceleration: Vector3<f64>,
    pub angular_acceleration: Vector3<f64>,
    pub last_linear_velocity: Vector3<f64>,
    pub last_angular_velocity: Vector3<f64>,
}

impl Default for GroundTruthState {
    fn default() -> Self {
        Self {
            pose: Isometry3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            linear_acceleration: Vector3::zeros(),
            angular_acceleration: Vector3::zeros(),
            last_linear_velocity: Vector3::zeros(),
            last_angular_velocity: Vector3::zeros(),
        }
    }
}

#[derive(Component, Clone)]
pub struct ConfiguredMissionGoal(pub PlannerGoal);

#[derive(Component)]
pub struct GoalDispatched;

// =========================================================================
// == World Object Components ==
// =========================================================================

/// Identifies the prefab catalog key for a world object entity (e.g. `"entities.objects.crate_1m"`).
/// Useful for runtime queries such as "which objects of type X are near the agent?".
#[derive(Component, Clone, Debug)]
pub struct WorldObjectType(pub String);

/// A world object's identity in ground truth, `<layout>/<placement>` (e.g.
/// `yard/crate_a`). Stable across runs and list order; unlike `Name`, which
/// is only a debug label, this is the key datasets are joined on.
#[derive(Component, Clone, Debug, PartialEq, Eq, Hash)]
pub struct ObjectInstanceId(pub String);

/// A world object's semantic class, from the scenario's class catalog.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ObjectClass(pub SemanticClass);

/// Axis-aligned bounding box in the entity's local frame (Bevy axes, meters).
/// Used for debug visualization, sensor hit attribution, and dataset annotation.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct BoundingBox3D {
    /// Box centre relative to the entity's origin. Not zero in general: an
    /// object standing on its origin has its centre half its height up.
    pub centre: Vec3,
    /// Half-extents along the entity's local X/Y/Z axes.
    pub half_extents: Vec3,
}

/// Identifies the physics medium represented by a terrain entity.
/// Agents and sensors can query this to determine applicable physics rules
/// (drag, buoyancy, sensor propagation) for their current environment.
///
/// Recognised values: `"air"`, `"water"`, `"vacuum"`.
#[derive(Component, Clone, Debug)]
pub struct TerrainMedium(pub String);
