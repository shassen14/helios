use crate::core::run_clock::RunClock;

use helios_core::control::actuators::ActuatorCommand;
use helios_core::interchange::perception::semantic_class::SemanticClass;
use helios_core::prelude::{MonotonicTime, PlannerGoal};

use bevy::prelude::*;
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
    /// The run instant this state describes. Truth, sensors and the brain's
    /// tick are stamped with it, so the instant travels with the data rather
    /// than with which set read the clock.
    pub at: MonotonicTime,
    pub pose: Isometry3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    /// The mean over the step that ended at `at`, a backward difference of
    /// velocity, so it lags `at` by half a step. Under explicit force
    /// integration the acceleration at `at` isn't known until the next
    /// step's forces are.
    pub linear_acceleration: Vector3<f64>,
    /// The mean over the step that ended at `at`, as `linear_acceleration`.
    pub angular_acceleration: Vector3<f64>,
    pub last_linear_velocity: Vector3<f64>,
    pub last_angular_velocity: Vector3<f64>,
}

impl Default for GroundTruthState {
    fn default() -> Self {
        Self {
            at: RunClock::START,
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
