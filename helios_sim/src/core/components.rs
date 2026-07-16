use bevy::prelude::*;
use helios_core::data::primitives::FrameHandle;
use helios_core::frames::{
    layout::standard_ins_state_layout, FrameAwareState, FrameId, StateVariable,
};
use helios_core::prelude::{ControlOutput, PlannerGoal};
use nalgebra::{Isometry3, Vector3};
use serde::Serialize;

// --- Controller Output Component ---

/// The output of the last `Controller::compute()` call.
/// Written by `SimulationSet::BrainOutput`; read by `SimulationSet::Actuation`.
#[derive(Component)]
pub struct ControlOutputComponent(pub ControlOutput);

/// Selects which state estimate the controller reads. Toggled by the HUD's T key.
#[derive(Component, Clone, Debug, PartialEq, Default)]
pub enum ControllerStateSource {
    #[default]
    GroundTruth,
    Estimated,
}

// --- Agent State Components ---

/// The perfect, physics-driven ground truth state of an agent.
/// Written by the StateSync system; read by sensors, vehicle adapters, and
/// the ground-truth (oracle) publisher.
#[derive(Component, Clone, Debug, Serialize)]
pub struct GroundTruthState {
    pub pose: Isometry3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub linear_acceleration: Vector3<f64>,
    pub last_linear_velocity: Vector3<f64>,
}

impl Default for GroundTruthState {
    fn default() -> Self {
        Self {
            pose: Isometry3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            linear_acceleration: Vector3::zeros(),
            last_linear_velocity: Vector3::zeros(),
        }
    }
}

impl GroundTruthState {
    /// Converts physics ground truth into a `FrameAwareState` using the standard INS layout.
    /// The layout matches what the EKF produces, so the controller sees an identical type.
    pub fn to_frame_aware_state(
        &self,
        agent_handle: FrameHandle,
        timestamp: f64,
    ) -> FrameAwareState {
        let body_frame = FrameId::Body(agent_handle);
        let world_frame = FrameId::World;
        let layout = standard_ins_state_layout(agent_handle);
        let mut state = FrameAwareState::new(layout, 1e-6, timestamp);
        let t = &self.pose.translation;
        state.set_variable(&StateVariable::Px(FrameId::World), t.x);
        state.set_variable(&StateVariable::Py(FrameId::World), t.y);
        state.set_variable(&StateVariable::Pz(FrameId::World), t.z);
        let v = &self.linear_velocity;
        state.set_variable(&StateVariable::Vx(FrameId::World), v.x);
        state.set_variable(&StateVariable::Vy(FrameId::World), v.y);
        state.set_variable(&StateVariable::Vz(FrameId::World), v.z);
        let q = self.pose.rotation.quaternion();
        state.set_variable(
            &StateVariable::Qx(body_frame.clone(), world_frame.clone()),
            q.i,
        );
        state.set_variable(
            &StateVariable::Qy(body_frame.clone(), world_frame.clone()),
            q.j,
        );
        state.set_variable(
            &StateVariable::Qz(body_frame.clone(), world_frame.clone()),
            q.k,
        );
        state.set_variable(&StateVariable::Qw(body_frame, world_frame), q.w);
        state
    }
}

#[derive(Component, Clone)]
pub struct ConfiguredMissionGoal(pub PlannerGoal);

#[derive(Component)]
pub struct GoalDispatched;

// =========================================================================
// == World Object Components ==
// =========================================================================

/// Identifies the prefab catalog key for a world object entity (e.g. `"objects.stop_sign"`).
/// Useful for runtime queries such as "which objects of type X are near the agent?".
#[derive(Component, Clone, Debug)]
pub struct WorldObjectType(pub String);

/// Semantic classification attached to every world object entity.
/// Used by perception systems for ground-truth labeling and by dataset exporters.
#[derive(Component, Clone, Debug)]
pub struct SemanticLabel {
    /// Human-readable class name, e.g. `"stop_sign"`, `"building"`.
    pub label: String,
    /// Integer class ID used by perception algorithms and ML dataset formats.
    pub class_id: u32,
}

/// Axis-aligned bounding box in object-local space (half-extents, meters).
/// Used for debug visualization, sensor hit attribution, and dataset annotation.
#[derive(Component, Clone, Debug)]
pub struct BoundingBox3D {
    /// Half-extents [hx, hy, hz] along the object's local X/Y/Z axes.
    pub half_extents: Vec3,
}

/// Identifies the physics medium represented by a terrain entity.
/// Agents and sensors can query this to determine applicable physics rules
/// (drag, buoyancy, sensor propagation) for their current environment.
///
/// Recognised values: `"air"`, `"water"`, `"vacuum"`.
#[derive(Component, Clone, Debug)]
pub struct TerrainMedium(pub String);
