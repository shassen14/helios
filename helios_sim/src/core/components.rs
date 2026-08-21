use bevy::prelude::*;
use helios_core::control::actuators::ActuatorCommand;
use helios_core::data::primitives::FrameHandle;
use helios_core::estimation::dynamics::integrated_imu::ins_state_layout;
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
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

impl GroundTruthState {
    /// Converts physics ground truth into a `FrameAwareState` using the standard INS
    /// layout. The layout matches what the EKF produces — including its odom reference
    /// frame — so the controller sees an identical type. The oracle reports truth, but
    /// publishes it into the same odom-frame estimate slot the real filter fills.
    pub fn to_frame_aware_state(
        &self,
        agent_handle: FrameHandle,
        timestamp: f64,
    ) -> FrameAwareState {
        let body_frame = FrameId::Body(agent_handle);
        let odom_frame = FrameId::Odom(agent_handle);
        let layout = ins_state_layout(agent_handle);
        let mut state = FrameAwareState::new(layout, 1e-6, timestamp);
        let t = &self.pose.translation;
        state.set_variable(&StateVariable::Px(odom_frame.clone()), t.x);
        state.set_variable(&StateVariable::Py(odom_frame.clone()), t.y);
        state.set_variable(&StateVariable::Pz(odom_frame.clone()), t.z);
        let v = &self.linear_velocity;
        state.set_variable(&StateVariable::Vx(odom_frame.clone()), v.x);
        state.set_variable(&StateVariable::Vy(odom_frame.clone()), v.y);
        state.set_variable(&StateVariable::Vz(odom_frame.clone()), v.z);
        let q = self.pose.rotation.quaternion();
        state.set_variable(
            &StateVariable::Qx(body_frame.clone(), odom_frame.clone()),
            q.i,
        );
        state.set_variable(
            &StateVariable::Qy(body_frame.clone(), odom_frame.clone()),
            q.j,
        );
        state.set_variable(
            &StateVariable::Qz(body_frame.clone(), odom_frame.clone()),
            q.k,
        );
        state.set_variable(&StateVariable::Qw(body_frame, odom_frame), q.w);
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
