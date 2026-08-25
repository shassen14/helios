use helios_core::control::actuation_model::ActuationModel;
use serde::Deserialize;

/// Body structure: which rigid bodies the solver integrates and the frames
/// things mount to. `RigidBodyWithMount` is the general single-body case (one
/// solver body plus, in future, named mount frames) shared by every
/// non-articulated agent; articulated bodies get their own variant.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(deny_unknown_fields)]
pub enum TopologyConfig {
    RigidBodyWithMount { mass: f32 },
}

/// Plant fidelity: how setpoints and state become forces on the body.
///
/// `L0Shim` is the arcade shim — open-loop feedforward gains that scale a
/// resolved setpoint into a chassis wrench, plus passive damping that stands in
/// for a real resistive force. It is retired wholesale when a raycast/dynamic
/// plant supplies real forces.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(deny_unknown_fields)]
pub enum PlantConfig {
    L0Shim {
        l0_force_gain: f32,
        l0_yaw_gain: f32,
        linear_damping: f32,
        angular_damping: f32,
    },
}

/// Physics collider shape and contact material — the geometry the solver
/// collides, distinct from the visual mesh and from any perception geometry.
/// `Cuboid` dims are full side lengths in the Bevy body frame (x = width,
/// y = height, z = length), construction-space, not ENU.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(deny_unknown_fields)]
pub enum CollisionConfig {
    Cuboid {
        x: f32,
        y: f32,
        z: f32,
        friction: f32,
    },
}

/// One agent body, decomposed into independent embodiment axes. Each axis is a
/// tagged enum keyed on its own `kind`, so a new agent type extends an axis
/// rather than reshaping this struct.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct Vehicle {
    pub topology: TopologyConfig,
    pub plant: PlantConfig,
    pub collision: CollisionConfig,
    pub actuation: ActuationModel,
}
