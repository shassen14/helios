use helios_core::control::actuation_model::ActuationModel;
use serde::Deserialize;

pub const RIGID_BODY_WITH_MOUNT: &str = "RigidBodyWithMount";

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

impl TopologyConfig {
    pub fn kind_str(&self) -> &'static str {
        match self {
            TopologyConfig::RigidBodyWithMount { .. } => RIGID_BODY_WITH_MOUNT,
        }
    }
}

pub const L0_SHIM: &str = "L0Shim";

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

impl PlantConfig {
    pub fn kind_str(&self) -> &'static str {
        match self {
            PlantConfig::L0Shim { .. } => L0_SHIM,
        }
    }
}

pub const CUBOID: &str = "Cuboid";

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

impl CollisionConfig {
    pub fn kind_str(&self) -> &'static str {
        match self {
            CollisionConfig::Cuboid { .. } => CUBOID,
        }
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    use figment::{
        providers::{Format, Toml},
        Figment,
    };

    // Each `kind_str` must return the exact string a TOML author writes as
    // `kind = "..."`. serde derives that tag from the variant identifier, so the
    // const and the variant name are two independent forms of one literal; these
    // round-trips are the guard that they never drift apart — a rename on one side
    // without the other fails here rather than at an agent spawn.
    fn parse<T: serde::de::DeserializeOwned>(toml: &str) -> T {
        Figment::new().merge(Toml::string(toml)).extract().unwrap()
    }

    #[test]
    fn topology_kind_str_matches_serde_tag() {
        let cfg: TopologyConfig = parse("kind = \"RigidBodyWithMount\"\nmass = 1500.0");
        assert_eq!(cfg.kind_str(), RIGID_BODY_WITH_MOUNT);
    }

    #[test]
    fn plant_kind_str_matches_serde_tag() {
        let cfg: PlantConfig = parse(
            "kind = \"L0Shim\"\n\
             l0_force_gain = 1.0\n\
             l0_yaw_gain = 1.0\n\
             linear_damping = 0.0\n\
             angular_damping = 0.0",
        );
        assert_eq!(cfg.kind_str(), L0_SHIM);
    }

    #[test]
    fn collision_kind_str_matches_serde_tag() {
        let cfg: CollisionConfig =
            parse("kind = \"Cuboid\"\nx = 1.0\ny = 1.0\nz = 1.0\nfriction = 0.5");
        assert_eq!(cfg.kind_str(), CUBOID);
    }
}
