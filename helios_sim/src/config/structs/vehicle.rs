use super::Pose;

use helios_core::control::actuation_model::ActuationModel;

use serde::Deserialize;

pub const RIGID_BODY_WITH_MOUNT: &str = "RigidBodyWithMount";

/// A named attachment point on a body — a wheel hub, a sensor boom, a hardpoint.
/// Pure pose in the body's FLU frame: a mount says *where*, never *what* mounts
/// there or *what role* it plays. Downstream axes resolve that by `name` — the
/// visual anchors a part to it, the plant reads which mounts steer or drive — so
/// a mount carries no geometry or role of its own.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct MountConfig {
    pub name: String,
    pub pose: Pose,
}

/// Body structure: which rigid bodies the solver integrates and the frames
/// things mount to. `RigidBodyWithMount` is the general single-body case (one
/// solver body plus zero or more named mount frames) shared by every
/// non-articulated agent; articulated bodies get their own variant. Mounts are
/// optional — an agent that attaches nothing declares none.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(deny_unknown_fields)]
pub enum TopologyConfig {
    RigidBodyWithMount {
        mass: f32,
        #[serde(default, rename = "mount")]
        mounts: Vec<MountConfig>,
    },
}

impl TopologyConfig {
    pub fn kind_str(&self) -> &'static str {
        match self {
            TopologyConfig::RigidBodyWithMount { .. } => RIGID_BODY_WITH_MOUNT,
        }
    }

    pub fn mounts(&self) -> &[MountConfig] {
        match self {
            TopologyConfig::RigidBodyWithMount { mounts, .. } => mounts,
        }
    }
}

/// Suspension geometry and spring-damper constants, shared by every wheel.
/// SI throughout; lengths in m, `stiffness` N/m, `damping` N·s/m.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct SuspensionConfig {
    pub rest_length: f32,
    pub wheel_radius: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub max_travel: f32,
    pub ray_margin: f32,
}

/// Friction-circle tire constants, shared by every wheel. Cornering stiffness
/// splits front/rear so their ratio sets the understeer balance; `rolling_resistance`
/// is dimensionless, `low_speed_threshold` in m/s and must be > 0 (it divides the
/// low-speed regularizer — enforced at load, not here).
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct TireConfig {
    pub cornering_stiffness_front: f32,
    pub cornering_stiffness_rear: f32,
    pub rolling_resistance: f32,
    pub low_speed_threshold: f32,
}

/// One wheel: the mount frame it rides on (matched by `name`, pose single-sourced
/// on that `[[mount]]`), its axle, and its roles. Carries no offset of its own.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct WheelConfig {
    pub mount: String,
    pub axle: AxleConfig,
    #[serde(default)]
    pub steer: bool,
    #[serde(default)]
    pub drive: bool,
}

/// Which axle a wheel belongs to — selects the front/rear cornering stiffness.
/// Deserializes from the bare string an author writes (`axle = "Front"`).
#[derive(Debug, Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub enum AxleConfig {
    Front,
    Rear,
}

pub const RAYCAST_WHEELS: &str = "RaycastWheels";

/// Plant fidelity: how setpoints and state become forces on the body.
///
/// `RaycastWheels` is the four-corner raycast model — per-wheel spring-damper
/// suspension and a friction-circle tire, so weight transfer and per-surface
/// grip emerge from the model rather than a scalar gain.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(deny_unknown_fields)]
pub enum PlantConfig {
    RaycastWheels {
        suspension: SuspensionConfig,
        tire: TireConfig,
        #[serde(rename = "wheel")]
        wheels: Vec<WheelConfig>,
    },
}

impl PlantConfig {
    pub fn kind_str(&self) -> &'static str {
        match self {
            PlantConfig::RaycastWheels { .. } => RAYCAST_WHEELS,
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

pub const WHEELED_PRIMITIVES: &str = "WheeledPrimitives";

/// Cosmetic body geometry — what the agent looks like, never what the solver
/// collides or what a sensor perceives. Deliberately its own axis: a visual mesh
/// can differ from the collider and from the plant's contact geometry (a
/// low-poly wheel over a cylinder collider over a single suspension ray).
/// `WheeledPrimitives` draws a chassis box plus one wheel cylinder per mount
/// frame, the primitive stand-in until a mesh-asset variant lands.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(deny_unknown_fields)]
pub enum VisualConfig {
    WheeledPrimitives {
        chassis: BoxVisual,
        wheel: WheelVisual,
    },
}

impl VisualConfig {
    pub fn kind_str(&self) -> &'static str {
        match self {
            VisualConfig::WheeledPrimitives { .. } => WHEELED_PRIMITIVES,
        }
    }
}

/// The chassis box: full side lengths in the Bevy body frame (x = width,
/// y = height, z = length), construction-space, matching the collision cuboid's
/// convention. A separate value from `[collision]` on purpose — the drawn body
/// and the collider are independent axes. `color` is linear sRGBA; an alpha below
/// 1.0 renders translucent (so the wheels tucked under the body stay visible).
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct BoxVisual {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub color: [f32; 4],
}

/// One wheel drawn as a cylinder: `radius` and axle-axis `width`, both in metres,
/// `color` linear sRGBA. `drop` lowers the drawn hub below its mount frame: the
/// mount is the suspension pickup (strut top / ray origin), and the wheel centre
/// hangs roughly the rest suspension extension beneath it, so a wheel drawn at
/// the mount would sit up inside the body. `radius`/`drop` relate to the plant's
/// `suspension` values but are authored independently — the visual is a
/// projection of the body, not a reader of the plant.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct WheelVisual {
    pub radius: f32,
    pub width: f32,
    pub drop: f32,
    pub color: [f32; 4],
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
    pub visual: VisualConfig,
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

    // `[[mount]]` renames to the `mounts` field and each entry nests a `Pose`.
    // Parsing here is the guard that the rename and the nested pose table stay
    // wired; the pose's own array/degree decoding is tested where `Pose` lives.
    #[test]
    fn topology_parses_mount_list() {
        let cfg: TopologyConfig = parse(
            "kind = \"RigidBodyWithMount\"\n\
             mass = 1500.0\n\
             [[mount]]\n\
             name = \"wheel_fl\"\n\
             pose = { translation = [1.3, 0.75, 0.0], rotation = [0.0, 0.0, 0.0] }",
        );
        let TopologyConfig::RigidBodyWithMount { mounts, .. } = cfg;
        assert_eq!(mounts.len(), 1);
        assert_eq!(mounts[0].name, "wheel_fl");
    }

    // A body that declares no `[[mount]]` still parses — the field defaults to an
    // empty list, the shape a single-body agent with no wheel frames relies on.
    #[test]
    fn topology_mounts_default_to_empty() {
        let cfg: TopologyConfig = parse("kind = \"RigidBodyWithMount\"\nmass = 1500.0");
        let TopologyConfig::RigidBodyWithMount { mounts, .. } = cfg;
        assert!(mounts.is_empty());
    }

    // The plant tag must round-trip: `RAYCAST_WHEELS` and the `RaycastWheels`
    // variant name are two forms of one literal and must never drift apart.
    #[test]
    fn plant_raycast_kind_str_matches_serde_tag() {
        let cfg: PlantConfig = parse(RAYCAST_WHEELS_TOML);
        assert_eq!(cfg.kind_str(), RAYCAST_WHEELS);
    }

    // The nested sub-tables and the `[[wheel]]` -> `wheels` rename stay wired, and
    // each row lands with its mount, axle, and roles. The tire/suspension scalars
    // are decoded by serde and not re-asserted here.
    #[test]
    fn plant_parses_raycast_wheel_rows() {
        let cfg: PlantConfig = parse(RAYCAST_WHEELS_TOML);
        let PlantConfig::RaycastWheels { wheels, .. } = cfg else {
            panic!("expected RaycastWheels");
        };
        assert_eq!(wheels.len(), 2);
        assert_eq!(wheels[0].mount, "wheel_fl");
        assert!(matches!(wheels[0].axle, AxleConfig::Front));
        assert!(wheels[0].steer);
        assert!(!wheels[0].drive);
        assert!(matches!(wheels[1].axle, AxleConfig::Rear));
        assert!(wheels[1].drive);
    }

    #[test]
    fn visual_kind_str_matches_serde_tag() {
        let cfg: VisualConfig = parse(VISUAL_TOML);
        assert_eq!(cfg.kind_str(), WHEELED_PRIMITIVES);
    }

    // The chassis and wheel sub-tables decode into their dims structs, including the
    // color array and the wheel `drop`. The scalars are plain serde and not
    // re-asserted exhaustively — this is the guard that the nested-table shape stays
    // wired, not a value check.
    #[test]
    fn visual_parses_chassis_and_wheel_dims() {
        let cfg: VisualConfig = parse(VISUAL_TOML);
        let VisualConfig::WheeledPrimitives { chassis, wheel } = cfg;
        assert_eq!(chassis.z, 4.0);
        assert_eq!(chassis.color[3], 0.35);
        assert_eq!(wheel.radius, 0.3);
        assert_eq!(wheel.drop, 0.3);
    }

    const VISUAL_TOML: &str = "kind = \"WheeledPrimitives\"\n\
         chassis = { x = 1.8, y = 0.8, z = 4.0, color = [0.2, 0.4, 0.8, 0.35] }\n\
         wheel = { radius = 0.3, width = 0.2, drop = 0.3, color = [0.05, 0.05, 0.05, 1.0] }";

    // A front steering wheel and a rear drive wheel — exercises both axles, both
    // roles, and the sub-tables in one fixture.
    const RAYCAST_WHEELS_TOML: &str = "kind = \"RaycastWheels\"\n\
         [suspension]\n\
         rest_length = 0.40\n\
         wheel_radius = 0.30\n\
         stiffness = 50000.0\n\
         damping = 4000.0\n\
         max_travel = 0.20\n\
         ray_margin = 0.10\n\
         [tire]\n\
         cornering_stiffness_front = 80000.0\n\
         cornering_stiffness_rear = 60000.0\n\
         rolling_resistance = 0.015\n\
         low_speed_threshold = 1.0\n\
         [[wheel]]\n\
         mount = \"wheel_fl\"\n\
         axle = \"Front\"\n\
         steer = true\n\
         drive = false\n\
         [[wheel]]\n\
         mount = \"wheel_rl\"\n\
         axle = \"Rear\"\n\
         steer = false\n\
         drive = true";
}
