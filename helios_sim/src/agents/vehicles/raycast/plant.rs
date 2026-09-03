use helios_core::{
    frames::{conventions::Flu, quantities::Point},
    plant::{Axle, RaycastWheelPlant, SuspensionParams, TireParams, Wheel},
};

use bevy::prelude::*;

use crate::{
    agents::vehicles::components::ActuationModelComponent,
    config::structs::{AxleConfig, MountConfig, PlantConfig, WheelConfig},
    registry::contexts::PlantBuildContext,
};

/// The L1 raycast plant on an agent — the host's handle to its portable
/// [`RaycastWheelPlant`]. Wraps the core model (wheel rays → per-wheel tire forces
/// → chassis wrench) so the host holds a Bevy component while the physics stays in
/// `helios_core`; the apply system reads it each tick to cast the rays and apply
/// the resulting wrench.
#[derive(Component)]
pub struct RaycastWheelPlantComponent(pub RaycastWheelPlant);

/// Builds the L1 raycast plant: resolves each configured wheel to its mount pose,
/// widens the suspension and tire constants into the core parameter types, and
/// deposits the [`RaycastWheelPlantComponent`] the apply system drives. Unlike the
/// L0 shim it inserts no passive damping — the tire model produces real rolling
/// resistance, so adding damping here would double-count it. A bad threshold, an
/// unresolved mount, or an actuation kind the plant cannot apply fails the spawn.
pub fn build_raycast_wheels(ctx: &mut PlantBuildContext) -> Result<(), String> {
    // `RaycastWheels` is the only plant variant today, so the pattern is
    // irrefutable; the `else` guards the seam for the fidelity variants to come,
    // when this builder must reject a config meant for a different plant.
    #[allow(irrefutable_let_patterns)]
    let PlantConfig::RaycastWheels {
        suspension,
        tire,
        wheels,
    } = ctx.config
    else {
        return Err(format!(
            "RaycastWheels builder received a `{}` plant",
            ctx.config.kind_str(),
        ));
    };

    if tire.low_speed_threshold <= 0.0 {
        return Err(format!(
            "low_speed_threshold must be > 0, got {}",
            tire.low_speed_threshold,
        ));
    }

    let wheels = resolve_wheels(wheels, ctx.mounts)?;

    let suspension = SuspensionParams::new(
        suspension.rest_length as f64,
        suspension.wheel_radius as f64,
        suspension.stiffness as f64,
        suspension.damping as f64,
        suspension.max_travel as f64,
        suspension.ray_margin as f64,
    );
    let tire = TireParams::new(
        tire.cornering_stiffness_front as f64,
        tire.cornering_stiffness_rear as f64,
        tire.rolling_resistance as f64,
        tire.low_speed_threshold as f64,
    );

    let plant = RaycastWheelPlant::new(wheels, suspension, tire);

    // Plant ↔ actuation kind agreement, the same guard the L0 builder runs: the
    // raycast plant applies only Torque (drive) and Position (steer), so any other
    // kind in the contract would be silently dropped every tick. Reject it here at
    // spawn, naming the offenders, rather than warn-and-ignore forever.
    let unaccepted: Vec<String> = ctx
        .actuation
        .actuators()
        .iter()
        .filter(|spec| !plant.accepts(spec.kind()))
        .map(|spec| format!("{:?} ({:?})", spec.id(), spec.kind()))
        .collect();

    if !unaccepted.is_empty() {
        return Err(format!(
            "{} plant cannot apply the actuation contract's declared kind(s): {}",
            ctx.config.kind_str(),
            unaccepted.join(", "),
        ));
    }

    ctx.commands.entity(ctx.entity).insert((
        RaycastWheelPlantComponent(plant),
        ActuationModelComponent(ctx.actuation.clone()),
    ));

    Ok(())
}

/// Resolve each configured wheel to a core [`Wheel`], looking its mount up by name
/// in the topology's mount list. Offsets come from the mount *config*, not from the
/// spawned mount-frame entities: body assembly runs the topology, plant, and
/// collision builders over one command buffer, so those entities are still queued —
/// unspawned — when this runs. An unknown mount name is a config error surfaced as
/// `Err`, which fails the spawn.
fn resolve_wheels(wheels: &[WheelConfig], mounts: &[MountConfig]) -> Result<Vec<Wheel>, String> {
    wheels
        .iter()
        .map(|w| {
            let mount = mounts
                .iter()
                .find(|m| m.name == w.mount)
                .ok_or_else(|| format!("wheel references unknown mount `{}`", w.mount))?;
            let t = mount.pose.translation; // body-FLU
            let offset = Point::<Flu>::new(t.x, t.y, t.z);
            let axle = match w.axle {
                AxleConfig::Front => Axle::Front,
                AxleConfig::Rear => Axle::Rear,
            };
            Ok(Wheel::new(offset, axle, w.drive, w.steer))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::control::actuation_model::ActuationModel;

    use bevy::ecs::system::SystemState;
    use figment::{
        providers::{Format, Toml},
        Figment,
    };
    use nalgebra::{UnitQuaternion, Vector3};

    use crate::config::structs::{Pose, SuspensionConfig, TireConfig};

    // The suspension and tire scalars are irrelevant to the wiring these tests
    // assert — the physics they feed is tested in helios_core — so one well-formed
    // set stands in everywhere.
    fn suspension() -> SuspensionConfig {
        SuspensionConfig {
            rest_length: 0.40,
            wheel_radius: 0.30,
            stiffness: 50_000.0,
            damping: 4_000.0,
            max_travel: 0.20,
            ray_margin: 0.10,
        }
    }

    fn tire(low_speed_threshold: f32) -> TireConfig {
        TireConfig {
            cornering_stiffness_front: 80_000.0,
            cornering_stiffness_rear: 60_000.0,
            rolling_resistance: 0.015,
            low_speed_threshold,
        }
    }

    fn wheel(mount: &str, axle: AxleConfig, steer: bool, drive: bool) -> WheelConfig {
        WheelConfig {
            mount: mount.to_string(),
            axle,
            steer,
            drive,
        }
    }

    // Only the name is matched against; the pose is arbitrary but well-formed.
    fn mount(name: &str) -> MountConfig {
        MountConfig {
            name: name.to_string(),
            pose: Pose {
                translation: Vector3::new(1.3, 0.75, 0.0),
                rotation: UnitQuaternion::identity(),
            },
        }
    }

    fn raycast_plant(low_speed_threshold: f32, wheels: Vec<WheelConfig>) -> PlantConfig {
        PlantConfig::RaycastWheels {
            suspension: suspension(),
            tire: tire(low_speed_threshold),
            wheels,
        }
    }

    // `ActuatorSpec`'s fields are private, so a TOML parse is the only way to build
    // a contract from outside helios_core (the approach the L0 plant tests use too).
    fn actuation(toml: &str) -> ActuationModel {
        Figment::new()
            .merge(Toml::string(toml))
            .extract()
            .expect("actuation TOML should deserialize")
    }

    // A Torque drive and a Position steer — exactly the two kinds the raycast plant
    // applies.
    fn torque_steer_contract() -> ActuationModel {
        actuation(
            "[[actuators]]\n\
             id = \"drive\"\n\
             kind = \"Torque\"\n\
             limit = 400.0\n\
             safe_state = { Torque = 0.0 }\n\
             sign = \"Normal\"\n\
             [[actuators]]\n\
             id = \"steer\"\n\
             kind = \"Position\"\n\
             limit = 0.61\n\
             safe_state = { Position = 0.0 }\n\
             sign = \"Normal\"\n",
        )
    }

    // Runs the builder against one config / contract / mount set, returning its
    // result and whether the plant component landed on the entity.
    fn build_with(
        config: &PlantConfig,
        actuation: &ActuationModel,
        mounts: &[MountConfig],
    ) -> (Result<(), String>, bool) {
        let mut world = World::new();
        let entity = world.spawn_empty().id();

        let mut state: SystemState<Commands> = SystemState::new(&mut world);
        let result = {
            let mut commands = state.get_mut(&mut world).unwrap();
            let mut ctx = PlantBuildContext {
                entity,
                commands: &mut commands,
                config,
                actuation,
                mounts,
            };
            build_raycast_wheels(&mut ctx)
        };
        state.apply(&mut world);

        let built = world
            .entity(entity)
            .get::<RaycastWheelPlantComponent>()
            .is_some();
        (result, built)
    }

    // Every wheel row whose mount name is present resolves to one core `Wheel`. The
    // per-wheel geometry is helios_core's concern; here we assert only that the
    // name→mount join produces the right count.
    #[test]
    fn resolve_wheels_maps_every_row() {
        let wheels = vec![
            wheel("wheel_fl", AxleConfig::Front, true, false),
            wheel("wheel_rl", AxleConfig::Rear, false, true),
        ];
        let mounts = vec![mount("wheel_fl"), mount("wheel_rl")];

        let resolved = resolve_wheels(&wheels, &mounts).expect("every mount is present");
        assert_eq!(resolved.len(), 2);
    }

    // A wheel naming a mount the topology never declared is a config error, and the
    // message names the missing mount so the author can find it. (`Wheel` is not
    // `Debug`, so the Ok arm is destructured by hand rather than via `expect_err`.)
    #[test]
    fn resolve_wheels_rejects_unknown_mount() {
        let wheels = vec![wheel("wheel_fl", AxleConfig::Front, true, false)];
        let mounts = vec![mount("wheel_rl")];

        let Err(error) = resolve_wheels(&wheels, &mounts) else {
            panic!("a wheel with no matching mount must be rejected");
        };
        assert!(
            error.contains("wheel_fl"),
            "error should name the missing mount, got: {error}"
        );
    }

    // A valid threshold, wheels whose mounts all resolve, and a contract of kinds
    // the plant accepts: the build succeeds and deposits the plant component.
    #[test]
    fn builds_and_deposits_for_accepted_contract() {
        let config = raycast_plant(
            1.0,
            vec![
                wheel("wheel_fl", AxleConfig::Front, true, false),
                wheel("wheel_rl", AxleConfig::Rear, false, true),
            ],
        );
        let mounts = vec![mount("wheel_fl"), mount("wheel_rl")];

        let (result, built) = build_with(&config, &torque_steer_contract(), &mounts);

        assert!(result.is_ok(), "build should succeed, got: {result:?}");
        assert!(built, "the plant component should be deposited");
    }

    // `low_speed_threshold` divides the low-speed regularizer, so a non-positive
    // value is rejected at spawn rather than dividing by zero at runtime.
    #[test]
    fn rejects_nonpositive_low_speed_threshold() {
        let config = raycast_plant(0.0, vec![wheel("wheel_fl", AxleConfig::Front, true, true)]);
        let mounts = vec![mount("wheel_fl")];

        let (result, built) = build_with(&config, &torque_steer_contract(), &mounts);

        assert!(result.is_err(), "a zero threshold must be rejected");
        assert!(!built, "a rejected build must not deposit the plant");
    }

    // The raycast plant applies only Torque and Position; a Velocity-commanded drive
    // is a kind it cannot apply, so the build fails at spawn — naming the offender —
    // instead of silently dropping the setpoint every tick.
    #[test]
    fn rejects_unacceptable_actuator_kind() {
        let config = raycast_plant(1.0, vec![wheel("wheel_fl", AxleConfig::Front, true, true)]);
        let mounts = vec![mount("wheel_fl")];
        let velocity_drive = actuation(
            "[[actuators]]\n\
             id = \"drive\"\n\
             kind = \"Velocity\"\n\
             limit = 40.0\n\
             safe_state = { Velocity = 0.0 }\n\
             sign = \"Normal\"\n",
        );

        let (result, built) = build_with(&config, &velocity_drive, &mounts);

        let error = result.expect_err("a velocity-commanded drive must be rejected");
        assert!(
            error.contains("drive"),
            "error should name the offending actuator, got: {error}"
        );
        assert!(!built, "a rejected build must not deposit the plant");
    }
}
