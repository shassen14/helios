//! The head and tail every plant's per-tick apply system shares.
//!
//! Each vehicle family runs its own `Actuation` system — the L0 shim folds a
//! command straight to a wrench, the raycast car casts wheel rays first — but the
//! ends are identical: resolve the pipeline's command against the vehicle's
//! actuator contract on the way in, and rotate the resulting body-frame wrench
//! into world space for Avian on the way out. Those ends live here as free
//! functions so a new morphology writes only the middle that is actually its own.

use crate::core::components::ActuatorCommandComponent;
use crate::core::transforms::FluVector;

use helios_core::control::actuation_model::ActuationModel;
use helios_core::control::actuators::ActuatorCommand;
use helios_core::plant::PlantWrench;

use avian3d::prelude::{ConstantForce, ConstantTorque};
use bevy::prelude::*;

/// Enforce the vehicle's actuator contract on the brain's latest command — the
/// head of every plant's apply system.
///
/// `None` means the pipeline has published no command yet, so the caller skips the
/// agent this tick. Otherwise [`ActuationModel::resolve`] clamps each setpoint to
/// its limit, applies its sign convention, and substitutes the fail-safe for any
/// missing, wrong-kind, or non-finite value, so the command handed to the plant is
/// always safe to apply.
pub(crate) fn resolve_command(
    model: &ActuationModel,
    command: Option<&ActuatorCommandComponent>,
) -> Option<ActuatorCommand> {
    Some(model.resolve(&command?.0))
}

/// Rotate a plant's body-frame wrench into world space and apply it — the tail of
/// every plant's apply system.
///
/// Every plant funnels its [`PlantWrench`] through here, so the body-FLU → Bevy
/// world rotation and the `ConstantForce` / `ConstantTorque` insert live in one
/// place regardless of which rung produced the wrench. `body_rotation` is the
/// body's world orientation; the wrench is expressed about the centre of mass in
/// the body FLU frame. Any actuator the plant reported it could not apply is warned
/// once, in the command's actuator order.
pub(crate) fn apply_body_wrench(
    commands: &mut Commands,
    entity: Entity,
    body_rotation: Quat,
    wrench: &PlantWrench,
) {
    for actuator in wrench.unsupported() {
        warn!(
            ?actuator,
            "plant received a setpoint kind it cannot apply; ignoring"
        );
    }

    let w = wrench.wrench();
    let force_world = body_rotation * Vec3::from(FluVector(w.force().into_inner()));
    let torque_world = body_rotation * Vec3::from(FluVector(w.torque().into_inner()));

    commands.entity(entity).insert((
        ConstantForce::new(force_world.x, force_world.y, force_world.z),
        ConstantTorque::new(torque_world.x, torque_world.y, torque_world.z),
    ));
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::control::actuators::{ActuatorId, ActuatorSetpoint, SetpointValue};
    use helios_core::control::commands::BodyWrench;
    use helios_core::frames::quantities::FluVector as CoreFluVector;

    use bevy::ecs::system::SystemState;
    use figment::{
        providers::{Format, Toml},
        Figment,
    };

    // A one-actuator contract: a drive torque saturating at 400 N·m, coasting to
    // zero on fail-safe. Parsed from TOML because `ActuatorSpec`'s fields are
    // private — the same route the plant-builder tests take.
    fn torque_contract() -> ActuationModel {
        Figment::new()
            .merge(Toml::string(
                "[[actuators]]\n\
                 id = \"drive\"\n\
                 kind = \"Torque\"\n\
                 limit = 400.0\n\
                 safe_state = { Torque = 0.0 }\n\
                 sign = \"Normal\"\n",
            ))
            .extract()
            .expect("actuation TOML should deserialize")
    }

    // The scalar of the resolved command's `drive` setpoint.
    fn drive_scalar(command: &ActuatorCommand) -> f64 {
        command
            .setpoints()
            .iter()
            .find(|sp| sp.actuator() == &ActuatorId::new("drive"))
            .expect("the resolved command carries the contract's drive actuator")
            .value()
            .scalar()
    }

    // No command published yet: the head yields `None` so the caller skips the
    // agent, rather than applying a stale or zeroed wrench.
    #[test]
    fn resolve_command_returns_none_without_a_command() {
        assert!(resolve_command(&torque_contract(), None).is_none());
    }

    // A present command is resolved against the contract, not passed through: a
    // 1000 N·m setpoint on a 400 N·m actuator comes back clamped to the limit.
    #[test]
    fn resolve_command_enforces_the_contract() {
        let command = ActuatorCommandComponent(ActuatorCommand::new(vec![ActuatorSetpoint::new(
            ActuatorId::new("drive"),
            SetpointValue::Torque(1000.0),
        )]));

        let resolved = resolve_command(&torque_contract(), Some(&command))
            .expect("a present command resolves");

        assert_eq!(drive_scalar(&resolved), 400.0);
    }

    // A pure normal load: 5000 N straight up the body FLU +Z, no torque and no
    // unsupported actuators — a known wrench to check the tail's rotation against.
    fn normal_load_wrench() -> PlantWrench {
        PlantWrench::new(
            BodyWrench::new(CoreFluVector::new(0.0, 0.0, 5000.0), CoreFluVector::zeros()),
            Vec::new(),
        )
    }

    // The tail rotates the body-FLU wrench into Bevy world and inserts it. Under an
    // identity body rotation FLU up (+Z) is Bevy up (+Y), so a 5000 N normal load
    // lands as a +Y `ConstantForce`; acting at the centre of mass, it makes no
    // torque. This locks the shared FLU→world crossing every plant depends on.
    #[test]
    fn apply_body_wrench_rotates_flu_up_to_bevy_up() {
        let mut world = World::new();
        let entity = world.spawn_empty().id();

        let mut state: SystemState<Commands> = SystemState::new(&mut world);
        {
            let mut commands = state.get_mut(&mut world).unwrap();
            apply_body_wrench(&mut commands, entity, Quat::IDENTITY, &normal_load_wrench());
        }
        state.apply(&mut world);

        let force = world
            .entity(entity)
            .get::<ConstantForce>()
            .expect("the tail inserts a ConstantForce");
        assert!(
            force.0.x.abs() < 1e-3,
            "no lateral force, got {}",
            force.0.x
        );
        assert!(
            (force.0.y - 5000.0).abs() < 1e-3,
            "FLU +Z maps to Bevy +Y, got {}",
            force.0.y
        );
        assert!(
            force.0.z.abs() < 1e-3,
            "no fore-aft force, got {}",
            force.0.z
        );

        let torque = world
            .entity(entity)
            .get::<ConstantTorque>()
            .expect("the tail inserts a ConstantTorque");
        assert!(
            torque.0.length() < 1e-3,
            "a force at the CoM makes no torque, got {torque:?}"
        );
    }
}
