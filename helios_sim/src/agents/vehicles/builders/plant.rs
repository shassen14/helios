use crate::agents::vehicles::components::ActuationModelComponent;
use crate::config::structs::PlantConfig;
use crate::registry::contexts::PlantBuildContext;

use helios_core::plant::L0ShimPlant;

use avian3d::prelude::*;
use bevy::prelude::*;

/// The L0 arcade plant model on an agent, the marker its apply system queries.
/// Wraps the portable [`L0ShimPlant`] fold (setpoints → chassis wrench) so the
/// host holds a Bevy component while the map itself stays in `helios_core`.
/// Retired when a dynamic plant replaces the shim.
#[derive(Component)]
pub struct L0ShimPlantComponent(pub L0ShimPlant);

/// Builds the L0 arcade plant: the gain marker its apply system reads, the passive
/// damping that stands in for real resistive forces (an L0 crutch, gone at L1), and
/// the resolved actuation contract the apply system enforces each tick.
#[allow(irrefutable_let_patterns)] // single variant today; a second makes the else-arm live
pub fn build_l0_shim(ctx: &mut PlantBuildContext) -> Result<(), String> {
    let PlantConfig::L0Shim {
        l0_force_gain,
        l0_yaw_gain,
        linear_damping,
        angular_damping,
    } = ctx.config
    else {
        return Err(format!(
            "L0Shim builder received a `{}` plant",
            ctx.config.kind_str(),
        ));
    };

    let plant = L0ShimPlant::new(*l0_force_gain as f64, *l0_yaw_gain as f64);

    // Plant ↔ actuation kind agreement (a cross-axis guard the per-axis registries
    // cannot express). `resolve` makes the command the apply system hands this plant
    // total over the actuation contract's *declared* kinds, so the plant must accept
    // every one of them — a kind it can't apply (a Torque-commanded drive on the
    // velocity-folding shim) would otherwise be silently warned-and-ignored every
    // tick. Caught here at spawn instead, as a startup failure naming the offenders.
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
        L0ShimPlantComponent(plant),
        LinearDamping(*linear_damping),
        AngularDamping(*angular_damping),
        ActuationModelComponent(ctx.actuation.clone()),
    ));

    Ok(())
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

    // A stand-in L0 plant config; the gains and damping are irrelevant to the
    // kind guard, which reads only the actuation contract.
    fn l0_config() -> PlantConfig {
        PlantConfig::L0Shim {
            l0_force_gain: 1.0,
            l0_yaw_gain: 1.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
        }
    }

    // The vehicle's actuation contract, deserialized the same way config load
    // builds it — `ActuatorSpec`'s fields are private, so a real parse is the
    // only way to construct one from outside `helios_core`.
    fn actuation(toml: &str) -> ActuationModel {
        Figment::new()
            .merge(Toml::string(toml))
            .extract()
            .expect("actuation TOML should deserialize")
    }

    // Runs the builder against one actuation contract, returning both its result
    // and whether the plant component landed on the entity.
    fn build_with(actuation: &ActuationModel) -> (Result<(), String>, bool) {
        let config = l0_config();
        let mut world = World::new();
        let entity = world.spawn_empty().id();

        let mut state: SystemState<Commands> = SystemState::new(&mut world);
        let result = {
            let mut commands = state.get_mut(&mut world).unwrap();
            let mut ctx = PlantBuildContext {
                entity,
                commands: &mut commands,
                config: &config,
                actuation,
                mounts: &[],
            };
            build_l0_shim(&mut ctx)
        };
        state.apply(&mut world);

        let built = world.entity(entity).get::<L0ShimPlantComponent>().is_some();
        (result, built)
    }

    #[test]
    fn actuation_the_shim_can_apply_builds() {
        // The car's contract: a velocity-commanded drive and a position-commanded
        // steer — exactly the two kinds the L0 fold applies — so the build succeeds
        // and deposits the plant.
        let (result, built) = build_with(&actuation(
            "[[actuators]]\n\
             id = \"drive\"\n\
             kind = \"Velocity\"\n\
             limit = 40.0\n\
             safe_state = { Velocity = 0.0 }\n\
             sign = \"Normal\"\n\
             [[actuators]]\n\
             id = \"steer\"\n\
             kind = \"Position\"\n\
             limit = 0.61\n\
             safe_state = { Position = 0.0 }\n\
             sign = \"Normal\"\n",
        ));

        assert!(result.is_ok(), "build should succeed, got: {result:?}");
        assert!(built, "the plant component should be deposited");
    }

    #[test]
    fn actuation_kind_the_shim_cannot_apply_is_rejected() {
        // A torque-commanded drive: the L0 shim folds velocity, not torque, so it
        // cannot apply this actuator. The build fails at spawn — naming the
        // offender — rather than warning-and-ignoring the setpoint every tick.
        let (result, built) = build_with(&actuation(
            "[[actuators]]\n\
             id = \"drive\"\n\
             kind = \"Torque\"\n\
             limit = 400.0\n\
             safe_state = { Torque = 0.0 }\n\
             sign = \"Normal\"\n",
        ));

        let error = result.expect_err("a torque-commanded drive must be rejected");
        assert!(
            error.contains("drive"),
            "error should name the offending actuator, got: {error}"
        );
        assert!(!built, "a rejected build must not deposit the plant");
    }
}
