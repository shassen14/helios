use crate::config::structs::CollisionConfig;
use crate::registry::contexts::CollisionBuildContext;

use avian3d::prelude::*;
use bevy::prelude::*;

/// Builds the collision axis: a cuboid collider plus its contact friction. The
/// collider is the solver's geometry, deliberately distinct from the visual mesh
/// and from any future perception geometry.
#[allow(irrefutable_let_patterns)] // single variant today; a second makes the else-arm live
pub fn build_cuboid(ctx: &mut CollisionBuildContext) -> Result<(), String> {
    let CollisionConfig::Cuboid { x, y, z, friction } = ctx.config else {
        return Err(format!(
            "Cuboid builder received a `{}` collision",
            ctx.config.kind_str(),
        ));
    };

    ctx.commands
        .entity(ctx.entity)
        .insert((Collider::cuboid(*x, *y, *z), Friction::new(*friction)));

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::ecs::system::SystemState;

    // The build contract is an ECS effect (deposit a collider + friction on the
    // entity), so this is a Tier-2 minimal-World test, not a pure-fn one.
    #[test]
    fn cuboid_builder_deposits_collider_and_friction() {
        let mut world = World::new();
        let entity = world.spawn_empty().id();
        let config = CollisionConfig::Cuboid {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            friction: 0.6,
        };

        let mut state: SystemState<Commands> = SystemState::new(&mut world);
        {
            let mut commands = state.get_mut(&mut world).unwrap();
            let mut ctx = CollisionBuildContext {
                entity,
                commands: &mut commands,
                config: &config,
            };
            build_cuboid(&mut ctx).unwrap();
        }
        state.apply(&mut world);

        assert!(world.entity(entity).get::<Collider>().is_some());
        assert!(world.entity(entity).get::<Friction>().is_some());
    }
}
