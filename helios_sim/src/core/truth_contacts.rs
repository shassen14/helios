//! `truth.contacts`: what each agent's body touched during the step, read
//! from Avian and named by identity, so a reader tells a wall from another
//! car without an ECS handle.

use crate::brain_bridge::{AgentIdComponent, AutonomyPipelineComponent};
use crate::core::components::{GroundTruthState, ObjectClass, ObjectInstance};
use crate::core::ground_truth::{stamped_now, warn_dropped_truth};

use helios_core::interchange::perception::contact::{Contact, Contacts, Touched};
use helios_core::prelude::AgentId;
use helios_core::spatial::FrameId;
use helios_runtime::channels::truth_contacts;
use helios_runtime::port::ChannelKey;

use avian3d::prelude::{ColliderOf, CollidingEntities, CollisionStart};
use bevy::prelude::*;
use std::collections::{HashMap, HashSet};

/// Publishes each agent's `truth.contacts`: every body its own touched at
/// any point in the step, stamped [`GroundTruthState::at`]. Runs in
/// [`SimulationSet::StateSync`](crate::core::app_state::SimulationSet::StateSync)
/// after [`ground_truth_sync_system`](crate::core::ground_truth_sync_system).
///
/// A step's touches are those that began in it ([`CollisionStart`], so one
/// shorter than a step is kept) and those still held at its end
/// ([`CollidingEntities`]). Each side is resolved to its rigid body, the
/// entity that carries its identity. Written every tick, empty or not.
///
/// A side with no agent or object identity is published as
/// [`Touched::Unknown`], never dropped, and warned about once. A refused
/// write is a host bug, as for the other truth channels, and warns once.
#[allow(clippy::type_complexity)]
pub fn publish_truth_contacts_system(
    mut started: MessageReader<CollisionStart>,
    agents: Query<(
        Entity,
        &GroundTruthState,
        &AgentIdComponent,
        &AutonomyPipelineComponent,
        &CollidingEntities,
    )>,
    bodies: Query<&ColliderOf>,
    identities: Query<(
        Option<&AgentIdComponent>,
        Option<&ObjectInstance>,
        Option<&ObjectClass>,
        Option<&Name>,
    )>,
    mut warned_unknown: Local<HashSet<Entity>>,
    mut warned_refused: Local<HashSet<(AgentId, ChannelKey)>>,
) {
    let body_of = |collider: Entity| bodies.get(collider).map_or(collider, |of| of.body);

    let mut began: HashMap<Entity, HashSet<Entity>> = HashMap::new();
    for event in started.read() {
        let first = event.body1.unwrap_or(event.collider1);
        let second = event.body2.unwrap_or(event.collider2);
        began.entry(first).or_default().insert(second);
        began.entry(second).or_default().insert(first);
    }

    for (entity, ground_truth, agent, autonomy_pipeline, colliding) in &agents {
        let held = colliding.0.iter().map(|&collider| body_of(collider));
        let others: HashSet<Entity> = began
            .get(&entity)
            .into_iter()
            .flatten()
            .copied()
            .chain(held)
            .collect();

        let touching = others.into_iter().map(|other| {
            let (other_agent, instance, class, name) = identities.get(other).unwrap_or_default();
            let touched = identify(other_agent, instance, class);
            if touched == Touched::Unknown && warned_unknown.insert(other) {
                warn!(
                    "Agent '{}' touched {other:?} ({}), which has no agent or object identity; \
                     its contacts are published as unknown.",
                    agent.0,
                    name.map_or("unnamed", Name::as_str),
                );
            }
            Contact { other: touched }
        });
        let contacts = Contacts::new(
            FrameId::base_link(agent.0.clone()),
            ground_truth.at,
            touching,
        );

        let key = truth_contacts();
        let result = autonomy_pipeline
            .0
            .bus()
            .write_keyed(key.clone(), stamped_now(contacts, ground_truth.at));
        warn_dropped_truth(&agent.0, key, result, &mut warned_refused);
    }
}

/// The other side of a contact, from the identity its body carries: an
/// agent, a world object (it needs both its instance and its class), or
/// unknown.
fn identify(
    agent: Option<&AgentIdComponent>,
    instance: Option<&ObjectInstance>,
    class: Option<&ObjectClass>,
) -> Touched {
    match (agent, instance, class) {
        (Some(agent), _, _) => Touched::Agent(agent.0.clone()),
        (None, Some(instance), Some(class)) => Touched::Object {
            instance: instance.0.clone(),
            class: class.0,
        },
        _ => Touched::Unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::interchange::perception::object_instance::ObjectInstanceId;
    use helios_core::interchange::perception::semantic_class::SemanticClass;

    #[test]
    fn an_agent_is_named_by_its_id_and_an_object_by_instance_and_class() {
        let agent = AgentIdComponent(AgentId::new("truck"));
        let instance = ObjectInstance(ObjectInstanceId::new("yard", "wall_a"));
        let class = ObjectClass(SemanticClass::UNLABELED);

        assert_eq!(
            identify(Some(&agent), None, None),
            Touched::Agent(AgentId::new("truck"))
        );
        assert_eq!(
            identify(None, Some(&instance), Some(&class)),
            Touched::Object {
                instance: ObjectInstanceId::new("yard", "wall_a"),
                class: SemanticClass::UNLABELED,
            }
        );
    }

    #[test]
    fn a_side_without_a_full_identity_is_unknown() {
        let instance = ObjectInstance(ObjectInstanceId::new("yard", "wall_a"));

        assert_eq!(identify(None, None, None), Touched::Unknown);
        assert_eq!(identify(None, Some(&instance), None), Touched::Unknown);
    }
}
