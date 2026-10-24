//! What a body is touching: each other side of a contact, named by what it
//! is rather than by its geometry, so a reader can tell a wall from another
//! car without a world model of its own.

use super::object_instance::ObjectInstanceId;
use super::semantic_class::SemanticClass;
use crate::prelude::{AgentId, MonotonicTime};
use crate::spatial::FrameId;

/// Every contact `body` had at some point in the step ending at `stamp`,
/// one per other side. An empty list means it touched nothing.
///
/// The list is sorted by the other side, so two producers that saw the same
/// touches in a different order build equal values.
#[derive(Debug, Clone, PartialEq)]
pub struct Contacts {
    /// The frame of the body that touched.
    pub body: FrameId,
    /// The end of the step the touches happened in.
    pub stamp: MonotonicTime,
    touching: Vec<Contact>,
}

impl Contacts {
    /// `touching` sorted by the other side, the first contact with each side
    /// kept.
    pub fn new(
        body: FrameId,
        stamp: MonotonicTime,
        touching: impl IntoIterator<Item = Contact>,
    ) -> Self {
        let mut touching: Vec<Contact> = touching.into_iter().collect();
        touching.sort_by(|a, b| a.other.cmp(&b.other));
        touching.dedup_by(|a, b| a.other == b.other);
        Self {
            body,
            stamp,
            touching,
        }
    }

    /// Each contact, one per other side, in the other side's order.
    pub fn touching(&self) -> &[Contact] {
        &self.touching
    }
}

/// One contact of the body: the other side, and room for what the contact
/// itself was (how hard, which way) as readers come to need it.
#[derive(Debug, Clone, PartialEq)]
pub struct Contact {
    pub other: Touched,
}

/// The other side of one contact.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Touched {
    /// Another agent's body.
    Agent(AgentId),
    /// A world object: its instance in ground truth and its class.
    Object {
        instance: ObjectInstanceId,
        class: SemanticClass,
    },
    /// Something with no identity, as a bumper reports it. Every unknown
    /// side touched in one step is one contact.
    Unknown,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contacts_are_sorted_and_kept_once_per_side_whatever_order_they_arrive_in() {
        let body = FrameId::base_link(AgentId::new("car"));
        let wall = Touched::Object {
            instance: ObjectInstanceId::new("yard", "wall_a"),
            class: SemanticClass::UNLABELED,
        };
        let other_car = Touched::Agent(AgentId::new("truck"));
        let contact = |other: &Touched| Contact {
            other: other.clone(),
        };

        let forward = Contacts::new(
            body.clone(),
            MonotonicTime(1.0),
            [
                contact(&wall),
                contact(&other_car),
                contact(&Touched::Unknown),
            ],
        );
        let backward = Contacts::new(
            body,
            MonotonicTime(1.0),
            [
                contact(&Touched::Unknown),
                contact(&other_car),
                contact(&wall),
                contact(&other_car),
            ],
        );

        assert_eq!(forward, backward);
        assert_eq!(
            forward.touching(),
            &[
                contact(&other_car),
                contact(&wall),
                contact(&Touched::Unknown)
            ]
        );
    }
}
