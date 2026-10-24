//! What a body is touching: each other side of a contact, named by what it
//! is rather than by its geometry, so a reader can tell a wall from another
//! car without a world model of its own.

use super::semantic_class::SemanticClass;
use crate::prelude::{AgentId, MonotonicTime};
use crate::spatial::FrameId;

/// Everything `body` touched at some point in the step ending at `stamp`,
/// each other side named once. An empty list means it touched nothing.
///
/// The list is sorted, so two producers that saw the same touches in a
/// different order build equal values.
#[derive(Debug, Clone, PartialEq)]
pub struct Contacts {
    /// The frame of the body that touched.
    pub body: FrameId,
    /// The end of the step the touches happened in.
    pub stamp: MonotonicTime,
    touching: Vec<Touched>,
}

impl Contacts {
    /// `touching` sorted, each other side kept once.
    pub fn new(
        body: FrameId,
        stamp: MonotonicTime,
        touching: impl IntoIterator<Item = Touched>,
    ) -> Self {
        let mut touching: Vec<Touched> = touching.into_iter().collect();
        touching.sort();
        touching.dedup();
        Self {
            body,
            stamp,
            touching,
        }
    }

    /// Each other side touched, once, in sorted order.
    pub fn touching(&self) -> &[Touched] {
        &self.touching
    }
}

/// The other side of one contact.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Touched {
    /// Another agent's body.
    Agent(AgentId),
    /// A world object: its instance in ground truth and its class.
    Object {
        instance: String,
        class: SemanticClass,
    },
    /// Something with no identity, as a bumper reports it. Every unknown
    /// side touched in one step is one entry.
    Unknown,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn touches_are_sorted_and_kept_once_whatever_order_they_arrive_in() {
        let body = FrameId::base_link(AgentId::new("car"));
        let wall = Touched::Object {
            instance: "yard/wall_a".to_string(),
            class: SemanticClass::UNLABELED,
        };
        let other_car = Touched::Agent(AgentId::new("truck"));

        let forward = Contacts::new(
            body.clone(),
            MonotonicTime(1.0),
            [wall.clone(), other_car.clone(), Touched::Unknown],
        );
        let backward = Contacts::new(
            body,
            MonotonicTime(1.0),
            [
                Touched::Unknown,
                other_car.clone(),
                wall.clone(),
                other_car.clone(),
            ],
        );

        assert_eq!(forward, backward);
        assert_eq!(forward.touching(), &[other_car, wall, Touched::Unknown]);
    }
}
