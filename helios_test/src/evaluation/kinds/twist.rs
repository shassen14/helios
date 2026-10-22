//! `twist`: how fast one body moves, at each of its velocities. Read with
//! `final`, whether it came to rest; with `max`, its top speed.

use crate::evaluation::{
    kind::{BoundInput, InputRejection},
    typed::{projections::body_twist, BaseFacts, Component, NoParams, OneInput},
};

use helios_core::control::commands::BodyTwist;
use helios_core::interchange::motion::StampedTwist;
use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::FrameId;
use helios_core::spatial::conventions::Flu;

/// _Per-step._ The `twist` kind, bound to one entry: reads the velocity of
/// `body`, expressed in it, each value checked for both.
pub struct Twist {
    body: FrameId,
}

impl OneInput for Twist {
    const NAME: &'static str = "twist";
    /// The body's velocity, its instance the moving body.
    const BASE: &'static str = "twist";
    type Params = NoParams;
    type BasePayload = StampedTwist<Flu>;
    type BaseValue = BodyTwist;
    type Fit = ();
    type State = ();
    type Output = BodyTwist;

    fn keep_base(&self, twist: &StampedTwist<Flu>) -> Result<BodyTwist, InputRejection> {
        body_twist(twist, &self.body)
    }

    fn menu() -> Option<Vec<Component<BodyTwist>>> {
        Some(components())
    }

    /// The input must select the body whose velocity it reads.
    fn bind(_params: NoParams, twist: &BoundInput) -> Result<Self, Vec<String>> {
        match twist.frame(Self::BASE) {
            Ok(Some(body)) => Ok(Self { body }),
            Ok(None) => Err(vec![format!(
                "the {} has no instance: {} reads one body's velocity, so select the body as \
                 its instance",
                Self::BASE,
                Self::NAME
            )]),
            Err(problem) => Err(vec![problem]),
        }
    }

    fn components(&self) -> Vec<Component<BodyTwist>> {
        components()
    }

    fn fit(&self, _run: &[&BodyTwist]) -> Result<(), InputRejection> {
        Ok(())
    }

    fn compute(
        &self,
        twist: &BodyTwist,
        _facts: BaseFacts,
        _state: &mut (),
        _fit: &(),
    ) -> Result<BodyTwist, InputRejection> {
        Ok(*twist)
    }
}

/// The body's speed: the size of its linear velocity, every axis included,
/// so a car at rest on its suspension reads its bounce.
fn components() -> Vec<Component<BodyTwist>> {
    vec![Component::each(
        "speed",
        SiUnit::MetersPerSecond,
        |twist: &BodyTwist| Ok(twist.linear().raw().norm()),
    )]
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::{kind::EvaluatorKind, typed::OneInputKind, Gap, GapWidth};

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::quantities::FreeVector;

    fn input(instance: Option<String>) -> BoundInput {
        BoundInput {
            agent: AgentId::new("car"),
            instance,
            expressed_in: None,
            convention: None,
            rate_hz: None,
            writer: None,
        }
    }

    fn body() -> FrameId {
        FrameId::base_link(AgentId::new("car"))
    }

    #[test]
    fn no_instance_is_refused() {
        let problems = Twist::bind(NoParams {}, &input(None)).err();

        assert!(problems.is_some_and(|problems| problems[0].contains("has no instance")));
    }

    #[test]
    fn speed_is_the_size_of_the_linear_velocity_and_another_frames_is_refused() {
        let twist = |moving: FrameId| StampedTwist::<Flu> {
            moving,
            relative_to: FrameId::global("datum"),
            expressed_in: body(),
            stamp: MonotonicTime(0.0),
            linear: FreeVector::new(3.0, 0.0, 4.0),
            angular: FreeVector::new(0.0, 0.0, 1.0),
        };
        let mut evaluation = OneInputKind::<Twist>::new()
            .build(
                &toml::Table::new(),
                &[input(Some(body().to_string()))],
                &[Some(Gap::Seconds(
                    GapWidth::try_from(1.0).expect("positive"),
                ))],
            )
            .unwrap_or_else(|problems| panic!("{problems:?}"));
        evaluation.receive(0, MonotonicTime(0.0), &twist(body()));
        evaluation.receive(
            0,
            MonotonicTime(0.1),
            &twist(FrameId::agent(AgentId::new("car"), "wheel")),
        );

        let derived = evaluation.derive(MonotonicTime(1.0), &[0]);

        let speed = derived.metrics[0].as_ref().expect("wanted").samples();
        assert_eq!(speed.len(), 1);
        assert_eq!(format!("{:?}", speed[0].value), "Float(5.0)");
        assert_eq!(evaluation.refused()[0].count, 1);
    }
}
