//! `twist_tracking`: how closely a body drives the velocity its guidance asks
//! for. At each true velocity, that velocity minus the guidance reference in
//! force, on each of the six body axes, and as a speed. Every axis is offered
//! whatever the body; a run file reads the ones its body is commanded on.

use crate::evaluation::{
    kind::{BoundInput, InputRejection},
    typed::{
        projections::{body_twist, reference_twist},
        Component, Lining, NoParams, TwoInputs,
    },
};

use helios_core::control::{commands::BodyTwist, BodyTwistRef};
use helios_core::interchange::motion::StampedTwist;
use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::FrameId;
use helios_core::spatial::conventions::Flu;

/// _Per-step._ The `twist_tracking` kind, bound to one entry: compares
/// velocities of `body`, expressed in it. Both inputs are expressed in the
/// body's `base_link`: the reference by its type's contract, the truth by
/// its value's `expressed_in`, which each value is checked for. So they
/// compare with no anchor between trees, and the kind has no frame policy.
pub struct TwistTracking {
    body: FrameId,
}

impl TwoInputs for TwistTracking {
    const NAME: &'static str = "twist_tracking";
    /// The body's true velocity, from a truth source.
    const BASE: &'static str = "truth";
    /// The guidance reference the body's controllers track.
    const OTHER: &'static str = "reference";
    type Params = NoParams;
    type BasePayload = StampedTwist<Flu>;
    type BaseValue = BodyTwist;
    type OtherPayload = BodyTwistRef;
    type OtherValue = BodyTwist;
    type Fit = ();
    type Output = TwistError;

    fn keep_base(&self, truth: &StampedTwist<Flu>) -> Result<BodyTwist, InputRejection> {
        body_twist(truth, &self.body)
    }

    fn keep_other(&self, reference: &BodyTwistRef) -> Result<BodyTwist, InputRejection> {
        Ok(reference_twist(reference))
    }

    /// A reference holds until the next, and expires past its cap: guidance
    /// that stopped is no longer a reference.
    fn other_lining(&self) -> Lining<BodyTwist> {
        Lining::HoldWithinCap
    }

    fn menu() -> Option<Vec<Component<TwistError>>> {
        Some(components())
    }

    fn bind(
        _params: NoParams,
        truth: &BoundInput,
        reference: &BoundInput,
    ) -> Result<Self, Vec<String>> {
        if truth.agent != reference.agent {
            return Err(vec![format!(
                "the {} is agent '{}''s and the {} agent '{}''s: a reference is compared with \
                 the velocity of the body it was given to",
                Self::BASE,
                truth.agent,
                Self::OTHER,
                reference.agent
            )]);
        }
        let body = FrameId::base_link(reference.agent.clone());
        let moving = truth.frame(Self::BASE).map_err(|problem| vec![problem])?;
        if moving.as_ref() != Some(&body) {
            return Err(vec![format!(
                "the {} is the velocity of '{}', and the {} is expressed in '{body}': select \
                 instance = \"{body}\"",
                Self::BASE,
                moving.map_or_else(|| "no instance".to_owned(), |frame| frame.to_string()),
                Self::OTHER
            )]);
        }
        Ok(Self { body })
    }

    fn components(&self) -> Vec<Component<TwistError>> {
        components()
    }

    fn fit(&self, _run: &[(&BodyTwist, &BodyTwist)]) -> Result<(), InputRejection> {
        Ok(())
    }

    fn compute(
        &self,
        truth: &BodyTwist,
        reference: &BodyTwist,
        _fit: &(),
    ) -> Result<TwistError, InputRejection> {
        Ok(TwistError {
            truth: *truth,
            reference: *reference,
        })
    }
}

/// _Per-step._ One sample of `twist_tracking`: the body's true velocity and
/// the reference in force, both on the body's axes.
#[derive(Debug, Clone, Copy)]
pub struct TwistError {
    truth: BodyTwist,
    reference: BodyTwist,
}

impl TwistError {
    /// The truth minus the reference, on each axis.
    fn difference(&self) -> BodyTwist {
        self.truth - self.reference
    }

    /// How much faster the body moves than it was asked to: positive when
    /// faster. Zero for one that moves as fast in another direction.
    fn speed(&self) -> f64 {
        self.truth.linear().raw().norm() - self.reference.linear().raw().norm()
    }

    /// How far the body's velocity is from the one asked for, direction
    /// included.
    fn velocity(&self) -> f64 {
        self.difference().linear().raw().norm()
    }
}

/// The truth minus the reference, in FLU order: the linear axes (forward,
/// left, up) in m/s, then the rotation rates about them in rad/s; then the
/// difference of the two speeds, and the size of the difference of the two
/// linear velocities.
fn components() -> Vec<Component<TwistError>> {
    vec![
        Component::each("surge", SiUnit::MetersPerSecond, |e: &TwistError| {
            Ok(e.difference().linear().x())
        }),
        Component::each("sway", SiUnit::MetersPerSecond, |e: &TwistError| {
            Ok(e.difference().linear().y())
        }),
        Component::each("heave", SiUnit::MetersPerSecond, |e: &TwistError| {
            Ok(e.difference().linear().z())
        }),
        Component::each("roll_rate", SiUnit::RadiansPerSecond, |e: &TwistError| {
            Ok(e.difference().angular().x())
        }),
        Component::each("pitch_rate", SiUnit::RadiansPerSecond, |e: &TwistError| {
            Ok(e.difference().angular().y())
        }),
        Component::each("yaw_rate", SiUnit::RadiansPerSecond, |e: &TwistError| {
            Ok(e.difference().angular().z())
        }),
        Component::each("speed", SiUnit::MetersPerSecond, |e: &TwistError| {
            Ok(e.speed())
        }),
        Component::each("velocity", SiUnit::MetersPerSecond, |e: &TwistError| {
            Ok(e.velocity())
        }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::{kind::EvaluatorKind, typed::TwoInputsKind, Gap, GapWidth};

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::quantities::FreeVector;

    const CAR: &str = "car";
    const TOL: f64 = 1e-12;

    /// Each role's gap, the reference's a cap it takes whatever its rate.
    fn gaps() -> [Option<Gap>; 2] {
        let seconds = GapWidth::try_from(0.1).expect("positive and finite");
        [Some(Gap::Seconds(seconds)); 2]
    }

    fn input(agent: &str, instance: Option<String>) -> BoundInput {
        BoundInput {
            agent: AgentId::new(agent),
            instance,
            expressed_in: None,
            convention: None,
            rate_hz: None,
            writer: None,
        }
    }

    fn base_link(agent: &str) -> FrameId {
        FrameId::base_link(AgentId::new(agent))
    }

    fn kind() -> TwoInputsKind<TwistTracking> {
        TwoInputsKind::new()
    }

    fn evaluator() -> TwistTracking {
        let truth = input(CAR, Some(base_link(CAR).to_string()));
        TwistTracking::bind(NoParams {}, &truth, &input(CAR, None))
            .unwrap_or_else(|problems| panic!("{problems:?}"))
    }

    fn truth(surge: f64, yaw_rate: f64) -> BodyTwist {
        BodyTwist::new(
            FreeVector::new(surge, 0.1, -0.5),
            FreeVector::new(0.02, -0.25, yaw_rate),
        )
    }

    fn reference(surge: f64, yaw_rate: f64) -> BodyTwist {
        BodyTwist::unicycle(surge, yaw_rate)
    }

    /// Every component's value at one sample, in menu order.
    fn read(error: &TwistError) -> Vec<f64> {
        let errors = [(MonotonicTime(0.0), *error)];
        components()
            .iter()
            .flat_map(|component| component.values(&errors, &mut Default::default()))
            .map(|(_, value)| value)
            .collect()
    }

    #[test]
    fn each_axis_is_the_truth_minus_the_reference() {
        // A unicycle reference commands surge and yaw rate only; the truth
        // moves on every axis, so each other axis reads the truth.
        let evaluator = evaluator();

        let error = evaluator
            .compute(&truth(4.0, 0.25), &reference(5.0, 0.5), &())
            .expect("computed");

        let values = read(&error);
        assert_eq!(&values[..6], &[-1.0, 0.1, -0.5, 0.02, -0.25, -0.25]);
    }

    #[test]
    fn speed_compares_the_two_speeds_and_velocity_the_two_vectors() {
        // Asked for 5 m/s forward, the body moves 5 m/s to its left: as fast
        // as asked, in a direction 5√2 m/s off.
        let error = TwistError {
            truth: BodyTwist::new(FreeVector::new(0.0, 5.0, 0.0), FreeVector::zeros()),
            reference: reference(5.0, 0.0),
        };

        let values = read(&error);

        assert!(values[6].abs() < TOL, "{values:?}");
        assert!((values[7] - 50.0_f64.sqrt()).abs() < TOL, "{values:?}");
    }

    #[test]
    fn the_menu_follows_the_flu_axes_then_speed_with_their_units() {
        let menu: Vec<(&str, SiUnit)> = kind()
            .menu()
            .expect("a fixed menu")
            .iter()
            .map(|metric| (metric.name, metric.si_unit))
            .collect();

        assert_eq!(
            menu,
            vec![
                ("surge", SiUnit::MetersPerSecond),
                ("sway", SiUnit::MetersPerSecond),
                ("heave", SiUnit::MetersPerSecond),
                ("roll_rate", SiUnit::RadiansPerSecond),
                ("pitch_rate", SiUnit::RadiansPerSecond),
                ("yaw_rate", SiUnit::RadiansPerSecond),
                ("speed", SiUnit::MetersPerSecond),
                ("velocity", SiUnit::MetersPerSecond),
            ]
        );
    }

    #[test]
    fn a_truth_velocity_of_another_frame_is_refused_on_arrival() {
        let truth = input(CAR, Some(base_link(CAR).to_string()));
        let mut evaluation = kind()
            .build(&toml::Table::new(), &[truth, input(CAR, None)], &gaps())
            .unwrap_or_else(|problems| panic!("{problems:?}"));
        let in_datum = StampedTwist::<Flu> {
            moving: base_link(CAR),
            relative_to: FrameId::global("datum"),
            expressed_in: FrameId::global("datum"),
            stamp: MonotonicTime(0.0),
            linear: FreeVector::new(4.0, 0.0, 0.0),
            angular: FreeVector::zeros(),
        };

        evaluation.receive(0, MonotonicTime(0.0), &in_datum);

        assert_eq!(evaluation.refused()[0].count, 1);
    }

    #[test]
    fn inputs_from_two_agents_are_refused() {
        let truth = input(CAR, Some(base_link(CAR).to_string()));

        let refused = TwistTracking::bind(NoParams {}, &truth, &input("truck", None));

        assert!(refused.is_err());
    }

    #[test]
    fn a_truth_instance_other_than_the_body_names_the_one_to_select() {
        let wheel = input(CAR, Some("agent.car.wheel".to_owned()));

        let problems = TwistTracking::bind(NoParams {}, &wheel, &input(CAR, None))
            .err()
            .expect("another frame's velocity");

        assert!(
            problems[0].contains("select instance = \"agent.car.base_link\""),
            "{problems:?}"
        );
    }

    #[test]
    fn a_truth_instance_that_isnt_a_frame_is_refused_naming_it() {
        let ros_frame = input(CAR, Some("base_link".to_owned()));

        let problems = TwistTracking::bind(NoParams {}, &ros_frame, &input(CAR, None))
            .err()
            .expect("not a frame");

        assert!(
            problems[0].contains("the truth's instance 'base_link' is not a frame"),
            "{problems:?}"
        );
    }

    #[test]
    fn a_parameter_is_refused() {
        let mut params = toml::Table::new();
        params.insert("gain".to_owned(), toml::Value::Float(1.0));

        assert!(kind().check_params(&params).is_err());
        assert!(kind().check_params(&toml::Table::new()).is_ok());
    }
}
