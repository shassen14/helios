//! `twist_tracking`: how closely a body drives the velocity its guidance asks
//! for. At each true velocity, the guidance reference in force minus that
//! velocity, on each of the six body axes. Every axis is derived whatever
//! the body; a run file asserts on the ones its body is commanded on.

use crate::evaluation::kind::{
    Alignment, BoundInput, Evaluator, EvaluatorKind, InputRejection, Metric, Projection, Role,
};

use helios_core::control::{commands::BodyTwist, BodyTwistRef};
use helios_core::interchange::motion::StampedTwist;
use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::FrameId;
use helios_core::spatial::conventions::Flu;

use std::any::Any;

/// The name a run file's `kind` gives.
const KIND: &str = "twist_tracking";

/// The body's true velocity, from a truth source: the base.
const TRUTH: &str = "truth";

/// The guidance reference the body's controllers track, held until the
/// next one.
const REFERENCE: &str = "reference";

/// Each role's index in [`TwistTracking::roles`].
const TRUTH_ROLE: usize = 0;
const REFERENCE_ROLE: usize = 1;

/// Each axis of the body twist, reference minus truth, in FLU order: the
/// linear axes (forward, left, up), then the rotation rates about them.
const METRICS: [Metric; 6] = [
    Metric {
        name: "surge_error",
        si_unit: SiUnit::MetersPerSecond,
    },
    Metric {
        name: "sway_error",
        si_unit: SiUnit::MetersPerSecond,
    },
    Metric {
        name: "heave_error",
        si_unit: SiUnit::MetersPerSecond,
    },
    Metric {
        name: "roll_rate_error",
        si_unit: SiUnit::RadiansPerSecond,
    },
    Metric {
        name: "pitch_rate_error",
        si_unit: SiUnit::RadiansPerSecond,
    },
    Metric {
        name: "yaw_rate_error",
        si_unit: SiUnit::RadiansPerSecond,
    },
];

/// _Pre-step._ The `twist_tracking` kind. Both inputs are expressed in the
/// body's `base_link`: the reference by its type's contract, the truth by
/// its value's `expressed_in`, which each value is checked for. So the
/// `declared` frame policy holds with no anchor between trees.
pub struct TwistTracking;

impl EvaluatorKind for TwistTracking {
    fn name(&self) -> &'static str {
        KIND
    }

    fn roles(&self) -> Vec<Role> {
        vec![
            Role::of::<StampedTwist<Flu>>(TRUTH, Alignment::Base),
            Role::of::<BodyTwistRef>(REFERENCE, Alignment::Hold),
        ]
    }

    fn metrics(&self) -> &'static [Metric] {
        &METRICS
    }

    fn check_params(&self, params: &toml::Table) -> Result<(), String> {
        match params.keys().next() {
            None => Ok(()),
            Some(key) => Err(format!("{KIND} reads no parameters, and was given '{key}'")),
        }
    }

    fn build(
        &self,
        _params: &toml::Table,
        inputs: &[BoundInput],
    ) -> Result<Box<dyn Evaluator>, Vec<String>> {
        let (Some(truth), Some(reference)) = (inputs.get(TRUTH_ROLE), inputs.get(REFERENCE_ROLE))
        else {
            return Err(vec![format!("{KIND} needs both of its inputs bound")]);
        };
        if truth.agent != reference.agent {
            return Err(vec![format!(
                "the {TRUTH} is agent '{}''s and the {REFERENCE} agent '{}''s: a reference is \
                 compared with the velocity of the body it was given to",
                truth.agent, reference.agent
            )]);
        }
        let body = FrameId::base_link(reference.agent.clone());
        let body_spelled = body.to_string();
        if truth.instance.as_deref() != Some(body_spelled.as_str()) {
            return Err(vec![format!(
                "the {TRUTH} is the velocity of '{}', and the {REFERENCE} is expressed in \
                 '{body_spelled}': select instance = \"{body_spelled}\"",
                truth.instance.as_deref().unwrap_or("no instance")
            )]);
        }
        Ok(Box::new(TwistTrackingEvaluator { body }))
    }
}

/// _Per-step._ One `twist_tracking` entry: compares velocities of `body`,
/// expressed in it.
struct TwistTrackingEvaluator {
    body: FrameId,
}

impl Evaluator for TwistTrackingEvaluator {
    fn project(&self, role: usize, payload: &dyn Any) -> Result<Projection, InputRejection> {
        let twist = match role {
            TRUTH_ROLE => {
                let truth = payload
                    .downcast_ref::<StampedTwist<Flu>>()
                    .ok_or(InputRejection(
                        "a truth value that isn't a StampedTwist<Flu>",
                    ))?;
                if truth.moving != self.body {
                    return Err(InputRejection(
                        "a truth velocity of another frame than the body's base_link",
                    ));
                }
                if truth.expressed_in != self.body {
                    return Err(InputRejection(
                        "a truth velocity expressed in another frame than the body's base_link",
                    ));
                }
                BodyTwist::new(truth.linear, truth.angular)
            }
            REFERENCE_ROLE => *payload
                .downcast_ref::<BodyTwistRef>()
                .ok_or(InputRejection("a reference that isn't a BodyTwistRef"))?
                .twist(),
            _ => {
                return Err(InputRejection(
                    "a value for a role twist_tracking doesn't have",
                ))
            }
        };
        Ok(Box::new(twist))
    }

    fn compute(&self, aligned: &[&Projection]) -> Vec<f64> {
        let twist = |role: usize| {
            aligned
                .get(role)
                .and_then(|projection| projection.downcast_ref::<BodyTwist>())
        };
        match (twist(TRUTH_ROLE), twist(REFERENCE_ROLE)) {
            (Some(truth), Some(reference)) => {
                let error = *reference - *truth;
                let (linear, angular) = (error.linear(), error.angular());
                vec![
                    linear.x(),
                    linear.y(),
                    linear.z(),
                    angular.x(),
                    angular.y(),
                    angular.z(),
                ]
            }
            // Projections are this evaluator's own, so this is never reached;
            // a NaN fails any bound rather than passing one.
            _ => vec![f64::NAN; METRICS.len()],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::{AgentId, MonotonicTime};
    use helios_core::spatial::quantities::FreeVector;

    const CAR: &str = "car";

    fn input(agent: &str, instance: Option<String>) -> BoundInput {
        BoundInput {
            agent: AgentId::new(agent),
            instance,
        }
    }

    fn base_link(agent: &str) -> FrameId {
        FrameId::base_link(AgentId::new(agent))
    }

    fn evaluator() -> Box<dyn Evaluator> {
        let truth = input(CAR, Some(base_link(CAR).to_string()));
        TwistTracking
            .build(&toml::Table::new(), &[truth, input(CAR, None)])
            .unwrap_or_else(|problems| panic!("{problems:?}"))
    }

    fn truth(surge: f64, yaw_rate: f64, expressed_in: FrameId) -> StampedTwist<Flu> {
        StampedTwist {
            moving: base_link(CAR),
            relative_to: FrameId::global("datum"),
            expressed_in,
            stamp: MonotonicTime(0.0),
            linear: FreeVector::new(surge, 0.1, -0.5),
            angular: FreeVector::new(0.02, -0.25, yaw_rate),
        }
    }

    fn reference(surge: f64, yaw_rate: f64) -> BodyTwistRef {
        BodyTwistRef::new(BodyTwist::unicycle(surge, yaw_rate))
    }

    #[test]
    fn each_axis_is_the_reference_minus_the_truth() {
        // A unicycle reference commands surge and yaw rate only; the truth
        // moves on every axis, so each other axis reads minus the truth.
        let evaluator = evaluator();
        let truth = evaluator
            .project(TRUTH_ROLE, &truth(4.0, 0.25, base_link(CAR)))
            .expect("a truth velocity in base_link");
        let reference = evaluator
            .project(REFERENCE_ROLE, &reference(5.0, 0.5))
            .expect("a reference");

        assert_eq!(
            evaluator.compute(&[&truth, &reference]),
            vec![1.0, -0.1, 0.5, -0.02, 0.25, 0.25]
        );
    }

    #[test]
    fn the_metrics_follow_the_flu_axes_with_their_units() {
        let declared: Vec<(&str, SiUnit)> = TwistTracking
            .metrics()
            .iter()
            .map(|metric| (metric.name, metric.si_unit))
            .collect();

        assert_eq!(
            declared,
            vec![
                ("surge_error", SiUnit::MetersPerSecond),
                ("sway_error", SiUnit::MetersPerSecond),
                ("heave_error", SiUnit::MetersPerSecond),
                ("roll_rate_error", SiUnit::RadiansPerSecond),
                ("pitch_rate_error", SiUnit::RadiansPerSecond),
                ("yaw_rate_error", SiUnit::RadiansPerSecond),
            ]
        );
    }

    #[test]
    fn a_truth_velocity_expressed_in_another_frame_is_refused() {
        let evaluator = evaluator();

        let refused = evaluator.project(TRUTH_ROLE, &truth(4.0, 0.0, FrameId::global("datum")));

        assert!(refused.is_err());
    }

    #[test]
    fn inputs_from_two_agents_are_refused() {
        let truth = input(CAR, Some(base_link(CAR).to_string()));

        let refused = TwistTracking.build(&toml::Table::new(), &[truth, input("truck", None)]);

        assert!(refused.is_err());
    }

    #[test]
    fn a_truth_instance_other_than_the_body_names_the_one_to_select() {
        let wheel = input(CAR, Some("agent.car.wheel".to_owned()));

        let problems = TwistTracking
            .build(&toml::Table::new(), &[wheel, input(CAR, None)])
            .err()
            .expect("another frame's velocity");

        assert!(
            problems[0].contains("select instance = \"agent.car.base_link\""),
            "{problems:?}"
        );
    }

    #[test]
    fn a_parameter_is_refused() {
        let mut params = toml::Table::new();
        params.insert("gain".to_owned(), toml::Value::Float(1.0));

        assert!(TwistTracking.check_params(&params).is_err());
        assert!(TwistTracking.check_params(&toml::Table::new()).is_ok());
    }
}
