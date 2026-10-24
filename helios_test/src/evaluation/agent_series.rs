//! Agent series: what a suite's entry names for a role, meaning the same
//! series on every agent the suite is applied to. Each is a channel the
//! runtime's contract defines once (a seam's output or the body's truth),
//! read at the agent's `base_link` when the channel is keyed, so binding a
//! suite to an agent spells no channel the runtime doesn't own.

use helios_core::prelude::{AgentId, FrameId};
use helios_runtime::channels::{self, control, estimate, tf};
use helios_runtime::port::ChannelKey;

use serde::Deserialize;
use std::{
    any::{type_name, TypeId},
    fmt,
};

const ESTIMATE_POSE: &str = "estimate.pose";
const ESTIMATE_STATE: &str = "estimate.state";
const TRUTH_POSE: &str = "truth.pose";
const TRUTH_TWIST: &str = "truth.twist";
const REFERENCE: &str = "reference";

/// _Pre-step._ A series every agent with the matching seam or truth source
/// has, named in a suite by its spelling.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Deserialize)]
#[serde(try_from = "String")]
pub enum AgentSeries {
    /// `base_link`'s estimated pose in `odom`: the `tf` edge the estimate
    /// seam's relay writes.
    EstimatePose,
    /// The authoritative estimator's state, forwarded by the estimate seam.
    EstimateState,
    /// `base_link`'s true pose, from the body's truth source.
    TruthPose,
    /// `base_link`'s true velocity, expressed in it.
    TruthTwist,
    /// The guidance reference the controllers track, of the morphology's
    /// type.
    Reference,
}

impl AgentSeries {
    /// Every series, in the order a message lists them.
    const ALL: [AgentSeries; 5] = [
        AgentSeries::EstimatePose,
        AgentSeries::EstimateState,
        AgentSeries::TruthPose,
        AgentSeries::TruthTwist,
        AgentSeries::Reference,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            AgentSeries::EstimatePose => ESTIMATE_POSE,
            AgentSeries::EstimateState => ESTIMATE_STATE,
            AgentSeries::TruthPose => TRUTH_POSE,
            AgentSeries::TruthTwist => TRUTH_TWIST,
            AgentSeries::Reference => REFERENCE,
        }
    }

    /// The instance it reads of `agent`'s channel: the agent's `base_link`
    /// for a keyed channel, `None` for one holding a single value.
    pub fn instance(self, agent: &AgentId) -> Option<FrameId> {
        match self {
            AgentSeries::EstimatePose | AgentSeries::TruthPose | AgentSeries::TruthTwist => {
                Some(FrameId::base_link(agent.clone()))
            }
            AgentSeries::EstimateState | AgentSeries::Reference => None,
        }
    }

    /// What gives an agent this series, as a refusal names its absence:
    /// "agent `car` has no estimate seam".
    pub fn supplier(self) -> &'static str {
        match self {
            AgentSeries::EstimatePose | AgentSeries::EstimateState => "estimate seam",
            AgentSeries::TruthPose | AgentSeries::TruthTwist => "truth source",
            AgentSeries::Reference => "reference seam",
        }
    }

    /// Its channel, for `role` reading values of `T`: the reference carrying
    /// `T`, or a fixed-type series' own channel when it carries `T`. One
    /// carrying another type is refused, naming both.
    pub fn key<T: 'static>(self, role: &str) -> Result<ChannelKey, String> {
        let key = match self {
            AgentSeries::EstimatePose => tf::tf(),
            AgentSeries::EstimateState => estimate::estimate(),
            AgentSeries::TruthPose => channels::truth_tf(),
            AgentSeries::TruthTwist => channels::truth_twist(),
            AgentSeries::Reference => return Ok(control::reference::<T>()),
        };
        if key.type_id() == TypeId::of::<T>() {
            Ok(key)
        } else {
            Err(format!(
                "`{self}` carries {}, and the {role} reads {}",
                key.type_name(),
                type_name::<T>()
            ))
        }
    }
}

impl TryFrom<String> for AgentSeries {
    type Error = String;

    fn try_from(spelling: String) -> Result<Self, Self::Error> {
        AgentSeries::ALL
            .into_iter()
            .find(|series| series.as_str() == spelling)
            .ok_or_else(|| {
                let known: Vec<&str> = AgentSeries::ALL.iter().map(|s| s.as_str()).collect();
                format!(
                    "'{spelling}' is not an agent series: use one of {}",
                    known.join(", ")
                )
            })
    }
}

impl fmt::Display for AgentSeries {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::standard_evaluators;

    use helios_core::control::BodyTwistRef;
    use helios_core::interchange::motion::StampedTwist;
    use helios_core::spatial::conventions::Flu;
    use helios_core::spatial::transforms::tf::stamped::StampedTransform;
    use helios_core::spatial::FrameAwareState;

    const ROLE: &str = "estimate";

    fn car() -> AgentId {
        AgentId::new("car")
    }

    #[test]
    fn every_spelling_reads_back_as_its_series() {
        for series in AgentSeries::ALL {
            assert_eq!(AgentSeries::try_from(series.to_string()), Ok(series));
        }
    }

    #[test]
    fn an_unknown_spelling_is_refused_listing_the_known_ones() {
        let refusal = AgentSeries::try_from("estimate.twist".to_owned()).unwrap_err();

        assert!(refusal.contains("'estimate.twist'"), "{refusal}");
        for series in AgentSeries::ALL {
            assert!(refusal.contains(series.as_str()), "{refusal}");
        }
    }

    #[test]
    fn each_series_reads_its_runtime_channel() {
        assert_eq!(
            AgentSeries::EstimatePose.key::<StampedTransform>(ROLE),
            Ok(tf::tf())
        );
        assert_eq!(
            AgentSeries::EstimateState.key::<FrameAwareState>(ROLE),
            Ok(estimate::estimate())
        );
        assert_eq!(
            AgentSeries::TruthPose.key::<StampedTransform>(ROLE),
            Ok(channels::truth_tf())
        );
        assert_eq!(
            AgentSeries::TruthTwist.key::<StampedTwist<Flu>>(ROLE),
            Ok(channels::truth_twist())
        );
        assert_eq!(
            AgentSeries::Reference.key::<BodyTwistRef>(ROLE),
            Ok(control::reference::<BodyTwistRef>())
        );
    }

    #[test]
    fn a_keyed_series_reads_the_agents_base_link_and_the_others_no_instance() {
        let base_link = Some(FrameId::base_link(car()));

        assert_eq!(AgentSeries::EstimatePose.instance(&car()), base_link);
        assert_eq!(AgentSeries::TruthPose.instance(&car()), base_link);
        assert_eq!(AgentSeries::TruthTwist.instance(&car()), base_link);
        assert_eq!(AgentSeries::EstimateState.instance(&car()), None);
        assert_eq!(AgentSeries::Reference.instance(&car()), None);
    }

    #[test]
    fn a_fixed_type_series_read_as_another_type_is_refused_naming_both() {
        let refusal = AgentSeries::TruthPose
            .key::<StampedTwist<Flu>>("truth")
            .unwrap_err();

        assert!(refusal.contains("`truth.pose`"), "{refusal}");
        assert!(refusal.contains("the truth reads"), "{refusal}");
        assert!(
            refusal.contains(type_name::<StampedTransform>()),
            "{refusal}"
        );
        assert!(
            refusal.contains(type_name::<StampedTwist<Flu>>()),
            "{refusal}"
        );
    }

    #[test]
    fn the_reference_carries_whatever_the_role_reads() {
        let twist = AgentSeries::Reference.key::<BodyTwistRef>(ROLE).unwrap();
        let other = AgentSeries::Reference.key::<f64>(ROLE).unwrap();

        assert_eq!(twist.type_id(), TypeId::of::<BodyTwistRef>());
        assert_eq!(other.type_id(), TypeId::of::<f64>());
        assert_eq!(twist.name(), other.name());
    }

    #[test]
    fn a_kind_answers_for_each_role_from_its_payload_type() {
        let registry = standard_evaluators();
        let pose_error = registry.get("pose_error").unwrap();
        let twist_tracking = registry.get("twist_tracking").unwrap();
        let twist = registry.get("twist").unwrap();

        assert_eq!(
            pose_error.series_key(1, AgentSeries::TruthPose),
            Ok(channels::truth_tf())
        );
        assert_eq!(
            twist_tracking.series_key(1, AgentSeries::Reference),
            Ok(control::reference::<BodyTwistRef>())
        );
        assert_eq!(
            twist.series_key(0, AgentSeries::TruthTwist),
            Ok(channels::truth_twist())
        );
        assert!(twist_tracking
            .series_key(0, AgentSeries::TruthPose)
            .is_err());
    }

    #[test]
    fn a_kind_refuses_a_role_it_doesnt_have() {
        let registry = standard_evaluators();

        let refusal = registry
            .get("twist")
            .unwrap()
            .series_key(1, AgentSeries::TruthTwist)
            .unwrap_err();

        assert!(refusal.contains("twist has no role 1"), "{refusal}");
    }
}
