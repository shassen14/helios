//! `[evaluate.params]` of a `pose_error` entry: the frame policy relating the
//! estimate's root to the truth's.

use serde::Deserialize;
use std::fmt;

/// _Pre-step._ A `pose_error` entry's parameters.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PoseErrorConfig {
    #[serde(default)]
    pub align: Align,
}

/// _Pre-step._ How the estimate's root is placed in the truth's, when the
/// two trees don't share one.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Align {
    /// The two poses must be in one root. Otherwise every sample is
    /// refused.
    #[default]
    Declared,
    /// Place the estimate's root in the truth's by the two poses at the
    /// first sample, so a constant offset between the roots is not graded.
    /// The method for an estimator with no anchor that starts where its body
    /// is.
    FirstPose,
    /// Place it by a least-squares fit over the run.
    BestFit,
    /// Compare motion over a window, needing no common root.
    Relative,
}

impl Align {
    /// The first case that will need this policy built, or `None` for one
    /// that is built.
    pub fn unbuilt_until(&self) -> Option<&'static str> {
        match self {
            Align::Declared | Align::FirstPose => None,
            Align::BestFit => Some("the first hardware or SLAM run"),
            Align::Relative => Some("drifting odometry graded between loop closures"),
        }
    }
}

impl fmt::Display for Align {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Align::Declared => "declared",
            Align::FirstPose => "first_pose",
            Align::BestFit => "best_fit",
            Align::Relative => "relative",
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(text: &str) -> Result<PoseErrorConfig, toml::de::Error> {
        toml::from_str(text)
    }

    #[test]
    fn no_params_is_declared() {
        assert_eq!(read("").ok(), Some(PoseErrorConfig::default()));
        assert_eq!(PoseErrorConfig::default().align, Align::Declared);
    }

    #[test]
    fn every_policy_parses_and_declared_and_first_pose_are_built() {
        for (spelled, align) in [
            ("declared", Align::Declared),
            ("first_pose", Align::FirstPose),
            ("best_fit", Align::BestFit),
            ("relative", Align::Relative),
        ] {
            assert_eq!(align.to_string(), spelled);
            assert_eq!(
                read(&format!("align = \"{spelled}\"")).ok(),
                Some(PoseErrorConfig { align })
            );
            assert_eq!(
                align.unbuilt_until().is_none(),
                matches!(align, Align::Declared | Align::FirstPose)
            );
        }
    }

    #[test]
    fn an_unknown_key_or_policy_is_refused() {
        assert!(read("gain = 2.0").is_err());
        assert!(read("align = \"guess\"").is_err());
    }
}
