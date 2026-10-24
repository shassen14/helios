//! `[evaluate.params]` of a `nees` entry: the frame policy relating the
//! state's root to the truth's.

use crate::evaluation::kinds::pose_error::config::Align;

use serde::Deserialize;

/// _Pre-step._ A `nees` entry's parameters.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NeesConfig {
    #[serde(default)]
    pub align: Align,
}

impl NeesConfig {
    /// Why these parameters can't grade consistency, or `None` when they
    /// can: `declared` alone. Aligning the roots on the first sample sets
    /// that sample's error to zero whatever the covariance says, so the
    /// error isn't the one the covariance is judged against.
    pub fn refusal(&self) -> Option<String> {
        match self.align {
            Align::Declared => None,
            Align::FirstPose => Some(format!(
                "align = \"{}\" can't grade consistency: aligning the roots on the first \
                 sample's estimate zeroes the error its covariance is judged against",
                self.align
            )),
            Align::BestFit | Align::Relative => Some(format!(
                "align = \"{}\" is not built for nees; align = \"{}\" is",
                self.align,
                Align::Declared
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(text: &str) -> Result<NeesConfig, toml::de::Error> {
        toml::from_str(text)
    }

    #[test]
    fn no_params_is_declared_and_grades() {
        let params = read("").expect("no params");

        assert_eq!(params.align, Align::Declared);
        assert_eq!(params.refusal(), None);
    }

    #[test]
    fn first_pose_is_refused_saying_why() {
        let refusal = read("align = \"first_pose\"")
            .expect("a policy")
            .refusal()
            .expect("refused");

        assert!(
            refusal.contains("zeroes the error its covariance is judged against"),
            "{refusal}"
        );
    }

    #[test]
    fn an_unbuilt_policy_or_unknown_key_is_refused() {
        assert!(read("align = \"best_fit\"")
            .expect("a policy")
            .refusal()
            .is_some());
        assert!(read("gain = 2.0").is_err());
    }
}
