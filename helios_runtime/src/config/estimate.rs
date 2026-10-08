//! [`EstimateSeamConfig`] — the stack's `[estimate]` section.

use serde::Deserialize;

/// Which estimator's state is the agent's estimate: the one control, planning
/// and mapping read, and the one whose pose becomes the `odom → base_link`
/// edge.
///
/// An estimator writes its state on a channel named after itself and knows
/// nothing of this seam; which one is authoritative is a fact about the stack,
/// so it is stated here. Any other estimator in the stack runs in shadow: its
/// state is on the bus to compare, but nothing steers by it.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct EstimateSeamConfig {
    /// The estimator node forwarded as the estimate, by its node name.
    pub source: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_source_parses_and_an_unknown_key_is_refused() {
        let config: EstimateSeamConfig =
            toml::from_str(r#"source = "primary""#).expect("a lone source parses");
        assert_eq!(config.source, "primary");

        assert!(toml::from_str::<EstimateSeamConfig>(r#"sorce = "primary""#).is_err());
    }
}
