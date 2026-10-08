//! [`ReferenceSeamConfig`] — the stack's `[reference]` section.

use serde::Deserialize;

/// Which nodes feed the guidance reference the controllers track, and how one
/// is chosen.
///
/// Members are named by their `[nodes]` table key. A node writes its reference
/// on a channel named after itself and knows nothing of this seam; whether it
/// contends, and with what priority, is a fact about the stack, so it is stated
/// here. The list is explicit even for a lone source, so the resolved config
/// shows the whole topology.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ReferenceSeamConfig {
    /// The member forwarded whenever no `preferred` member wins. Its reference
    /// is never checked for freshness, so a lone `base` is forwarded every tick.
    pub base: String,

    /// Members that override `base` while their reference is fresh, highest
    /// priority first.
    #[serde(default)]
    pub preferred: Vec<String>,

    /// How a `preferred` member wins over `base`.
    #[serde(default)]
    pub policy: ArbitrationPolicyConfig,

    /// How recent a `preferred` member's reference must be (seconds) to win.
    /// Older than this and `base` is forwarded.
    #[serde(default = "default_max_age_s")]
    pub max_age_s: f64,
}

/// Default freshness window (seconds) for a `preferred` member. A starting
/// value; tune per platform in the `[reference]` section.
const DEFAULT_MAX_AGE_S: f64 = 0.5;

fn default_max_age_s() -> f64 {
    DEFAULT_MAX_AGE_S
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum ArbitrationPolicyConfig {
    #[default]
    FreshnessOverride,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_lone_base_takes_every_default() {
        let config: ReferenceSeamConfig =
            toml::from_str(r#"base = "pure_pursuit""#).expect("a lone base parses");

        assert_eq!(config.base, "pure_pursuit");
        assert!(config.preferred.is_empty());
        assert!(matches!(
            config.policy,
            ArbitrationPolicyConfig::FreshnessOverride
        ));
        assert!((config.max_age_s - DEFAULT_MAX_AGE_S).abs() < f64::EPSILON);
    }

    #[test]
    fn base_is_required() {
        let result: Result<ReferenceSeamConfig, _> = toml::from_str(r#"preferred = ["teleop"]"#);

        assert!(result.is_err(), "a seam without a base must not parse");
    }

    #[test]
    fn a_retired_key_is_rejected() {
        let result: Result<ReferenceSeamConfig, _> =
            toml::from_str("base = \"pure_pursuit\"\nsources = [\"autonomy\"]");

        assert!(result.is_err(), "`sources` is not a seam key");
    }
}
