use serde::Deserialize;

/// How the command terminal is fed: which sources contend for it and how the
/// winner is chosen.
///
/// Named for its seam — it arbitrates *commands*. Other seams (e.g. estimator
/// failover) arbitrate different payloads over different channel kinds and get
/// their own sibling config rather than widening this one.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct CommandArbitrationConfig {
    /// Command sources in priority order, highest authority first. The count
    /// picks the topology: empty infers `[Autonomy]` when a controller exists
    /// (else a build error); one source writes `command` directly with no
    /// arbiter; two or more insert a `Selector`.
    #[serde(default)]
    pub sources: Vec<CommandSource>,

    /// How the arbiter chooses among two or more sources. Ignored for the
    /// zero- and one-source topologies, which have nothing to arbitrate.
    #[serde(default)]
    pub policy: ArbitrationPolicyConfig,

    /// How recent a teleop command must be (seconds) to override the autonomy
    /// command. Older than this and the selector falls back to autonomy.
    #[serde(default = "default_teleop_max_age_s")]
    pub teleop_max_age_s: f64,
}

impl Default for CommandArbitrationConfig {
    fn default() -> Self {
        Self {
            sources: Vec::default(),
            policy: ArbitrationPolicyConfig::default(),
            teleop_max_age_s: DEFAULT_TELEOP_MAX_AGE_S,
        }
    }
}

/// Default teleop freshness window (seconds). A starting value — tune per
/// platform via the `[command_arbitration]` config section.
const DEFAULT_TELEOP_MAX_AGE_S: f64 = 0.5;

fn default_teleop_max_age_s() -> f64 {
    DEFAULT_TELEOP_MAX_AGE_S
}

#[derive(Debug, Deserialize, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CommandSource {
    Autonomy, // → control::autonomy::<T>()
    Teleop,   // → control::teleop::<T>()
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum ArbitrationPolicyConfig {
    #[default]
    FreshnessOverride,
}
