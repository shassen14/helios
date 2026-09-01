use serde::Deserialize;

/// How the guidance reference seam is fed: which sources contend for it and how
/// the winner is chosen.
///
/// Named for its seam — it arbitrates *references*. Other seams (e.g. estimator
/// failover) arbitrate different payloads over different channel kinds and get
/// their own sibling config rather than widening this one.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ReferenceArbitrationConfig {
    /// Reference sources in priority order, highest authority first. The count
    /// and kind pick the topology: empty infers `[Autonomy]` when a controller
    /// exists (else a build error); a lone internal source (autonomy) writes
    /// the `reference` seam directly with no arbiter; a lone host source
    /// (teleop) or two or more sources insert a `Selector`.
    #[serde(default)]
    pub sources: Vec<ReferenceSource>,

    /// How the arbiter chooses among two or more sources. Ignored for the
    /// zero- and one-source topologies, which have nothing to arbitrate.
    #[serde(default)]
    pub policy: ArbitrationPolicyConfig,

    /// How recent a teleop reference must be (seconds) to override the autonomy
    /// reference. Older than this and the selector falls back to autonomy.
    #[serde(default = "default_teleop_max_age_s")]
    pub teleop_max_age_s: f64,
}

impl Default for ReferenceArbitrationConfig {
    fn default() -> Self {
        Self {
            sources: Vec::default(),
            policy: ArbitrationPolicyConfig::default(),
            teleop_max_age_s: DEFAULT_TELEOP_MAX_AGE_S,
        }
    }
}

/// Default teleop freshness window (seconds). A starting value — tune per
/// platform via the `[reference_arbitration]` config section.
const DEFAULT_TELEOP_MAX_AGE_S: f64 = 0.5;

fn default_teleop_max_age_s() -> f64 {
    DEFAULT_TELEOP_MAX_AGE_S
}

#[derive(Debug, Deserialize, Clone, Copy, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum ReferenceSource {
    Autonomy, // → control::autonomy::<T>()
    Teleop,   // → control::teleop::<T>()
}

impl ReferenceSource {
    /// The snake_case token this source deserializes from, for error messages
    /// that echo the user's config.
    pub fn as_str(self) -> &'static str {
        match self {
            ReferenceSource::Autonomy => "autonomy",
            ReferenceSource::Teleop => "teleop",
        }
    }
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum ArbitrationPolicyConfig {
    #[default]
    FreshnessOverride,
}
