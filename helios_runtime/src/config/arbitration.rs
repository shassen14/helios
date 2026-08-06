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
    /// and kind pick the topology: empty infers `[Autonomy]` when a controller
    /// exists (else a build error); a lone internal source (autonomy) writes
    /// `command` directly with no arbiter; a lone host source (teleop) or two or
    /// more sources insert a `Selector`.
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

#[derive(Debug, Deserialize, Clone, Copy, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum CommandSource {
    Autonomy, // → control::autonomy::<T>()
    Teleop,   // → control::teleop::<T>()
}

impl CommandSource {
    /// The snake_case token this source deserializes from, for error messages
    /// that echo the user's config.
    pub fn as_str(self) -> &'static str {
        match self {
            CommandSource::Autonomy => "autonomy",
            CommandSource::Teleop => "teleop",
        }
    }

    /// Whether this source's command arrives from outside the DAG (the host)
    /// rather than from an internal producer node.
    ///
    /// This is the fork in command-terminal topology. An internal source
    /// (autonomy — its controller is a DAG node) can be retargeted to write
    /// `command` directly, so alone it needs no arbiter. A host-published source
    /// (teleop) is written from beyond the graph and cannot be retargeted, so
    /// even alone it needs a synthesized `Selector` relaying its channel onto
    /// `command`. See `assembler::command::resolve_command_topology`.
    pub fn is_host_published(self) -> bool {
        match self {
            CommandSource::Autonomy => false,
            CommandSource::Teleop => true,
        }
    }
}

#[derive(Debug, Deserialize, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum ArbitrationPolicyConfig {
    #[default]
    FreshnessOverride,
}
