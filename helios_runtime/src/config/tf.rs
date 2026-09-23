//! Estimated transform-buffer sizing — the bounded per-edge history the
//! `TfService`'s `TfBuffer` keeps.
//!
//! Portable, not sim-specific: the buffer that answers a node's transform query
//! is the same in simulation and on hardware, so its sizing lives beside the rest
//! of the autonomy stack rather than in a host. Both fields carry conservative
//! defaults so a stack that never mentions `[tf]` still buffers sensibly; a
//! high-rate or long-latency stack overrides them in TOML.

use helios_core::prelude::MonotonicDuration;
use helios_core::frames::transforms::tf::buffer::TfWindow;

use serde::Deserialize;

/// How far back the estimated tf buffer remembers, in seconds. One second
/// comfortably covers a consumer asking for a transform several control ticks
/// old without unbounded growth.
const DEFAULT_HORIZON_SECS: f64 = 1.0;

/// Per-edge sample backstop. Sized so a single dynamic edge at a typical host
/// tick rate stays inside the horizon before this bound bites; hitting it means
/// the edge produces faster than the horizon assumed.
const DEFAULT_MAX_SAMPLES: usize = 256;

/// Sizing for the estimated transform buffer, resolved into a core [`TfWindow`]
/// at spawn.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TfBufferConfig {
    /// Oldest sample a lookup can reach, relative to an edge's newest, in seconds.
    #[serde(default = "default_horizon_secs")]
    pub horizon_secs: f64,
    /// Per-edge memory backstop, in samples.
    #[serde(default = "default_max_samples")]
    pub max_samples: usize,
}

impl TfBufferConfig {
    /// The core buffer policy this config resolves to.
    pub fn to_window(&self) -> TfWindow {
        TfWindow {
            horizon: MonotonicDuration(self.horizon_secs),
            max_samples: self.max_samples,
        }
    }
}

impl Default for TfBufferConfig {
    fn default() -> Self {
        Self {
            horizon_secs: DEFAULT_HORIZON_SECS,
            max_samples: DEFAULT_MAX_SAMPLES,
        }
    }
}

fn default_horizon_secs() -> f64 {
    DEFAULT_HORIZON_SECS
}

fn default_max_samples() -> usize {
    DEFAULT_MAX_SAMPLES
}
