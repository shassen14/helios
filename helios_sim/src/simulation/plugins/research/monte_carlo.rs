// helios_sim/src/simulation/plugins/research/monte_carlo.rs
//
// Monte Carlo configuration resource and run-result type.
// The outer loop in `helios_research` reads this; the Bevy plugin just holds the seed.

use bevy::prelude::Resource;

/// Injected as a Bevy resource so the simulation knows its run index and seed.
#[derive(Resource, Default, Clone, Debug)]
pub struct MonteCarloConfig {
    pub run_index: u32,
    pub seed: u64,
}

/// Per-run aggregate results extracted after a run completes.
#[derive(Default, Debug, Clone)]
pub struct RunResult {
    pub run_index: u32,
    pub seed: u64,
    /// Mean cross-track error (metres).
    pub mean_cte_m: f64,
    /// Max cross-track error (metres).
    pub max_cte_m: f64,
    /// Mean heading error (radians, absolute).
    pub mean_he_rad: f64,
    /// Rise time (seconds), if measured.
    pub rise_time_s: Option<f64>,
    /// Settling time (seconds), if measured.
    pub settling_time_s: Option<f64>,
    /// Overshoot percent.
    pub overshoot_pct: f64,
}
