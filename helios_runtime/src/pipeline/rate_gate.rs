//! Per-node rate gating.
//!
//! One [`RateTimer`] lives in [`AutonomyPipeline`](super::AutonomyPipeline)'s
//! `rate_timers` vector per [`NodeId`](super::node::NodeId). Each tick,
//! the pipeline calls [`RateTimer::should_fire_and_advance`] for every
//! node; only nodes whose elapsed time has crossed their configured
//! period actually execute. Slow-tick double-firing is **not** corrected
//! — see `should_fire_and_advance` for details.

use std::sync::atomic::{AtomicU64, Ordering};

/// Accumulates elapsed time and reports when a node is due to fire.
///
/// `elapsed_micros` is an [`AtomicU64`] rather than a `Mutex<f64>` so
/// [`AutonomyPipeline::tick`](super::AutonomyPipeline::tick) can advance every
/// timer through a shared `&self` reference without acquiring any lock.
/// No panic can poison shared state across ticks, and a future
/// `par_iter`-within-a-level optimization remains a one-line change.
pub(crate) struct RateTimer {
    /// `None` means "fire every tick" (no rate limit). Set at
    /// construction; immutable thereafter.
    period_secs: Option<f64>,
    /// Microseconds since the last fire. Reset to `0` on each successful
    /// fire; never reset otherwise.
    elapsed_micros: AtomicU64,
}

impl RateTimer {
    /// Creates a timer with the given fire rate. `None` means the node
    /// fires every tick.
    pub(crate) fn new(rate_hz: Option<f64>) -> Self {
        let period_secs = rate_hz.map(|rate| 1.0f64 / rate);

        RateTimer {
            period_secs,
            elapsed_micros: AtomicU64::new(0u64),
        }
    }

    /// Adds `dt` to the accumulator and returns whether the node should
    /// execute on this tick.
    ///
    /// Returns:
    /// - `true` unconditionally when `period_secs` is `None`.
    /// - `true` when accumulated elapsed time has crossed the period —
    ///   the accumulator is reset to `0` before returning.
    /// - `false` otherwise.
    ///
    /// **No catch-up:** if a single `dt` covers multiple periods (e.g.
    /// `dt = 1.0` with `period = 0.5`), the node still fires exactly once.
    /// This matches the design intent — rate-gated nodes must not
    /// double-fire after a slow tick.
    ///
    /// **Concurrency:** with one caller per timer per tick (the only
    /// shape `AutonomyPipeline::tick` ever calls in), the `compare_exchange`
    /// always succeeds. The CAS is defensive: if a future change ever
    /// adds parallel within-level execution that touched the same timer,
    /// it would still fire at most once per period rather than
    /// `store(0)`-clobbering a concurrent increment.
    pub(crate) fn should_fire_and_advance(&self, dt: f64) -> bool {
        let Some(period) = self.period_secs else {
            return true;
        };

        let period_micros = (period * 1_000_000f64) as u64;
        let dt_micros = (dt * 1_000_000f64) as u64;

        let prev = self.elapsed_micros.fetch_add(dt_micros, Ordering::Relaxed);
        let new = prev + dt_micros;

        if new < period_micros {
            return false;
        }

        // Compare current with new time and swap with 0 if succeeeds
        self.elapsed_micros
            .compare_exchange(new, 0, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
    }
}
