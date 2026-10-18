//! The rate limit on the estimator's skip warnings, shared by its predict and
//! its aiding sources.

use atomic_float::AtomicF64;
use std::sync::atomic::Ordering;

/// Minimum seconds of pipeline-clock time between two skip warnings from one
/// source: an aiding source's "aiding dropped", or the node's "predict
/// skipped". A missing extrinsic, a corrupt covariance or a misshapen input
/// fails on *every* reading or tick, so without a throttle the log floods at
/// sensor rate; the first fault always prints and later ones inside this window
/// are suppressed. Keyed on reading or tick time (not wall time) so the gate is
/// deterministic and testable.
pub(super) const SKIP_WARN_MIN_INTERVAL_SECS: f64 = 5.0;

/// Rate-limit gate shared by the skip warnings. Returns `true` at most once per
/// [`SKIP_WARN_MIN_INTERVAL_SECS`] of `at`, and records `at` in `last_warned`
/// when it does. A latch seeded with `NEG_INFINITY` lets the first fault pass.
pub(super) fn passes_warn_throttle(last_warned: &AtomicF64, at: f64) -> bool {
    let last = last_warned.load(Ordering::Relaxed);
    if at - last < SKIP_WARN_MIN_INTERVAL_SECS {
        return false;
    }
    last_warned.store(at, Ordering::Relaxed);
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The throttle lets the first fault through, holds the next ones for
    /// the interval, then lets one through again.
    #[test]
    fn the_warn_throttle_passes_once_per_interval() {
        let last = AtomicF64::new(f64::NEG_INFINITY);
        assert!(passes_warn_throttle(&last, 1.0));
        assert!(!passes_warn_throttle(
            &last,
            1.0 + SKIP_WARN_MIN_INTERVAL_SECS / 2.0
        ));
        assert!(passes_warn_throttle(
            &last,
            1.0 + SKIP_WARN_MIN_INTERVAL_SECS
        ));
    }
}
