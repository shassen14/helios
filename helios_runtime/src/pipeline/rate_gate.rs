use std::sync::atomic::{AtomicU64, Ordering};

pub(crate) struct RateTimer {
    period_secs: Option<f64>,
    elapsed_micros: AtomicU64,
}

impl RateTimer {
    pub(crate) fn new(rate_hz: Option<f64>) -> Self {
        let period_secs = rate_hz.map(|rate| 1.0f64 / rate);

        RateTimer {
            period_secs,
            elapsed_micros: AtomicU64::new(0u64),
        }
    }

    pub(crate) fn should_fire_and_advance(&self, dt: f64) -> bool {
        let Some(period) = self.period_secs else {
            return false;
        };

        let period_micros = (period * 1_000_000f64) as u64;
        let dt_micros = (dt * 1_000_000f64) as u64;

        let prev = self.elapsed_micros.fetch_add(dt_micros, Ordering::Relaxed);

        let new = prev + dt_micros;

        if new < period_micros {
            return false;
        }

        // last time fired and now have met the run rate.
        // attempt to reset elapsed time to be 0
        // if another thread already incremented, then the compare
        // and swap fails - letting one to claim the update
        self.elapsed_micros
            .compare_exchange(new, 0, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
    }
}
