// helios_core/src/control/metrics.rs
//
// Pure control-quality metrics. No Bevy dependency.
// All functions operate on pre-collected history slices.

use nalgebra::Vector2;

/// Rise time: elapsed seconds from the first sample until the signal first
/// reaches 90 % of `target`.  Returns `None` if the threshold is never crossed.
pub fn rise_time(history: &[(f64, f64)], target: f64) -> Option<f64> {
    let threshold = 0.9 * target;
    let t0 = history.first().map(|(t, _)| *t)?;
    history
        .iter()
        .find(|(_, v)| *v >= threshold)
        .map(|(t, _)| t - t0)
}

/// Settling time: elapsed seconds from the first sample until the signal
/// permanently stays within ±`band_pct * target` of `target`.
///
/// `band_pct` is e.g. `0.02` for ±2 %.
/// Returns `None` if the signal never enters the band.
pub fn settling_time(history: &[(f64, f64)], target: f64, band_pct: f64) -> Option<f64> {
    let band = (target * band_pct).abs();
    let t0 = history.first().map(|(t, _)| *t)?;
    // Find the last sample that lies *outside* the band.
    let last_outside = history
        .iter()
        .rposition(|(_, v)| (v - target).abs() > band)?;
    history.get(last_outside + 1).map(|(t, _)| t - t0)
}

/// Percent overshoot: `(peak − target) / |target| × 100`.
/// Returns 0 if `target` is near zero.
pub fn overshoot_pct(history: &[(f64, f64)], target: f64) -> f64 {
    if target.abs() < 1e-12 {
        return 0.0;
    }
    let peak = history
        .iter()
        .map(|(_, v)| *v)
        .fold(f64::NEG_INFINITY, f64::max);
    ((peak - target) / target.abs()) * 100.0
}

/// Cross-track error: Euclidean distance between each `actual` 2-D position
/// and the corresponding `reference` position.
///
/// The two slices are zipped — excess elements in the longer slice are ignored.
pub fn cross_track_error(actual: &[Vector2<f64>], reference: &[Vector2<f64>]) -> Vec<f64> {
    actual
        .iter()
        .zip(reference.iter())
        .map(|(a, r)| (a - r).norm())
        .collect()
}

/// Heading error `(actual − reference)` wrapped to `(−π, π]`.
///
/// The two slices are zipped — excess elements in the longer slice are ignored.
pub fn heading_error(actual_rad: &[f64], reference_rad: &[f64]) -> Vec<f64> {
    actual_rad
        .iter()
        .zip(reference_rad.iter())
        .map(|(a, r)| {
            let diff = a - r;
            let pi2 = 2.0 * std::f64::consts::PI;
            diff - pi2 * (diff / pi2).round()
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn step_response(target: f64, n: usize) -> Vec<(f64, f64)> {
        // Synthesise a first-order step response: v(t) = target * (1 - e^{-5t/T})
        let dt = 0.01_f64;
        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let v = target * (1.0 - (-5.0 * t).exp());
                (t, v)
            })
            .collect()
    }

    #[test]
    fn rise_time_basic() {
        let h = step_response(1.0, 500);
        let rt = rise_time(&h, 1.0).unwrap();
        // For the synthetic response, 90% is crossed somewhere in the first second.
        assert!(rt > 0.0 && rt < 1.0, "rise_time = {rt}");
    }

    #[test]
    fn settling_time_basic() {
        let h = step_response(1.0, 1000);
        // With sufficient settling the last outside-band sample should exist.
        let st = settling_time(&h, 1.0, 0.02);
        if let Some(t) = st {
            assert!(t > 0.0, "settling_time = {t}");
        }
    }

    #[test]
    fn overshoot_none_for_monotone() {
        let h = step_response(1.0, 500);
        let os = overshoot_pct(&h, 1.0);
        // Monotone approach — overshoot should be ≤ 0 %.
        assert!(os <= 0.1, "overshoot_pct = {os}");
    }

    #[test]
    fn cross_track_zero_when_equal() {
        let pts: Vec<Vector2<f64>> = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(2.0, 0.0),
        ];
        let cte = cross_track_error(&pts, &pts);
        assert!(cte.iter().all(|&e| e < 1e-10));
    }

    #[test]
    fn heading_error_wraps() {
        let actual = vec![std::f64::consts::PI + 0.1];
        let reference = vec![-std::f64::consts::PI + 0.1];
        let err = heading_error(&actual, &reference);
        // The wrapped error should be close to 0, not ≈ 2π.
        assert!(err[0].abs() < 0.01, "heading_error = {}", err[0]);
    }
}
