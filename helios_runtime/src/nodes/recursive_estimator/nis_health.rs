//! [`NisWindow`]: whether one aiding sensor's recent innovations agree with
//! the spread the filter predicted for them.

use helios_core::estimation::Innovation;

use std::collections::VecDeque;

/// The latest NIS values of one aiding sensor, each divided by its degrees of
/// freedom, and the band their mean must stay inside.
///
/// For an honest filter NIS ÷ dof averages 1. A mean well above the band
/// means the filter is over-confident (its P too small, or the sensor's R) or
/// the sensor is biased; well below means it is too cautious. One sample says
/// little, so the judgement is made only once the window is full, over all of
/// it.
pub(crate) struct NisWindow {
    capacity: usize,
    low: f64,
    high: f64,
    ratios: VecDeque<f64>,
}

impl NisWindow {
    /// A window over the latest `capacity` applied readings, healthy while
    /// their mean NIS ÷ dof lies in `[low, high]`.
    ///
    /// Fails unless `capacity` is at least one and `0 ≤ low < high`, both
    /// finite.
    pub(crate) fn new(capacity: usize, [low, high]: [f64; 2]) -> Result<Self, String> {
        if capacity == 0 {
            return Err("window must hold at least one reading".to_string());
        }
        if !low.is_finite() || !high.is_finite() || low < 0.0 || low >= high {
            return Err(format!(
                "band must satisfy 0 ≤ low < high, both finite; got [{low}, {high}]"
            ));
        }
        Ok(Self {
            capacity,
            low,
            high,
            ratios: VecDeque::with_capacity(capacity),
        })
    }

    /// The mean NIS ÷ dof over the full window when it lies outside the band,
    /// or `None` while it is inside or the window is not yet full.
    pub(crate) fn out_of_band_mean(&self) -> Option<f64> {
        if self.ratios.len() < self.capacity {
            return None;
        }
        let mean = self.ratios.iter().sum::<f64>() / self.capacity as f64;
        (mean < self.low || mean > self.high).then_some(mean)
    }

    /// The band, for the reason a degraded estimate carries.
    pub(crate) fn band(&self) -> [f64; 2] {
        [self.low, self.high]
    }

    /// How many readings a full window holds.
    pub(crate) fn capacity(&self) -> usize {
        self.capacity
    }

    /// Adds one applied reading's innovation, dropping the oldest once full.
    /// An innovation with no degrees of freedom says nothing and is ignored.
    pub(crate) fn record(&mut self, innovation: Innovation) {
        if innovation.dof == 0 {
            return;
        }
        if self.ratios.len() == self.capacity {
            self.ratios.pop_front();
        }
        self.ratios
            .push_back(innovation.nis / innovation.dof as f64);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BAND: [f64; 2] = [0.5, 2.0];

    fn window(capacity: usize) -> NisWindow {
        NisWindow::new(capacity, BAND).expect("a valid window")
    }

    #[test]
    fn an_unfilled_window_judges_nothing() {
        let mut nis = window(3);
        nis.record(Innovation::new(30.0, 3));
        nis.record(Innovation::new(30.0, 3));
        assert_eq!(nis.out_of_band_mean(), None);
    }

    #[test]
    fn a_full_window_is_judged_on_its_mean_per_degree_of_freedom() {
        let mut nis = window(2);
        nis.record(Innovation::new(3.0, 3));
        nis.record(Innovation::new(6.0, 3));
        assert_eq!(nis.out_of_band_mean(), None);

        let mut high = window(2);
        high.record(Innovation::new(9.0, 3));
        high.record(Innovation::new(9.0, 3));
        assert_eq!(high.out_of_band_mean(), Some(3.0));

        let mut low = window(2);
        low.record(Innovation::new(0.75, 3));
        low.record(Innovation::new(0.75, 3));
        assert_eq!(low.out_of_band_mean(), Some(0.25));
    }

    #[test]
    fn the_oldest_reading_leaves_a_full_window() {
        let mut nis = window(2);
        nis.record(Innovation::new(30.0, 1));
        nis.record(Innovation::new(1.0, 1));
        nis.record(Innovation::new(1.0, 1));
        assert_eq!(nis.out_of_band_mean(), None);
    }

    #[test]
    fn a_zero_dof_innovation_is_ignored() {
        let mut nis = window(1);
        nis.record(Innovation::new(5.0, 0));
        assert_eq!(nis.out_of_band_mean(), None);
    }

    #[test]
    fn an_empty_window_or_a_bad_band_is_refused() {
        assert!(NisWindow::new(0, BAND).is_err());
        for band in [[2.0, 0.5], [1.0, 1.0], [-0.1, 2.0], [0.5, f64::INFINITY]] {
            assert!(NisWindow::new(5, band).is_err(), "{band:?}");
        }
    }
}
