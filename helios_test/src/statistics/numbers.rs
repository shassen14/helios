//! The math every reduction shares, within a run or across runs: a bag of
//! numbers to one. Each is `None` on an empty bag, and NaN when any number
//! is NaN, so a NaN reaches the reduced value wherever it came from.

/// The arithmetic mean.
pub fn mean(numbers: &[f64]) -> Option<f64> {
    if numbers.is_empty() {
        return None;
    }
    Some(numbers.iter().sum::<f64>() / numbers.len() as f64)
}

/// The population standard deviation (divided by N): the spread of these
/// numbers, not an estimate of a wider population's. `0.0` for one number.
pub fn std(numbers: &[f64]) -> Option<f64> {
    let m = mean(numbers)?;
    let variance = numbers.iter().map(|x| (x - m).powi(2)).sum::<f64>() / numbers.len() as f64;
    Some(variance.sqrt())
}

/// The sample standard deviation (divided by N − 1): an estimate of the
/// spread of the population these numbers were drawn from, as a batch's runs
/// are drawn from every seed. `None` for fewer than two numbers, which
/// estimate no spread.
pub fn sample_std(numbers: &[f64]) -> Option<f64> {
    let m = mean(numbers)?;
    let degrees_of_freedom = numbers.len().checked_sub(1).filter(|n| *n > 0)?;
    let variance = numbers.iter().map(|x| (x - m).powi(2)).sum::<f64>() / degrees_of_freedom as f64;
    Some(variance.sqrt())
}

/// The root mean square.
pub fn rms(numbers: &[f64]) -> Option<f64> {
    let squares: Vec<f64> = numbers.iter().map(|x| x * x).collect();
    mean(&squares).map(f64::sqrt)
}

/// The smallest number.
pub fn min(numbers: &[f64]) -> Option<f64> {
    extreme(numbers, f64::min)
}

/// The largest number.
pub fn max(numbers: &[f64]) -> Option<f64> {
    extreme(numbers, f64::max)
}

/// The value `percent` of the way through the sorted numbers, interpolated
/// linearly between the two closest ranks, so the 0th is the smallest, the
/// 100th the largest and the 50th the median. `percent` is in [0, 100]; the
/// caller checks.
pub fn percentile(numbers: &[f64], percent: f64) -> Option<f64> {
    if numbers.iter().any(|x| x.is_nan()) {
        return Some(f64::NAN);
    }
    let mut sorted = numbers.to_vec();
    sorted.sort_by(f64::total_cmp);
    let last = sorted.len().checked_sub(1)?;
    let rank = percent / 100.0 * last as f64;
    let below = rank.floor() as usize;
    let above = rank.ceil() as usize;
    let (low, high) = (sorted.get(below)?, sorted.get(above)?);
    Some(low + (high - low) * (rank - below as f64))
}

/// The number `pick` keeps of every pair, NaN when any is NaN: `f64::min`
/// and `f64::max` return the other argument of a NaN, which would drop it.
fn extreme(numbers: &[f64], pick: fn(f64, f64) -> f64) -> Option<f64> {
    if numbers.iter().any(|x| x.is_nan()) {
        return Some(f64::NAN);
    }
    numbers.iter().copied().reduce(pick)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-12;

    /// Mean 5, population std 2, min 2, max 9.
    const KNOWN: [f64; 8] = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];

    #[test]
    fn known_numbers_reduce_to_known_values() {
        assert_eq!(mean(&KNOWN), Some(5.0));
        assert!((std(&KNOWN).expect("numbers") - 2.0).abs() < TOL);
        assert_eq!(min(&KNOWN), Some(2.0));
        assert_eq!(max(&KNOWN), Some(9.0));
        assert!((rms(&[3.0, 4.0]).expect("numbers") - 12.5_f64.sqrt()).abs() < TOL);
    }

    #[test]
    fn std_of_one_number_is_zero() {
        assert_eq!(std(&[3.0]), Some(0.0));
    }

    #[test]
    fn sample_std_divides_by_one_less_and_needs_two_numbers() {
        let sample = sample_std(&KNOWN).expect("numbers");

        assert!((sample - (32.0_f64 / 7.0).sqrt()).abs() < TOL);
        assert_eq!(sample_std(&[3.0]), None);
    }

    #[test]
    fn percentiles_interpolate_between_ranks() {
        let numbers = [4.0, 1.0, 3.0, 2.0];

        assert_eq!(percentile(&numbers, 0.0), Some(1.0));
        assert_eq!(percentile(&numbers, 100.0), Some(4.0));
        assert_eq!(percentile(&numbers, 50.0), Some(2.5));
        assert!((percentile(&numbers, 95.0).expect("numbers") - 3.85).abs() < TOL);
        assert_eq!(percentile(&[7.0], 95.0), Some(7.0));
    }

    #[test]
    fn an_empty_bag_has_no_value() {
        for reduce in [mean, std, sample_std, rms, min, max] {
            assert_eq!(reduce(&[]), None);
        }
        assert_eq!(percentile(&[], 50.0), None);
    }

    #[test]
    fn a_nan_reaches_every_reduction() {
        let numbers = [1.0, f64::NAN, 3.0];

        for reduce in [mean, std, sample_std, rms, min, max] {
            assert!(reduce(&numbers).is_some_and(f64::is_nan));
        }
        assert!(percentile(&numbers, 50.0).is_some_and(f64::is_nan));
    }
}
