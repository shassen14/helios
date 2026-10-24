//! `[evaluate.params]` of a `contacts` entry: how far apart two touches of
//! one side may be and still be one collision.

use serde::Deserialize;

/// _Pre-step._ A `contacts` entry's parameters.
///
/// `merge_gap` has no default: it decides what one collision is (a car
/// pinned on a corner breaks contact for a tick at a time), so a default
/// would make that choice where the report doesn't show it.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContactsConfig {
    /// Seconds: a side touched again no more than this after it was last
    /// touched continues its episode. `0` makes every re-touch a new one.
    pub merge_gap: f64,
}

impl ContactsConfig {
    /// Why these parameters can't count episodes: a gap that is negative or
    /// not finite.
    pub fn check(&self) -> Result<(), String> {
        if self.merge_gap.is_finite() && self.merge_gap >= 0.0 {
            Ok(())
        } else {
            Err(format!(
                "merge_gap = {} must be finite and not negative: it is the seconds a side may go \
                 untouched and still be the same collision",
                self.merge_gap
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(text: &str) -> Result<ContactsConfig, toml::de::Error> {
        toml::from_str(text)
    }

    #[test]
    fn a_gap_of_zero_or_more_is_read() {
        let zero = read("merge_gap = 0.0").expect("a gap");
        let one = read("merge_gap = 1.0").expect("a gap");

        assert_eq!(zero.check(), Ok(()));
        assert_eq!(one.merge_gap, 1.0);
        assert_eq!(one.check(), Ok(()));
    }

    #[test]
    fn no_gap_and_an_unknown_key_are_refused() {
        assert!(read("").is_err());
        assert!(read("merge_gap = 1.0\nthreshold = 2.0").is_err());
    }

    #[test]
    fn a_negative_or_unbounded_gap_is_refused() {
        for text in ["merge_gap = -0.5", "merge_gap = nan", "merge_gap = inf"] {
            let refusal = read(text).expect("parses").check().expect_err(text);

            assert!(refusal.contains("finite and not negative"), "{refusal}");
        }
    }
}
