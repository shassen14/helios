//! [`SiUnit`]: the SI unit a number is measured in, declared beside it so a
//! reader never has to guess whether `5` is radians or degrees.

use std::fmt;

/// The SI unit of a number: one a node reports for watchers, or a
/// component of a payload.
///
/// Runtime values are SI, so a unit here is never a scale to convert from,
/// only the quantity's name: degrees and other config-only units have no
/// variant. A number that counts or compares things (a drop, a normalized
/// error) is [`Dimensionless`](Self::Dimensionless).
///
/// A name, not an algebra: units don't compose, so a quantity in a unit not
/// listed (a squared one included) adds its variant here, and every reader
/// that prints units prints it. Exhaustive on purpose for the same reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SiUnit {
    /// A count, a ratio or a normalized value: NIS, a drop.
    Dimensionless,
    Seconds,
    Meters,
    Kilograms,
    Radians,
    MetersPerSecond,
    MetersPerSecondSquared,
    RadiansPerSecond,
    Microtesla,
}

impl SiUnit {
    /// The unit's symbol, as a report prints it beside a value.
    pub fn symbol(self) -> &'static str {
        match self {
            Self::Dimensionless => "dimensionless",
            Self::Seconds => "s",
            Self::Meters => "m",
            Self::Kilograms => "kg",
            Self::Radians => "rad",
            Self::MetersPerSecond => "m/s",
            Self::MetersPerSecondSquared => "m/s^2",
            Self::RadiansPerSecond => "rad/s",
            Self::Microtesla => "µT",
        }
    }
}

impl fmt::Display for SiUnit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.symbol())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_unit_displays_as_its_symbol() {
        assert_eq!(SiUnit::RadiansPerSecond.to_string(), "rad/s");
        assert_eq!(SiUnit::Dimensionless.to_string(), "dimensionless");
    }
}
