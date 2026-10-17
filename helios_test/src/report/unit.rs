//! _Post-step._ [`ReportedUnit`]: the unit a judged or measured number is
//! read in, printed beside its value and bound.

use crate::assertion::ValueKind;

use helios_core::kernel::si_unit::SiUnit;

use serde::{Serialize, Serializer};

use std::fmt;

/// What `unit = "undeclared"` reads in a report.
const UNDECLARED: &str = "undeclared";

/// _Post-step._ The unit of the number an entry judged or measured. Known at
/// setup, from the leaf or extractor its series comes from and the reducer.
/// Bounds are written in it: a run file has no unit of its own, so a bound
/// meant in degrees on a value in radians shows beside the unit it is read
/// in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReportedUnit {
    Declared(SiUnit),
    /// A number whose source declares no unit: a bare `f64` channel.
    Undeclared,
}

impl ReportedUnit {
    /// The unit of a value of `kind` measured in `si_unit`; `None` for a
    /// bool or a string, which has no unit.
    pub fn of(kind: ValueKind, si_unit: Option<SiUnit>) -> Option<Self> {
        kind.is_number()
            .then(|| si_unit.map_or(Self::Undeclared, Self::Declared))
    }
}

impl fmt::Display for ReportedUnit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Declared(si_unit) => si_unit.fmt(f),
            Self::Undeclared => f.write_str(UNDECLARED),
        }
    }
}

/// Written as its symbol, or `"undeclared"`.
impl Serialize for ReportedUnit {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_a_number_has_a_unit() {
        let speed = Some(SiUnit::MetersPerSecond);
        assert_eq!(
            ReportedUnit::of(ValueKind::Float, speed),
            Some(ReportedUnit::Declared(SiUnit::MetersPerSecond))
        );
        assert_eq!(
            ReportedUnit::of(ValueKind::Int, None),
            Some(ReportedUnit::Undeclared)
        );
        assert_eq!(ReportedUnit::of(ValueKind::Bool, speed), None);
        assert_eq!(ReportedUnit::of(ValueKind::String, None), None);
    }
}
