//! _Post-step._ The wording a run's console and a batch's share, so an entry
//! is named, a reducer spelled and a unit placed the same way in both.

use super::ReportedUnit;
use crate::statistics::within_run::Reducer;

use helios_core::kernel::si_unit::SiUnit;

/// The significant figures a measured number prints with.
const SIGNIFICANT_FIGURES: usize = 4;

/// The magnitudes a measured number prints in fixed notation: below them
/// it would be mostly leading zeros, from their end the figures past the
/// significant ones would print as zeros the number doesn't have.
const FIXED_NOTATION: std::ops::Range<f64> = 1e-4..1e6;

/// An entry's name, with the agent and the instance it judged where its
/// target selected them: `surge_settled[agent=north]`,
/// `edges[instance=base_link]`.
pub(super) fn entry_name(name: &str, agent: Option<&str>, instance: Option<&str>) -> String {
    let mut named = name.to_string();
    if let Some(agent) = agent {
        named.push_str(&format!("[agent={agent}]"));
    }
    if let Some(instance) = instance {
        named.push_str(&format!("[instance={instance}]"));
    }
    named
}

/// `text` followed by the unit it is read in. A dimensionless number, an
/// undeclared unit and a bool or string print bare: the series bracket
/// says when a unit is undeclared.
pub(super) fn with_unit(text: String, unit: Option<ReportedUnit>) -> String {
    match unit {
        Some(ReportedUnit::Declared(si_unit)) if si_unit != SiUnit::Dimensionless => {
            format!("{text} {si_unit}")
        }
        _ => text,
    }
}

/// A measured number to [`SIGNIFICANT_FIGURES`], trailing zeros dropped,
/// in scientific notation outside [`FIXED_NOTATION`]. Only for a number
/// with no bound beside it, whose last figures could cross it unseen; the
/// TOML report keeps every figure.
pub(super) fn rounded(value: f64) -> String {
    if !value.is_finite() || value == 0.0 {
        return value.to_string();
    }
    if !FIXED_NOTATION.contains(&value.abs()) {
        return format!("{value:.*e}", SIGNIFICANT_FIGURES - 1);
    }
    let leading = value.abs().log10().floor() as i64;
    let decimals = (SIGNIFICANT_FIGURES as i64 - 1 - leading).max(0) as usize;
    let fixed = format!("{value:.decimals$}");
    if fixed.contains('.') {
        fixed.trim_end_matches('0').trim_end_matches('.').to_owned()
    } else {
        fixed
    }
}

/// A reducer as the run file names it, with its parameter: `mean`,
/// `time_above(1.5)`.
pub(super) fn reducer_phrase(reducer: &Reducer) -> String {
    match reducer {
        Reducer::TimeAbove(threshold) => format!("{}({threshold})", reducer.name()),
        other => other.name().to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_entry_is_named_with_the_agent_and_instance_it_judged() {
        assert_eq!(entry_name("speed", None, None), "speed");
        assert_eq!(
            entry_name("speed", Some("north"), None),
            "speed[agent=north]"
        );
        assert_eq!(
            entry_name("edges", None, Some("base")),
            "edges[instance=base]"
        );
        assert_eq!(
            entry_name("edges", Some("north"), Some("base")),
            "edges[agent=north][instance=base]"
        );
    }

    #[test]
    fn a_measured_number_prints_to_four_significant_figures() {
        assert_eq!(rounded(4.824156631304349), "4.824");
        assert_eq!(rounded(54.99483736141082), "54.99");
        assert_eq!(rounded(0.027054920768516195), "0.02705");
        assert_eq!(rounded(-0.25), "-0.25");
        assert_eq!(rounded(7.0), "7");
        assert_eq!(rounded(0.99996), "1");
        assert_eq!(rounded(123456.7), "123457");
        assert_eq!(rounded(0.0000123456), "1.235e-5");
        assert_eq!(rounded(2.5e7), "2.500e7");
        assert_eq!(rounded(0.0), "0");
        assert_eq!(rounded(f64::NAN), "NaN");
    }

    #[test]
    fn a_reducer_prints_with_its_parameter() {
        assert_eq!(reducer_phrase(&Reducer::TimeAbove(1.5)), "time_above(1.5)");
        assert_eq!(reducer_phrase(&Reducer::Mean), "mean");
    }
}
