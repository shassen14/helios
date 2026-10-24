//! _Post-step._ The wording a run's console and a batch's share, so an entry
//! is named, a reducer spelled and a unit placed the same way in both.

use super::ReportedUnit;
use crate::statistics::within_run::Reducer;

use helios_core::kernel::si_unit::SiUnit;

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
    fn a_reducer_prints_with_its_parameter() {
        assert_eq!(reducer_phrase(&Reducer::TimeAbove(1.5)), "time_above(1.5)");
        assert_eq!(reducer_phrase(&Reducer::Mean), "mean");
    }
}
