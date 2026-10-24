//! A suite file, and `[[suite]]`, a run file applying one to agents.

use crate::{
    assertion::{target::WILDCARD, EntryName},
    evaluation::{AgentSeries, Evaluation, EvaluationDefaults},
};

use helios_core::kernel::identifier::is_level;
use helios_core::prelude::AgentId;

use serde::Deserialize;
use std::collections::HashSet;

/// _Pre-step._ A suite file: evaluations written once over agent series, with
/// no bounds, so nothing in it is tuned per run.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuiteConfig {
    /// `[[evaluate]]`, each entry's roles reading agent series. Names are
    /// unique within the suite.
    #[serde(default, rename = "evaluate")]
    pub evaluations: Vec<Evaluation<AgentSeries>>,
    /// `[evaluation]`, what its entries share unless one gives its own: how
    /// its numbers are computed, so it travels with the suite. The run's
    /// never applies to them, so a suite computes the same numbers in every
    /// run that applies it.
    #[serde(default, rename = "evaluation")]
    pub defaults: Option<EvaluationDefaults>,
}

/// _Pre-step._ One `[[suite]]`: the suite file `from` names, under the config
/// root, applied to `agents` under `name`. Each of its entries' metrics is
/// spelled `eval.agent.<agent>.<name>.<entry>.<metric>`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuiteApplication {
    /// Unique across the run file's assertions, measures, evaluations and
    /// suites.
    pub name: EntryName,
    pub from: String,
    pub agents: AgentSelection,
}

/// _Pre-step._ The agents a suite is applied to: those listed, or `"*"`, every
/// agent the run has once its pipelines exist.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(try_from = "AgentsAsWritten")]
pub enum AgentSelection {
    /// Each named once, in the order given.
    Listed(Vec<AgentId>),
    Every,
}

impl TryFrom<AgentsAsWritten> for AgentSelection {
    type Error = String;

    fn try_from(written: AgentsAsWritten) -> Result<Self, Self::Error> {
        let names = match written {
            AgentsAsWritten::Wildcard(spelling) if spelling == WILDCARD => {
                return Ok(AgentSelection::Every)
            }
            AgentsAsWritten::Wildcard(spelling) => {
                return Err(format!(
                    "agents is '{spelling}'; give a list of agents, or \"{WILDCARD}\" for every \
                     agent of the run"
                ))
            }
            AgentsAsWritten::Listed(names) => names,
        };
        if names.is_empty() {
            return Err(format!(
                "agents lists none; list one or more, or give \"{WILDCARD}\" for every agent of \
                 the run"
            ));
        }
        let mut seen = HashSet::new();
        for name in &names {
            if !is_level(name) {
                return Err(format!("agent '{name}' is not an agent's name, one level"));
            }
            if !seen.insert(name.as_str()) {
                return Err(format!("agent '{name}' is listed twice"));
            }
        }
        Ok(AgentSelection::Listed(
            names.into_iter().map(AgentId::new).collect(),
        ))
    }
}

/// `agents` as written: the wildcard, or a list of names.
#[derive(Deserialize)]
#[serde(untagged)]
enum AgentsAsWritten {
    Wildcard(String),
    Listed(Vec<String>),
}

/// _Pre-step._ A `[[suite]]` with the suite file it names, read.
#[derive(Debug, Clone, PartialEq)]
pub struct Suite {
    pub application: SuiteApplication,
    pub config: SuiteConfig,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn agents(written: &str) -> Result<AgentSelection, String> {
        #[derive(Deserialize)]
        struct Wrap {
            #[allow(dead_code)]
            agents: AgentSelection,
        }
        toml::from_str::<Wrap>(&format!("agents = {written}"))
            .map(|wrap| wrap.agents)
            .map_err(|error| error.to_string())
    }

    #[test]
    fn agents_are_a_list_or_the_wildcard() {
        assert_eq!(agents("\"*\""), Ok(AgentSelection::Every));
        assert_eq!(
            agents("[\"car\", \"truck\"]"),
            Ok(AgentSelection::Listed(vec![
                AgentId::new("car"),
                AgentId::new("truck")
            ]))
        );
    }

    #[test]
    fn a_selection_naming_no_agent_or_one_twice_is_refused() {
        for (written, why) in [
            ("\"car\"", "give a list of agents"),
            ("[]", "agents lists none"),
            ("[\"car\", \"car\"]", "listed twice"),
            ("[\"my car\"]", "not an agent's name"),
            ("[\"car.front\"]", "not an agent's name"),
        ] {
            let error = agents(written).expect_err(written);
            assert!(error.contains(why), "{written}: {error}");
        }
    }

    #[test]
    fn a_suite_file_reads_its_entries_over_agent_series_and_its_default() {
        let suite: SuiteConfig = toml::from_str(
            r#"
            [evaluation]
            gap_multiple = 3.0

            [[evaluate]]
            name = "twist"
            kind = "twist_tracking"
            [evaluate.inputs]
            truth = "truth.twist"
            reference = "reference"
            "#,
        )
        .expect("a valid suite");

        assert_eq!(
            suite.defaults.map(|defaults| defaults.gap_multiple.get()),
            Some(3.0)
        );
        let [twist] = suite.evaluations.as_slice() else {
            panic!("one entry");
        };
        assert_eq!(twist.inputs["truth"], AgentSeries::TruthTwist);
        assert_eq!(twist.inputs["reference"], AgentSeries::Reference);
    }

    #[test]
    fn a_suite_holds_no_bounds_and_no_selectors() {
        let bound = "[[assert]]\nname = \"a\"\ntarget = \"eval.inline.x.y\"\n\
                     condition = { less_than = 1.0 }\n";
        let selector = "[[evaluate]]\nname = \"e\"\nkind = \"twist\"\n[evaluate.inputs]\n\
                        twist = \"agent.car.channel.truth.twist\"\n";

        assert!(toml::from_str::<SuiteConfig>(bound).is_err());
        let error = toml::from_str::<SuiteConfig>(selector).expect_err("a path, not a series");
        assert!(
            error.to_string().contains("is not an agent series"),
            "{error}"
        );
    }

    #[test]
    fn an_application_names_its_suite_file_and_agents() {
        let application: SuiteApplication = toml::from_str(
            "name = \"tracking\"\nfrom = \"test/evaluations/twist.toml\"\nagents = \"*\"",
        )
        .expect("a valid application");

        assert_eq!(application.name.as_str(), "tracking");
        assert_eq!(application.from, "test/evaluations/twist.toml");
        assert_eq!(application.agents, AgentSelection::Every);
        assert!(toml::from_str::<SuiteApplication>(
            "name = \"t\"\nfrom = \"f\"\nagents = \"*\"\ngap_multiple = 2.0"
        )
        .is_err());
    }
}
