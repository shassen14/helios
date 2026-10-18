//! `[[evaluate]]`: an evaluation declared in the run file, as it is read.

use crate::assertion::{AssertionTarget, EntryName};

use serde::Deserialize;
use std::collections::BTreeMap;

/// _Pre-step._ One `[[evaluate]]` entry: an evaluator kind, the series each
/// of its roles reads, and its parameters. Its metrics are derived series,
/// `eval.inline.<name>.<metric>`, which assertions and measures name as
/// they name any other.
///
/// Inputs and parameters are sub-tables, `[evaluate.inputs]` and
/// `[evaluate.params]`: inline tables over several lines are TOML 1.1 only,
/// which tools reading TOML 1.0 refuse.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Evaluation {
    /// Unique across the run file's assertions, measures and evaluations.
    pub name: EntryName,
    /// A kind in the evaluator registry.
    pub kind: String,
    /// Each of the kind's roles, with the series it reads. A series is
    /// recorded, never another evaluation's.
    pub inputs: BTreeMap<String, AssertionTarget>,
    /// Read by the kind, which refuses any it doesn't know.
    #[serde(default)]
    pub params: toml::Table,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Deserialize)]
    struct Wrap {
        evaluate: Vec<Evaluation>,
    }

    fn read(text: &str) -> Result<Vec<Evaluation>, toml::de::Error> {
        toml::from_str::<Wrap>(text).map(|wrap| wrap.evaluate)
    }

    #[test]
    fn an_entry_reads_its_inputs_and_params_from_sub_tables() {
        let evaluations = read(
            r#"
            [[evaluate]]
            name = "twist"
            kind = "twist_tracking"
            [evaluate.inputs]
            reference = "agent.car.channel.reference"
            truth = { path = "agent.car.channel.truth.twist", instance = "agent.car.base_link" }
            [evaluate.params]
            gain = 2.0
            "#,
        )
        .expect("a valid entry");

        let [twist] = evaluations.as_slice() else {
            panic!("one entry");
        };
        assert_eq!(twist.name.as_str(), "twist");
        assert_eq!(
            twist.inputs.keys().map(String::as_str).collect::<Vec<_>>(),
            ["reference", "truth"]
        );
        assert_eq!(twist.params.get("gain"), Some(&toml::Value::Float(2.0)));
    }

    #[test]
    fn an_unknown_key_is_refused_at_parse() {
        let unknown_key = "[[evaluate]]\nname = \"e\"\nkind = \"k\"\nwhen = 1\n[evaluate.inputs]\n";
        // A frame policy is a parameter of the kinds that relate frames.
        let align = "[[evaluate]]\nname = \"e\"\nkind = \"k\"\nalign = \"first_pose\"\n\
                     [evaluate.inputs]\n";

        assert!(read(unknown_key).is_err());
        assert!(read(align).is_err());
    }
}
