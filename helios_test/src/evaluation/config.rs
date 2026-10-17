//! `[[evaluate]]`: an evaluation declared in the run file, as it is read.

use crate::assertion::{AssertionTarget, EntryName};

use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, fmt};

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
    #[serde(default)]
    pub align: Align,
    /// Each of the kind's roles, with the series it reads. A series is
    /// recorded, never another evaluation's.
    pub inputs: BTreeMap<String, AssertionTarget>,
    /// Read by the kind, which refuses any it doesn't know.
    #[serde(default)]
    pub params: toml::Table,
}

/// _Shared (pre + post)._ How an evaluation relates frames its inputs don't
/// share. Only `declared` is built: the inputs must be comparable in frames
/// they name, or the kind refuses them at setup.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Align {
    #[default]
    Declared,
    /// Align the two trees on their first common sample.
    FirstPose,
    /// Align the two trees by a least-squares fit over the run.
    BestFit,
    /// Compare motion over a window, needing no common frame.
    Relative,
}

impl Align {
    /// The first case that will need this policy built, or `None` for one
    /// that is built.
    pub fn unbuilt_until(&self) -> Option<&'static str> {
        match self {
            Align::Declared => None,
            Align::FirstPose => Some("an estimator with no anchor, graded on a short run"),
            Align::BestFit => Some("the first hardware or SLAM run"),
            Align::Relative => Some("drifting odometry graded between loop closures"),
        }
    }
}

impl fmt::Display for Align {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Align::Declared => "declared",
            Align::FirstPose => "first_pose",
            Align::BestFit => "best_fit",
            Align::Relative => "relative",
        })
    }
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
        assert_eq!(twist.align, Align::Declared);
        assert_eq!(
            twist.inputs.keys().map(String::as_str).collect::<Vec<_>>(),
            ["reference", "truth"]
        );
        assert_eq!(twist.params.get("gain"), Some(&toml::Value::Float(2.0)));
    }

    #[test]
    fn every_policy_parses_and_only_declared_is_built() {
        for (spelled, align) in [
            ("declared", Align::Declared),
            ("first_pose", Align::FirstPose),
            ("best_fit", Align::BestFit),
            ("relative", Align::Relative),
        ] {
            let text = format!(
                "[[evaluate]]\nname = \"e\"\nkind = \"k\"\nalign = \"{spelled}\"\n\
                 [evaluate.inputs]\n"
            );
            let [evaluation] = read(&text).expect("a valid entry").try_into().expect("one");
            assert_eq!(evaluation.align, align);
            assert_eq!(align.to_string(), spelled);
            assert_eq!(align.unbuilt_until().is_none(), align == Align::Declared);
        }
    }

    #[test]
    fn an_unknown_key_or_policy_is_refused_at_parse() {
        let unknown_key = "[[evaluate]]\nname = \"e\"\nkind = \"k\"\nwhen = 1\n[evaluate.inputs]\n";
        let unknown_policy =
            "[[evaluate]]\nname = \"e\"\nkind = \"k\"\nalign = \"guess\"\n[evaluate.inputs]\n";

        assert!(read(unknown_key).is_err());
        assert!(read(unknown_policy).is_err());
    }
}
