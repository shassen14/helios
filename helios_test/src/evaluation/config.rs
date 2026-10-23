//! `[[evaluate]]`: an evaluation declared in the run file, as it is read,
//! and `[evaluation]`, what every entry shares.

use crate::assertion::{AssertionTarget, EntryName};

use serde::Deserialize;
use std::collections::BTreeMap;

/// _Pre-step._ One `[[evaluate]]` entry: an evaluator kind, the series each
/// of its roles reads, and its parameters. Its metrics are derived series,
/// `eval.inline.<name>.<metric>`, which assertions and measures name as
/// they name any other.
///
/// Each role's series is an `I`: a recorded series' selector in a run file,
/// an [`AgentSeries`](super::AgentSeries) in a suite, which binding to an
/// agent turns into the selector.
///
/// Inputs and parameters are sub-tables, `[evaluate.inputs]` and
/// `[evaluate.params]`: inline tables over several lines are TOML 1.1 only,
/// which tools reading TOML 1.0 refuse.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Evaluation<I = AssertionTarget> {
    /// Unique across the run file's assertions, measures and evaluations.
    pub name: EntryName,
    /// A kind in the evaluator registry.
    pub kind: String,
    /// Each of the kind's roles, with the series it reads. A series is
    /// recorded, never another evaluation's.
    pub inputs: BTreeMap<String, I>,
    /// Read by the kind, which refuses any it doesn't know.
    #[serde(default)]
    pub params: toml::Table,
    /// The widest gap each capped role named spans, in place of the run's
    /// `gap_multiple`, keyed by role as `inputs` is.
    #[serde(default)]
    pub gap: BTreeMap<String, Gap>,
}

/// _Pre-step._ `[evaluation]`: what every `[[evaluate]]` entry shares unless
/// it gives its own.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvaluationDefaults {
    /// The widest gap a capped role spans, in periods of its input's
    /// declared rate. Policy for this run, so it shows in its report.
    pub gap_multiple: GapWidth,
}

/// _Pre-step._ One role's `[evaluate.gap]`: the widest gap it spans, given
/// as one of its two keys.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(try_from = "GapTable")]
pub enum Gap {
    /// In periods of the input's declared rate.
    Multiple(GapWidth),
    /// In seconds whatever the input's rate: the one cap an input with no
    /// declared rate can have.
    Seconds(GapWidth),
}

impl TryFrom<GapTable> for Gap {
    type Error = &'static str;

    fn try_from(table: GapTable) -> Result<Self, Self::Error> {
        match (table.multiple, table.seconds) {
            (Some(multiple), None) => Ok(Gap::Multiple(multiple)),
            (None, Some(seconds)) => Ok(Gap::Seconds(seconds)),
            (Some(_), Some(_)) => Err("a gap gives multiple and seconds; give one"),
            (None, None) => Err("a gap gives neither multiple nor seconds; give one"),
        }
    }
}

/// A role's `[evaluate.gap]` as written, before [`Gap`] takes one of its
/// keys.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GapTable {
    multiple: Option<GapWidth>,
    seconds: Option<GapWidth>,
}

/// _Pre-step._ A gap's number, a multiple or seconds: positive and finite,
/// so it spans some gap and not every one.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(try_from = "f64")]
pub struct GapWidth(f64);

impl GapWidth {
    pub fn get(self) -> f64 {
        self.0
    }
}

impl TryFrom<f64> for GapWidth {
    type Error = String;

    fn try_from(value: f64) -> Result<Self, Self::Error> {
        if value.is_finite() && value > 0.0 {
            Ok(Self(value))
        } else {
            Err(format!("a gap is {value}; give a positive, finite number"))
        }
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

    /// The `reference` role's gap, as `[evaluate.gap.reference]` gives it.
    fn reference_gap(gap: &str) -> Result<Option<Gap>, toml::de::Error> {
        read(&format!(
            "[[evaluate]]\nname = \"e\"\nkind = \"k\"\n[evaluate.inputs]\n\
             [evaluate.gap.reference]\n{gap}\n"
        ))
        .map(|evaluations| evaluations[0].gap.get("reference").copied())
    }

    fn width(value: f64) -> GapWidth {
        GapWidth::try_from(value).expect("positive and finite")
    }

    #[test]
    fn a_roles_gap_is_a_multiple_or_seconds_and_never_both() {
        assert_eq!(
            reference_gap("multiple = 2.0").ok(),
            Some(Some(Gap::Multiple(width(2.0))))
        );
        assert_eq!(
            reference_gap("seconds = 0.5").ok(),
            Some(Some(Gap::Seconds(width(0.5))))
        );
        let both = reference_gap("multiple = 2.0\nseconds = 0.5").expect_err("both keys");
        let neither = reference_gap("").expect_err("no key");
        assert!(
            both.to_string()
                .contains("a gap gives multiple and seconds; give one"),
            "{both}"
        );
        assert!(
            neither
                .to_string()
                .contains("a gap gives neither multiple nor seconds"),
            "{neither}"
        );
    }

    #[test]
    fn a_gap_that_spans_nothing_or_everything_is_refused_at_parse() {
        for refused in [
            "seconds = 0.0",
            "multiple = -1.0",
            "seconds = inf",
            "multiple = nan",
        ] {
            let error = reference_gap(refused).expect_err(refused);
            assert!(
                error.to_string().contains("give a positive, finite number"),
                "{refused}: {error}"
            );
        }
        assert!(toml::from_str::<EvaluationDefaults>("gap_multiple = 0.0").is_err());
        assert_eq!(
            toml::from_str::<EvaluationDefaults>("gap_multiple = 3.0")
                .map(|defaults| defaults.gap_multiple.get())
                .ok(),
            Some(3.0)
        );
    }
}
