use crate::assertion::target::AssertionTarget;

use serde::{
    de::{self, value::MapAccessDeserializer, MapAccess, Visitor},
    ser::SerializeMap,
    Deserialize, Deserializer, Serialize, Serializer,
};
use std::fmt;
use thiserror::Error;

/// How a run file names the run's first tick.
const RUN_START: &str = "run.start";

/// How a run file names the run's last tick.
const RUN_END: &str = "run.end";

/// The keys of a bound's table form.
const AT_KEY: &str = "at";
const AFTER_KEY: &str = "after";
const PLUS_KEY: &str = "plus";

/// _Shared (pre + per + post)._ The span an assertion or measure reads:
/// samples stamped from `start` to `end`, both included. An absent `start`
/// takes everything before; an absent `end` runs to `run.end`. A sample
/// belongs to a window by its stamp, never by when it arrived.
///
/// `window = { start = 40.0 }`, `window = { start = { at = "run.end", plus =
/// -5.0 } }`, or `start = { after = "<selector>", plus = 2.0 }`.
#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Window {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start: Option<Bound>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<Bound>,
}

impl Window {
    /// Whether the window is the whole run: no bound on either side.
    pub fn is_whole_run(&self) -> bool {
        self.start.is_none() && self.end.is_none()
    }

    /// Whether every bound present is seconds after `run.start`, so the
    /// window is known from the first tick and a sample can be judged as it
    /// arrives. A bound from `run.end` is unknown until the run stops, and
    /// one from `after` until its anchor occurs.
    pub fn fixed(&self) -> bool {
        self.bounds()
            .all(|(_, bound)| bound.secs_after_start().is_some())
    }

    /// The selector of each `after` bound, start first.
    pub fn anchors(&self) -> impl Iterator<Item = &AssertionTarget> {
        self.bounds().filter_map(|(_, bound)| match bound {
            Bound::After { after, .. } => Some(after),
            Bound::Secs(_) | Bound::At { .. } => None,
        })
    }

    /// Why a run whose budget is `budget` seconds could never cover this
    /// window, if it couldn't. The budget is the only way a run ends, and a
    /// window bounded from `run.end` is covered only when the run ran out,
    /// so `run.end` is read as the budget: every bound but `after` resolves
    /// here.
    pub fn check(&self, budget: f64) -> Result<(), WindowProblem> {
        for (side, bound) in self.bounds() {
            bound.check(side)?;
            // An offset from `run.end` reaching back past the run's start.
            if let Some(secs) = bound.secs_at_budget(budget).filter(|secs| *secs < 0.0) {
                return Err(WindowProblem::BeforeRunStart { side, plus: secs });
            }
        }

        let start = match &self.start {
            Some(bound) => bound.secs_at_budget(budget),
            None => Some(0.0),
        };
        let end = match &self.end {
            Some(bound) => bound.secs_at_budget(budget),
            None => Some(budget),
        };
        if let Some(
            Bound::Secs(_)
            | Bound::At {
                at: RunInstant::Start,
                ..
            },
        ) = &self.end
        {
            if let Some(end) = end.filter(|end| *end > budget) {
                return Err(WindowProblem::PastBudget { end, budget });
            }
        }
        if let (Some(start), Some(end)) = (start, end) {
            if end <= start {
                return Err(WindowProblem::Empty { start, end });
            }
        }
        if let (
            Some(Bound::After {
                after: start_anchor,
                plus: start,
            }),
            Some(Bound::After {
                after: end_anchor,
                plus: end,
            }),
        ) = (&self.start, &self.end)
        {
            if start_anchor == end_anchor && end <= start {
                return Err(WindowProblem::Empty {
                    start: *start,
                    end: *end,
                });
            }
        }
        Ok(())
    }

    /// Each bound present, with its side.
    fn bounds(&self) -> impl Iterator<Item = (Side, &Bound)> {
        [(Side::Start, &self.start), (Side::End, &self.end)]
            .into_iter()
            .filter_map(|(side, bound)| Some((side, bound.as_ref()?)))
    }
}

/// _Shared (pre + per + post)._ One edge of a [`Window`]: seconds after
/// `run.start`, or an instant plus an offset in seconds. An instant is the
/// run's first tick, its last, or the earliest stamp of one series (an
/// event, recorded like any other value), so a window can follow what
/// happened in the run rather than retype a time the scenario owns.
#[derive(Clone, Debug, PartialEq)]
pub enum Bound {
    /// Seconds after `run.start`.
    Secs(f64),
    /// `{ at = "run.start" | "run.end", plus = s }`.
    At { at: RunInstant, plus: f64 },
    /// `{ after = "<selector>", plus = s }`: `plus` seconds after the
    /// earliest stamp of the series `after` names.
    After { after: AssertionTarget, plus: f64 },
}

impl Bound {
    /// Seconds after `run.start`, when the bound is known before the run.
    pub fn secs_after_start(&self) -> Option<f64> {
        match self {
            Bound::Secs(secs) => Some(*secs),
            Bound::At {
                at: RunInstant::Start,
                plus,
            } => Some(*plus),
            Bound::At {
                at: RunInstant::End,
                ..
            }
            | Bound::After { .. } => None,
        }
    }

    /// Seconds after `run.start` in a run that ends at `budget`. `None` for
    /// an `after` bound, which only the run places.
    fn secs_at_budget(&self, budget: f64) -> Option<f64> {
        match self {
            Bound::At {
                at: RunInstant::End,
                plus,
            } => Some(budget + plus),
            other => other.secs_after_start(),
        }
    }

    /// Why the bound on `side` can't be read, if it can't.
    fn check(&self, side: Side) -> Result<(), WindowProblem> {
        match self {
            Bound::Secs(secs) if !secs.is_finite() => Err(WindowProblem::NonFinite(side)),
            Bound::Secs(secs) if *secs < 0.0 => Err(WindowProblem::Negative { side, secs: *secs }),
            Bound::At { plus, .. } | Bound::After { plus, .. } if !plus.is_finite() => {
                Err(WindowProblem::NonFinite(side))
            }
            Bound::At {
                at: RunInstant::Start,
                plus,
            } if *plus < 0.0 => Err(WindowProblem::BeforeRunStart { side, plus: *plus }),
            Bound::At {
                at: RunInstant::End,
                plus,
            } if *plus > 0.0 => Err(WindowProblem::AfterRunEnd { side, plus: *plus }),
            _ => Ok(()),
        }
    }
}

/// A number, or a table with exactly one of `at` and `after` and an
/// optional `plus`. By hand rather than `untagged`, so a malformed bound is
/// refused with what a bound may be instead of "did not match any variant".
impl<'de> Deserialize<'de> for Bound {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(BoundVisitor)
    }
}

impl Serialize for Bound {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            Bound::Secs(secs) => serializer.serialize_f64(*secs),
            Bound::At { at, plus } => {
                let mut map = serializer.serialize_map(Some(2))?;
                map.serialize_entry(AT_KEY, at.as_str())?;
                map.serialize_entry(PLUS_KEY, plus)?;
                map.end()
            }
            Bound::After { after, plus } => {
                let mut map = serializer.serialize_map(Some(2))?;
                map.serialize_entry(AFTER_KEY, after.as_str())?;
                map.serialize_entry(PLUS_KEY, plus)?;
                map.end()
            }
        }
    }
}

struct BoundVisitor;

impl<'de> Visitor<'de> for BoundVisitor {
    type Value = Bound;

    fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "seconds after {RUN_START}, {{ {AT_KEY} = \"{RUN_START}\" or \"{RUN_END}\", \
             {PLUS_KEY} = <seconds> }}, or {{ {AFTER_KEY} = \"<path>\", {PLUS_KEY} = <seconds> }}"
        )
    }

    fn visit_f64<E: de::Error>(self, secs: f64) -> Result<Bound, E> {
        Ok(Bound::Secs(secs))
    }

    fn visit_i64<E: de::Error>(self, secs: i64) -> Result<Bound, E> {
        Ok(Bound::Secs(secs as f64))
    }

    fn visit_u64<E: de::Error>(self, secs: u64) -> Result<Bound, E> {
        Ok(Bound::Secs(secs as f64))
    }

    fn visit_map<M: MapAccess<'de>>(self, map: M) -> Result<Bound, M::Error> {
        let table = BoundTable::deserialize(MapAccessDeserializer::new(map))?;
        match (table.at, table.after) {
            (Some(at), None) => Ok(Bound::At {
                at,
                plus: table.plus,
            }),
            (None, Some(after)) => Ok(Bound::After {
                after,
                plus: table.plus,
            }),
            _ => Err(de::Error::custom(format!(
                "a window bound's table names exactly one of `{AT_KEY}` and `{AFTER_KEY}`"
            ))),
        }
    }
}

/// A bound's table form as written, before exactly one instant is checked.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BoundTable {
    #[serde(default)]
    at: Option<RunInstant>,
    #[serde(default)]
    after: Option<AssertionTarget>,
    #[serde(default)]
    plus: f64,
}

/// _Shared (pre + per)._ An instant every run has.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(try_from = "String")]
pub enum RunInstant {
    /// The run's first tick.
    Start,
    /// The run's last tick.
    End,
}

impl RunInstant {
    pub fn as_str(&self) -> &'static str {
        match self {
            RunInstant::Start => RUN_START,
            RunInstant::End => RUN_END,
        }
    }
}

impl TryFrom<String> for RunInstant {
    type Error = String;

    fn try_from(name: String) -> Result<Self, Self::Error> {
        match name.as_str() {
            RUN_START => Ok(RunInstant::Start),
            RUN_END => Ok(RunInstant::End),
            _ => Err(format!(
                "'{name}' is not an instant: use \"{RUN_START}\" or \"{RUN_END}\""
            )),
        }
    }
}

/// _Pre-step._ Which edge of a window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Start,
    End,
}

impl fmt::Display for Side {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Side::Start => "start",
            Side::End => "end",
        })
    }
}

/// _Pre-step._ Why a window can't be read, or could never be covered.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum WindowProblem {
    #[error("the window's {0} is not a finite number")]
    NonFinite(Side),
    #[error("the window's {side} is {secs} s: a number is seconds after run.start")]
    Negative { side: Side, secs: f64 },
    #[error("the window's {side} is {plus} s from run.start, before the run")]
    BeforeRunStart { side: Side, plus: f64 },
    #[error("the window's {side} is {plus} s past run.end, after the run")]
    AfterRunEnd { side: Side, plus: f64 },
    #[error("the window ends at {end} s, past the {budget} s budget, so no run covers it")]
    PastBudget { end: f64, budget: f64 },
    /// With `run.end` read as the budget, or both bounds after one anchor.
    #[error("the window ends ({end} s) no later than it starts ({start} s)")]
    Empty { start: f64, end: f64 },
}

#[cfg(test)]
mod tests {
    use super::*;

    const BUDGET: f64 = 60.0;
    const EVENT: &str = "agent.car.node.faults.gps_outage";

    #[derive(Debug, Deserialize)]
    struct Wrap {
        window: Window,
    }

    fn parse(text: &str) -> Result<Window, toml::de::Error> {
        toml::from_str::<Wrap>(&format!("window = {text}")).map(|wrap| wrap.window)
    }

    fn window(text: &str) -> Window {
        parse(text).expect("a window parses")
    }

    fn event() -> AssertionTarget {
        AssertionTarget::parse(EVENT).expect("valid target")
    }

    #[test]
    fn each_bound_form_parses() {
        assert_eq!(
            window("{ start = 40 }"),
            Window {
                start: Some(Bound::Secs(40.0)),
                end: None,
            }
        );
        assert_eq!(
            window("{ start = { at = \"run.end\", plus = -5.0 } }").start,
            Some(Bound::At {
                at: RunInstant::End,
                plus: -5.0,
            })
        );
        assert_eq!(
            window(&format!(
                "{{ end = {{ after = \"{EVENT}\", plus = 7.0 }} }}"
            ))
            .end,
            Some(Bound::After {
                after: event(),
                plus: 7.0,
            })
        );
        assert_eq!(
            window("{ end = { at = \"run.start\" } }").end,
            Some(Bound::At {
                at: RunInstant::Start,
                plus: 0.0,
            })
        );
    }

    #[test]
    fn a_malformed_bound_is_refused_at_parse() {
        for text in [
            "{ start = \"40\" }",
            "{ start = { at = \"run.middle\" } }",
            "{ start = { plus = 1.0 } }",
            &format!("{{ start = {{ at = \"run.end\", after = \"{EVENT}\" }} }}"),
            "{ start = { at = \"run.end\", plus = 1.0, minus = 2.0 } }",
            "{ begin = 1.0 }",
            "{ start = { after = \"not a path\" } }",
        ] {
            assert!(parse(text).is_err(), "{text}");
        }
    }

    #[test]
    fn a_bad_bound_names_what_a_bound_may_be() {
        let error = parse("{ start = { plus = 1.0 } }").unwrap_err().to_string();
        assert!(error.contains("exactly one of `at` and `after`"), "{error}");
    }

    #[test]
    fn a_fixed_window_has_only_bounds_from_run_start() {
        assert!(window("{}").fixed());
        assert!(window("{ start = 5.0, end = { at = \"run.start\", plus = 9.0 } }").fixed());
        assert!(!window("{ start = { at = \"run.end\", plus = -5.0 } }").fixed());
        assert!(!window(&format!("{{ start = {{ after = \"{EVENT}\" }} }}")).fixed());
    }

    #[test]
    fn coherent_windows_pass_the_check() {
        for text in [
            "{}",
            "{ start = 40.0 }",
            "{ end = 60.0 }",
            "{ start = { at = \"run.end\", plus = -5.0 } }",
            &format!(
                "{{ start = {{ after = \"{EVENT}\", plus = 2.0 }}, \
                 end = {{ after = \"{EVENT}\", plus = 7.0 }} }}"
            ),
        ] {
            assert_eq!(window(text).check(BUDGET), Ok(()), "{text}");
        }
    }

    #[test]
    fn incoherent_windows_are_refused_with_the_reason() {
        let cases = [
            ("{ start = nan }", WindowProblem::NonFinite(Side::Start)),
            (
                "{ start = -1.0 }",
                WindowProblem::Negative {
                    side: Side::Start,
                    secs: -1.0,
                },
            ),
            (
                "{ start = { at = \"run.start\", plus = -1.0 } }",
                WindowProblem::BeforeRunStart {
                    side: Side::Start,
                    plus: -1.0,
                },
            ),
            (
                "{ start = { at = \"run.end\", plus = -70.0 } }",
                WindowProblem::BeforeRunStart {
                    side: Side::Start,
                    plus: -10.0,
                },
            ),
            (
                "{ end = { at = \"run.end\", plus = 1.0 } }",
                WindowProblem::AfterRunEnd {
                    side: Side::End,
                    plus: 1.0,
                },
            ),
            (
                "{ end = 70.0 }",
                WindowProblem::PastBudget {
                    end: 70.0,
                    budget: BUDGET,
                },
            ),
            (
                "{ end = { at = \"run.start\", plus = 70.0 } }",
                WindowProblem::PastBudget {
                    end: 70.0,
                    budget: BUDGET,
                },
            ),
            (
                "{ start = 70.0 }",
                WindowProblem::Empty {
                    start: 70.0,
                    end: BUDGET,
                },
            ),
            (
                "{ start = 10.0, end = { at = \"run.end\", plus = -55.0 } }",
                WindowProblem::Empty {
                    start: 10.0,
                    end: 5.0,
                },
            ),
        ];
        for (text, problem) in cases {
            assert_eq!(window(text).check(BUDGET), Err(problem), "{text}");
        }
    }

    #[test]
    fn offsets_from_one_anchor_must_increase() {
        let text = format!(
            "{{ start = {{ after = \"{EVENT}\", plus = 7.0 }}, \
             end = {{ after = \"{EVENT}\", plus = 2.0 }} }}"
        );
        assert_eq!(
            window(&text).check(BUDGET),
            Err(WindowProblem::Empty {
                start: 7.0,
                end: 2.0,
            })
        );
    }

    #[test]
    fn a_window_serializes_as_written() {
        let original = window(&format!(
            "{{ start = {{ after = \"{EVENT}\", plus = 2.0 }}, end = 30.0 }}"
        ));
        #[derive(Serialize)]
        struct Out<'a> {
            window: &'a Window,
        }
        let text = toml::to_string(&Out { window: &original }).expect("serializes");
        let back: Wrap = toml::from_str(&text).expect("reads back");
        assert_eq!(back.window, original);
    }
}
