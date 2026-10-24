//! What validation's tests check: runs parsed from TOML, so they exercise
//! the real type, checked whole against the standard evaluator kinds.

use super::{validate, ValidationError};
use crate::{
    evaluation::standard_evaluators,
    run::{self, Run},
};

/// `run` checked against the standard evaluator kinds.
pub(super) fn check(run: &Run) -> Result<(), ValidationError> {
    validate(run, &standard_evaluators())
}

// Callers supply the run-specific body (termination + any `[[assert]]` or
// `[[measure]]` blocks); the header fields every run needs are filled in
// here.
pub(super) fn run_with(body: &str) -> Run {
    let toml = format!(
        r#"
        schema_version = "2.0"
        scenario = {{ from = "s" }}
        observation = {{ from = "o" }}
        {body}
        "#
    );
    run::parse(&toml, run::no_suite_files).unwrap()
}

pub(super) const BUDGET: &str = "termination = { max_simulated_seconds = 5.0 }";

pub(super) const REDUCED: &str = r#"
    [[assert]]
    name = "reduced"
    target = "agent.car.node.estimator.x"
    reduce = "final"
    condition = { less_than = 1.0 }
"#;

pub(super) const EVERY_SAMPLE: &str = r#"
    [[assert]]
    name = "every_sample"
    target = "agent.car.node.estimator.x"
    condition = { less_than = 1.0 }
"#;

/// One assertion named `name` on `condition`, under a plain budget.
pub(super) fn asserting(name: &str, reduce: &str, condition: &str) -> Run {
    run_with(&format!(
        "{BUDGET}\n[[assert]]\nname = \"{name}\"\n\
         target = \"agent.car.node.estimator.x\"\n{reduce}\ncondition = {condition}"
    ))
}

/// A run with one `twist_tracking` evaluation named "twist", its body
/// edited by `edit`, and `rest` (more blocks).
pub(super) fn evaluating(edit: impl Fn(String) -> String, rest: &str) -> Run {
    let twist = edit(
        "[[evaluate]]\nname = \"twist\"\nkind = \"twist_tracking\"\n\
         [evaluate.inputs]\n\
         truth = { path = \"agent.car.channel.truth.twist\", instance = \"agent.car.base_link\" }\n\
         reference = \"agent.car.channel.reference\"\n"
            .to_owned(),
    );
    run_with(&format!("{BUDGET}\n{rest}\n{twist}"))
}

/// An assertion named `name` on `target`, reduced by its max.
pub(super) fn max_of(name: &str, target: &str) -> String {
    format!(
        "[[assert]]\nname = \"{name}\"\ntarget = \"{target}\"\nreduce = \"max\"\n\
         condition = {{ less_than = 1.0 }}"
    )
}

/// A run under a plain budget, with `body` and each of `suites` (its
/// application's name, the agents it is applied to, and the suite file's
/// text) applied.
pub(super) fn applying(body: &str, suites: &[(&str, &str, &str)]) -> Run {
    let mut text = format!(
        "schema_version = \"2.0\"\nscenario = {{ from = \"s\" }}\n\
         observation = {{ from = \"o\" }}\n{BUDGET}\n{body}\n"
    );
    for (name, agents, _) in suites {
        text.push_str(&format!(
            "\n[[suite]]\nname = \"{name}\"\nfrom = \"{name}.toml\"\nagents = {agents}\n"
        ));
    }
    run::parse(&text, |from| {
        suites
            .iter()
            .find(|(name, _, _)| format!("{name}.toml") == from)
            .map(|(_, _, suite)| suite.to_string())
            .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::NotFound, from.to_owned()))
    })
    .expect("a valid run and suites")
}

/// A suite of one `twist_tracking` entry named "twist", over the car's
/// reference and its true twist.
pub(super) const TWIST_SUITE: &str = "[[evaluate]]\nname = \"twist\"\nkind = \"twist_tracking\"\n\
     [evaluate.inputs]\ntruth = \"truth.twist\"\nreference = \"reference\"\n";
