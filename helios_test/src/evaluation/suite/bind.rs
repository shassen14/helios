//! Binding a suite's entry to an agent: the inline entry it stands for on
//! that agent, each role's series turned into the selector of its channel.

use crate::{
    assertion::AssertionTarget,
    evaluation::{kind::EvaluatorKind, AgentSeries, Evaluation},
};

use helios_core::prelude::AgentId;

/// The inline entry `entry` stands for on `agent`, read by `kind`, or every
/// reason a role's series can't be read by it: a series carrying another
/// type than the role reads, or a role the kind doesn't have. Whether the
/// agent writes the series is setup's question, asked of its catalog, as for
/// an inline entry's selectors.
pub fn bind(
    entry: &Evaluation<AgentSeries>,
    agent: &AgentId,
    kind: &dyn EvaluatorKind,
) -> Result<Evaluation, Vec<String>> {
    let roles = kind.roles();
    let mut inputs = Vec::new();
    let mut reasons = Vec::new();
    for (role, &series) in &entry.inputs {
        let key = match roles.iter().position(|declared| declared.name == role) {
            Some(index) => kind.series_key(index, series),
            None => Err(format!("{} has no role `{role}`", kind.name())),
        };
        match key {
            Ok(key) => inputs.push((
                role.clone(),
                AssertionTarget::channel(agent, &key, series.instance(agent).as_ref()),
            )),
            Err(reason) => reasons.push(reason),
        }
    }
    if !reasons.is_empty() {
        return Err(reasons);
    }

    Ok(Evaluation {
        name: entry.name.clone(),
        kind: entry.kind.clone(),
        inputs: inputs.into_iter().collect(),
        params: entry.params.clone(),
        gap: entry.gap.clone(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::evaluation::standard_evaluators;

    use serde::Deserialize;

    #[derive(Deserialize)]
    struct Wrap<I> {
        evaluate: Evaluation<I>,
    }

    fn read<I: for<'de> Deserialize<'de>>(text: &str) -> Evaluation<I> {
        toml::from_str::<Wrap<I>>(text)
            .expect("a valid entry")
            .evaluate
    }

    fn bound(suite_entry: &str, agent: &str) -> Result<Evaluation, Vec<String>> {
        let entry = read::<AgentSeries>(suite_entry);
        let registry = standard_evaluators();
        let kind = registry.get(&entry.kind).expect("a standard kind");
        bind(&entry, &AgentId::new(agent), kind)
    }

    #[test]
    fn a_bound_suite_entry_equals_the_inline_entry_it_stands_for() {
        let suite_entry = r#"
            [evaluate]
            name = "est"
            kind = "pose_error"
            [evaluate.inputs]
            estimate = "estimate.pose"
            truth = "truth.pose"
            [evaluate.params]
            align = "declared"
            [evaluate.gap.truth]
            multiple = 2.0
            "#;
        let inline = read::<AssertionTarget>(
            r#"
            [evaluate]
            name = "est"
            kind = "pose_error"
            [evaluate.inputs]
            estimate = { path = "agent.car.channel.tf", instance = "agent.car.base_link" }
            truth = { path = "agent.car.channel.truth.tf", instance = "agent.car.base_link" }
            [evaluate.params]
            align = "declared"
            [evaluate.gap.truth]
            multiple = 2.0
            "#,
        );

        assert_eq!(bound(suite_entry, "car"), Ok(inline));
    }

    #[test]
    fn a_series_with_no_instance_binds_to_its_channel_alone() {
        let twist = bound(
            r#"
            [evaluate]
            name = "tracking"
            kind = "twist_tracking"
            [evaluate.inputs]
            reference = "reference"
            truth = "truth.twist"
            "#,
            "car",
        )
        .expect("both series fit");

        let reference = &twist.inputs["reference"];
        assert_eq!(reference.path(), "agent.car.channel.reference");
        assert_eq!(reference.instance(), None);
    }

    #[test]
    fn every_role_whose_series_doesnt_fit_is_refused() {
        let reasons = bound(
            r#"
            [evaluate]
            name = "tracking"
            kind = "twist_tracking"
            [evaluate.inputs]
            reference = "estimate.state"
            truth = "truth.pose"
            "#,
            "car",
        )
        .expect_err("neither series carries its role's type");

        assert_eq!(reasons.len(), 2, "{reasons:?}");
        assert!(reasons[0].contains("`estimate.state`"), "{reasons:?}");
        assert!(reasons[1].contains("`truth.pose`"), "{reasons:?}");
    }

    #[test]
    fn a_role_the_kind_doesnt_have_is_refused_naming_it() {
        let reasons = bound(
            r#"
            [evaluate]
            name = "speed"
            kind = "twist"
            [evaluate.inputs]
            twist = "truth.twist"
            other = "truth.twist"
            "#,
            "car",
        )
        .expect_err("twist has one role");

        assert_eq!(reasons, ["twist has no role `other`"]);
    }
}
