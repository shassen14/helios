//! The `[estimate]` seam's pass.

use super::super::members::{member_output, SeamType};
use super::config::EstimateSeamConfig;

use crate::assembly::error::PipelineAssemblyError;
use crate::nodes::estimate_relay::EstimateRelay;
use crate::pipeline::node::PipelineNode;

use helios_core::prelude::AgentId;
use helios_core::spatial::FrameAwareState;

/// Node name for the relay this seam adds. Raw identity for observability,
/// so a referenced const; this pass is its sole synthesizer.
pub(crate) const ESTIMATE_RELAY_NODE: &str = "estimate_relay";

/// The seam's name in errors: its stack section.
const SEAM: &str = "estimate";

/// Builds `agent`'s estimate relay from `config`, resolving the source among
/// `nodes`. No section means no seam: `None`, and anything reading the
/// estimate fails the build as an unsatisfied input.
///
/// Fails if the source is unknown or lacks exactly one state output.
pub(in crate::assembly) fn estimate_relay(
    config: Option<&EstimateSeamConfig>,
    nodes: &[Box<dyn PipelineNode>],
    agent: &AgentId,
) -> Result<Option<Box<dyn PipelineNode>>, Vec<PipelineAssemblyError>> {
    let Some(config) = config else {
        return Ok(None);
    };

    let source = member_output(
        SEAM,
        SeamType::of::<FrameAwareState>(),
        &config.source,
        nodes,
    )
    .map_err(|err| vec![err])?;
    Ok(Some(Box::new(EstimateRelay::new(
        ESTIMATE_RELAY_NODE,
        source,
        agent,
    ))))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::test_stub::stub;
    use crate::channels::estimate::{estimate, estimator_output};
    use crate::channels::tf::is_tf_edge;
    use crate::port::ChannelKey;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn section(source: &str) -> EstimateSeamConfig {
        EstimateSeamConfig {
            source: source.to_string(),
        }
    }

    fn estimators() -> Vec<Box<dyn PipelineNode>> {
        vec![
            stub("primary", vec![estimator_output("primary")]),
            stub("shadow", vec![estimator_output("shadow")]),
        ]
    }

    #[test]
    fn no_section_adds_no_relay() {
        let relay = estimate_relay(None, &estimators(), &agent()).expect("no section is fine");
        assert!(relay.is_none());
    }

    #[test]
    fn the_named_source_is_relayed_onto_the_estimate_with_the_edge() {
        let relay = estimate_relay(Some(&section("shadow")), &estimators(), &agent())
            .expect("a known source resolves")
            .expect("a section adds a relay");

        assert_eq!(relay.name(), ESTIMATE_RELAY_NODE);
        let descriptor = relay.port_descriptor();
        let required: Vec<&ChannelKey> = descriptor.required_inputs().collect();
        assert_eq!(
            required,
            vec![&ChannelKey::from(estimator_output("shadow"))]
        );
        let outputs = descriptor.outputs();
        assert!(outputs.contains(&estimate().into()));
        assert_eq!(outputs.iter().filter(|key| is_tf_edge(key)).count(), 1);
    }

    #[test]
    fn an_unknown_source_is_refused() {
        let errors = estimate_relay(Some(&section("primray")), &estimators(), &agent())
            .err()
            .expect("an unknown source fails");

        assert!(matches!(
            errors.as_slice(),
            [PipelineAssemblyError::UnknownSeamMember { seam, member }]
                if seam == SEAM && member == "primray"
        ));
    }

    #[test]
    fn a_source_without_a_state_output_is_refused() {
        let nodes = vec![stub("primary", vec![])];
        let errors = estimate_relay(Some(&section("primary")), &nodes, &agent())
            .err()
            .expect("a source writing no state fails");

        assert!(matches!(
            errors.as_slice(),
            [PipelineAssemblyError::SeamMemberOutputMismatch { matching: 0, .. }]
        ));
    }
}
