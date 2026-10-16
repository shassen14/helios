//! The `[estimate]` seam's pass.

use super::super::members::{member_output, SeamType};
use super::config::EstimateSeamConfig;

use crate::assembly::error::PipelineAssemblyError;
use crate::nodes::estimate_relay::EstimateRelay;
use crate::pipeline::NodeDeclaration;

use helios_core::estimation::frames::EstimationFrames;
use helios_core::kernel::identifier::LEVEL_SEPARATOR;
use helios_core::prelude::AgentId;
use helios_core::spatial::id::ODOM;
use helios_core::spatial::{FrameAwareState, FrameId};

/// Node name for the relay this seam adds. Raw identity for observability,
/// so a referenced const; this pass is its sole synthesizer.
pub(crate) const ESTIMATE_RELAY_NODE: &str = "estimate_relay";

/// The seam's name in errors: its stack section.
const SEAM: &str = "estimate";

/// The root node `node` of `agent`'s stack holds any estimate it makes in:
/// the agent's `odom` for the estimate's `source`, `<node>.odom` for any
/// other node.
///
/// Node names are unique and one level, so a shadow estimator's root can't be
/// another estimator's; a body frame named like one is refused at build as a
/// frame with two owners. Switching the source swaps the roots: the
/// authoritative estimate is the one held in `odom`.
pub(in crate::assembly) fn estimator_root(
    agent: &AgentId,
    node: &str,
    source: Option<&str>,
) -> FrameId {
    if source == Some(node) {
        FrameId::odom(agent.clone())
    } else {
        FrameId::agent(agent.clone(), format!("{node}{LEVEL_SEPARATOR}{ODOM}"))
    }
}

/// Builds `agent`'s estimate relay from `config`, resolving the source among
/// `nodes`; its edge runs from the source's root to `base_link`. No section
/// means no seam: `None`, and anything reading the estimate fails the build
/// as an unsatisfied input.
///
/// Fails if the source is unknown or lacks exactly one state output.
pub(in crate::assembly) fn estimate_relay(
    config: Option<&EstimateSeamConfig>,
    nodes: &[NodeDeclaration],
    agent: &AgentId,
) -> Result<Option<EstimateRelay>, Vec<PipelineAssemblyError>> {
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
    let frames = EstimationFrames {
        body: FrameId::base_link(agent.clone()),
        root: estimator_root(agent, &config.source, Some(&config.source)),
    };
    Ok(Some(EstimateRelay::new(
        ESTIMATE_RELAY_NODE,
        source,
        frames,
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::test_stub::stub;
    use crate::channels::estimate::{estimate, estimator_output};
    use crate::channels::tf::{is_tf_edge, tf_edge};
    use crate::pipeline::node::PipelineNode;
    use crate::port::ChannelKey;

    use helios_core::spatial::transforms::tf::stamped::FrameEdge;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn section(source: &str) -> EstimateSeamConfig {
        EstimateSeamConfig {
            source: source.to_string(),
        }
    }

    fn estimators() -> Vec<NodeDeclaration> {
        vec![
            stub("primary", vec![estimator_output("primary")]).into(),
            stub("shadow", vec![estimator_output("shadow")]).into(),
        ]
    }

    #[test]
    fn the_source_holds_its_estimate_in_odom_and_every_other_node_in_its_own_root() {
        assert_eq!(
            estimator_root(&agent(), "primary", Some("primary")),
            FrameId::odom(agent())
        );
        assert_eq!(
            estimator_root(&agent(), "shadow", Some("primary")),
            FrameId::agent(agent(), "shadow.odom")
        );
        assert_eq!(
            estimator_root(&agent(), "primary", None),
            FrameId::agent(agent(), "primary.odom")
        );
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
        assert_eq!(required, vec![&estimator_output("shadow")]);
        let outputs: Vec<ChannelKey> = descriptor.outputs().cloned().collect();
        assert!(outputs.contains(&estimate()));
        assert_eq!(outputs.iter().filter(|key| is_tf_edge(key)).count(), 1);
        // The source's root is odom, so the relay's edge runs from odom.
        let edge = FrameEdge {
            child: FrameId::base_link(agent()),
            parent: FrameId::odom(agent()),
        };
        assert!(outputs.contains(&tf_edge(&edge)));
        assert_eq!(relay.root(), &FrameId::odom(agent()));
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
        let nodes = vec![stub("primary", vec![]).into()];
        let errors = estimate_relay(Some(&section("primary")), &nodes, &agent())
            .err()
            .expect("a source writing no state fails");

        assert!(matches!(
            errors.as_slice(),
            [PipelineAssemblyError::SeamMemberOutputMismatch { matching: 0, .. }]
        ));
    }
}
