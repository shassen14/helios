//! Preprocessing assembly: turns each `[preprocessing.<name>]` entry into the
//! node that implements its kind.
//!
//! Dispatch is an exhaustive match rather than a registry lookup. These nodes
//! are runtime-native plumbing with no swappable core algorithm behind them, and
//! the config enum is closed, so a new kind fails to compile here until its arm
//! is written instead of failing a lookup at build time.

use crate::config::PreprocessingConfig;
use crate::nodes::deproject::DeprojectNode;
use crate::pipeline::node::PipelineNode;

use helios_core::spatial::conventions::Flu;

/// Builds the node for one preprocessing entry, named by its config-map key.
pub(super) fn build_preprocessing_node(
    name: &str,
    config: &PreprocessingConfig,
) -> Box<dyn PipelineNode> {
    match config {
        PreprocessingConfig::Deproject { input, output } => {
            Box::new(DeprojectNode::<Flu, ()>::new(name, input, output))
        }
    }
}
