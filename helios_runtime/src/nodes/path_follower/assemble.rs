//! Config-side assembly for the path-follower family: resolves which planner
//! output channel the follower should read its `Path` from.

use crate::config::AutonomyStack;
use crate::port::InternalChannel;
use crate::PipelineAssemblyError;

use helios_core::planning::types::Path;

/// Determines the path channel the `PathFollowerNode` should read from.
///
/// Uses `PathFollowingConfig::path_source` if set; otherwise, falls back to
/// the single planner's key. Errors if multiple planners exist and no
/// explicit source is given.
pub(crate) fn resolve_path_channel(
    stack: &AutonomyStack,
) -> Result<InternalChannel, PipelineAssemblyError> {
    // Use an explicit path_source if configured.
    // TODO: add `path_source: Option<String>` to PathFollowingConfig to
    // support multi-planner stacks without ambiguity.
    // For now: auto-select the single planner, error if there are multiple.
    match stack.search_planners.len() {
        0 => Err(PipelineAssemblyError::NoPathSourceForFollower),
        1 => {
            let planner_name = stack.search_planners.keys().next().unwrap();
            Ok(InternalChannel::named::<Path>(planner_name.as_str()))
        }
        _ => {
            // Multiple planners: cannot auto-select. Caller should add
            // `path_source` to PathFollowingConfig (tracked as follow-up).
            Err(PipelineAssemblyError::NoPathSourceForFollower)
        }
    }
}
