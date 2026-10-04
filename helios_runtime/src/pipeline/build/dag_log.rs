//! The one-shot startup log of a built pipeline: the body, the outside inputs,
//! and every node by level.

use crate::{
    pipeline::key_format::format_key_short, BodyCapabilities, ChannelKey, NodeId, PipelineNode,
};

use tracing::info;

/// One-shot startup dump of the resolved DAG. Emits at `info` so it's
/// visible with the default `helios=info` filter; nothing else in the
/// per-tick path emits at that level, so this stays a single block.
pub(super) fn log_resolved_dag(
    levels: &[Vec<(NodeId, Box<dyn PipelineNode>)>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
) {
    info!(
        target: "helios_runtime::pipeline",
        levels = levels.len(),
        nodes = levels.iter().map(|level| level.len()).sum::<usize>(),
        "resolved autonomy pipeline",
    );

    // Body line: which body the graph runs against and what it offers. Indented
    // two spaces to nest under the pipeline header, matching the node lines below.
    info!(
        target: "helios_runtime::pipeline",
        name = capabilities.name,
        consumes_control = capabilities.consumes_control,
        published = capabilities.publishes.len(),
        "  body"
    );

    // One line per channel the body publishes, nested another level under body.
    for pc in &capabilities.publishes {
        info!(
            target: "helios_runtime::pipeline",
            channel = %format_key_short(&pc.key),
            provenance = ?pc.provenance,
            "    publishes"
        )
    }

    // Outside inputs: what an operator or mission system sends, kept apart from
    // the body's channels. Same nesting as the body section.
    info!(
        target: "helios_runtime::pipeline",
        declared = outside_inputs.len(),
        "  outside inputs"
    );

    for key in outside_inputs {
        info!(
            target: "helios_runtime::pipeline",
            channel = %format_key_short(key),
            "    input"
        )
    }

    for (level_idx, level) in levels.iter().enumerate() {
        for (node_id, node) in level {
            let descriptor = node.port_descriptor();
            let rate = match descriptor.rate() {
                Some(hz) => format!("{hz} Hz"),
                None => "every tick".to_string(),
            };
            let inputs = format_keys(descriptor.required_inputs());
            let optional = format_keys(descriptor.optional_inputs());
            let outputs = format_keys(descriptor.outputs());
            info!(
                target: "helios_runtime::pipeline",
                level = level_idx,
                id = *node_id,
                name = node.name(),
                rate = %rate,
                inputs = %inputs,
                optional_inputs = %optional,
                outputs = %outputs,
                "  node",
            );
        }
    }
}

fn format_keys<'a>(keys: impl IntoIterator<Item = &'a ChannelKey>) -> String {
    let joined = keys
        .into_iter()
        .map(format_key_short)
        .collect::<Vec<_>>()
        .join(", ");
    format!("[{joined}]")
}
