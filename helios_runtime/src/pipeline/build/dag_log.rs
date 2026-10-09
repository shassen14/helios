//! The one-shot startup log of a built pipeline: the body, the outside inputs,
//! and every node by level, with what each node can emit for watchers.
//!
//! Written for a person reading a console. The whole dump is one log event
//! with a multi-line message, so the subscriber's prefix (time, level,
//! spans, target) appears once and the block below it is plain text: each
//! node a header line with its inputs, outputs and observables indented
//! under it, empty lists left out, and channels in their compact form.

use crate::{
    pipeline::{autonomy_pipeline::observable_catalog, key_format::format_key_compact},
    port::{Determinism, Observable},
    BodyCapabilities, ChannelKey, NodeId, PipelineNode,
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
        "{}",
        render_dag(levels, capabilities, outside_inputs)
    );
}

/// The dump's text: a summary line, the body and outside inputs, then each
/// node, with a blank line before every section so they read as blocks.
fn render_dag(
    levels: &[Vec<(NodeId, Box<dyn PipelineNode>)>],
    capabilities: &BodyCapabilities,
    outside_inputs: &[ChannelKey],
) -> String {
    let node_count = levels.iter().map(|level| level.len()).sum::<usize>();
    let mut out = format!(
        "resolved autonomy pipeline: {node_count} nodes in {} levels\n",
        levels.len()
    );

    out.push_str(&format!(
        "\nbody {} publishes {}\n",
        capabilities.name,
        capabilities.publishes.len()
    ));
    for pc in &capabilities.publishes {
        out.push_str(&format!(
            "  {} [{:?}]\n",
            format_key_compact(&pc.key),
            pc.provenance
        ));
    }

    // What an operator or mission system sends, kept apart from the body.
    out.push_str(&format!("\noutside inputs {}\n", outside_inputs.len()));
    for key in outside_inputs {
        out.push_str(&format!("  {}\n", format_key_compact(key)));
    }

    for (level_idx, level) in levels.iter().enumerate() {
        for (node_id, node) in level {
            render_node(&mut out, level_idx, *node_id, node.as_ref());
        }
    }
    out
}

/// A node's header line, then one line per non-empty section.
fn render_node(out: &mut String, level: usize, id: NodeId, node: &dyn PipelineNode) {
    let descriptor = node.port_descriptor();
    let rate = match descriptor.rate() {
        Some(hz) => format!("{hz} Hz"),
        None => "every tick".to_string(),
    };
    out.push_str(&format!(
        "\nnode {id} {} (level {level}, {rate})\n",
        node.name()
    ));

    let sections = [
        ("in", format_keys(descriptor.required_inputs())),
        ("optional", format_keys(descriptor.optional_inputs())),
        ("out", format_keys(descriptor.outputs())),
        ("obs", format_observables(&observable_catalog(node))),
    ];
    for (label, items) in sections {
        render_section(out, label, &items);
    }
}

/// One item per line: the label on the first, the rest aligned under it.
/// Nothing when there are no items.
fn render_section(out: &mut String, label: &str, items: &[String]) {
    for (i, item) in items.iter().enumerate() {
        let label = if i == 0 { label } else { "" };
        out.push_str(&format!("  {label:<8} {item}\n"));
    }
}

/// Each key in its compact form.
fn format_keys<'a>(keys: impl IntoIterator<Item = &'a ChannelKey>) -> Vec<String> {
    keys.into_iter().map(format_key_compact).collect()
}

/// Each leaf, a wall-clock leaf marked so a reader knows not to expect it to
/// repeat across runs.
fn format_observables(observables: &[Observable]) -> Vec<String> {
    observables
        .iter()
        .map(|observable| match observable.determinism() {
            Determinism::Reproducible => observable.leaf_name().to_string(),
            Determinism::WallClock => format!("{} (wall clock)", observable.leaf_name()),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::InternalChannel;

    #[test]
    fn section_puts_one_item_per_line_aligned_under_the_label() {
        let mut out = String::new();
        render_section(&mut out, "in", &["drive: f64".into(), "steer: f64".into()]);
        assert_eq!(out, "  in       drive: f64\n           steer: f64\n");
    }

    #[test]
    fn empty_section_writes_nothing() {
        let mut out = String::new();
        render_section(&mut out, "optional", &[]);
        assert_eq!(out, "");
    }

    #[test]
    fn keys_are_compact() {
        let keys: Vec<ChannelKey> = vec![
            InternalChannel::named::<f64>("drive").into(),
            InternalChannel::of::<f64>().into(),
        ];
        assert_eq!(format_keys(&keys), ["drive: f64", "f64"]);
    }

    #[test]
    fn only_wall_clock_leaves_are_marked() {
        let observables = [
            Observable::new("aiding.gps.nis", Determinism::Reproducible),
            Observable::new("tick.duration", Determinism::WallClock),
        ];
        assert_eq!(
            format_observables(&observables),
            ["aiding.gps.nis", "tick.duration (wall clock)"]
        );
    }
}
