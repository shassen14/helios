//! Phase-0 debug helpers for the runtime.
//!
//! Cheap, on-demand introspection — not the full observability hook
//!
//! Usage is on-demand: call from a one-shot debug Bevy system, an error
//! path, or a test. Never wire `inspect_bus` into a per-tick system — at
//! the default 200 Hz host rate the slot walk is wasted work.

use crate::pipeline::key_format::format_key_short;
use crate::port::{ChannelKey, PortBus};

/// One row of a [`PortBus`] snapshot.
///
/// `has_value` is `true` once any writer has populated the slot; it never
/// flips back to `false` because slot semantics are last-known-good.
#[derive(Debug, Clone)]
pub struct SlotSummary {
    pub channel: ChannelKey,
    pub has_value: bool,
}

/// Snapshot every slot on a bus. Order is unspecified.
pub fn inspect_bus(bus: &PortBus) -> Vec<SlotSummary> {
    let mut rows: Vec<SlotSummary> = bus
        .slot_presence()
        .into_iter()
        .map(|(channel, has_value)| SlotSummary { channel, has_value })
        .collect();

    rows.sort_by(|a, b| {
        a.channel
            .type_name
            .cmp(b.channel.type_name)
            .then_with(|| a.channel.instance.as_ref().cmp(b.channel.instance.as_ref()))
    });

    rows
}

/// Render an [`inspect_bus`] snapshot as a multi-line string. Intended for
/// one-shot dumps to stderr or a tracing event.
pub fn format_bus(bus: &PortBus) -> String {
    let rows = inspect_bus(bus);
    let mut out = String::with_capacity(rows.len() * 96);
    out.push_str("PortBus snapshot:\n");
    for row in &rows {
        let marker = if row.has_value { "set" } else { "---" };
        out.push_str(&format!("  [{marker}] {}\n", format_key_short(&row.channel)));
    }
    out
}
