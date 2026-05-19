//! Short-form rendering for [`ChannelKey`] type names.
//!
//! Collapses each `::`-qualified path inside a `ChannelKey::type_name` to its
//! leaf — `helios_core::data::sensor::SensorReading` → `SensorReading` —
//! while leaving generic punctuation (`<`, `>`, `,`, `&`, parens) intact.
//! Used by the DAG dump and the bus snapshot for readability; production
//! code keeps full paths via [`ChannelKey`]'s `Display`.

use crate::port::ChannelKey;

pub(crate) fn format_key_short(key: &ChannelKey) -> String {
    let mut out = String::with_capacity(key.type_name.len());
    let mut segment_start = 0usize;
    let bytes = key.type_name.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i] as char;
        if matches!(c, '<' | '>' | ',' | ' ' | '&' | '(' | ')') {
            push_leaf(&mut out, &key.type_name[segment_start..i]);
            out.push(c);
            i += 1;
            segment_start = i;
        } else {
            i += 1;
        }
    }
    push_leaf(&mut out, &key.type_name[segment_start..]);

    if key.instance.trim().is_empty() {
        out
    } else {
        format!("{out} @ \"{}\"", key.instance)
    }
}

fn push_leaf(out: &mut String, segment: &str) {
    let leaf = segment.rsplit("::").next().unwrap_or(segment);
    out.push_str(leaf);
}
