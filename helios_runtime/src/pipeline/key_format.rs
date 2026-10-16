//! Short-form rendering for [`ChannelKey`] type names.
//!
//! Collapses each `::`-qualified path inside a `ChannelKey::type_name` to its
//! leaf — `helios_core::interchange::measurement::envelope::SensorReading` → `SensorReading` —
//! while leaving generic punctuation (`<`, `>`, `,`, `&`, parens) intact.
//! Used by the DAG dump and the bus snapshot for readability; production
//! code keeps full paths via [`ChannelKey`]'s `Display`.

use crate::port::{ChannelKey, ChannelKind};

/// `[kind] Type @ "name"`, every part always shown. The bus snapshot
/// uses it, where rows of every kind sit side by side.
pub(crate) fn format_key_short(key: &ChannelKey) -> String {
    let out = format!(
        "[{}] {}",
        key.kind().as_str(),
        short_type_name(key.type_name())
    );
    let name = key.name();
    if name.trim().is_empty() {
        out
    } else {
        format!("{out} @ \"{name}\"")
    }
}

/// `name: Type`, or `Type` for an unnamed channel, with `(kind)` after it
/// only when the kind is not internal. Inside the DAG almost every channel is
/// internal, so the tag is shown only where it says something.
pub(crate) fn format_key_compact(key: &ChannelKey) -> String {
    let type_name = short_type_name(key.type_name());
    let name = key.name();
    let named = if name.trim().is_empty() {
        type_name
    } else {
        format!("{name}: {type_name}")
    };
    match key.kind() {
        ChannelKind::Internal => named,
        kind => format!("{named} ({})", kind.as_str()),
    }
}

/// `type_name` with each `::`-qualified path cut to its last part.
fn short_type_name(type_name: &str) -> String {
    let mut out = String::with_capacity(type_name.len());
    let mut segment_start = 0usize;
    let bytes = type_name.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i] as char;
        if matches!(c, '<' | '>' | ',' | ' ' | '&' | '(' | ')') {
            push_leaf(&mut out, &type_name[segment_start..i]);
            out.push(c);
            i += 1;
            segment_start = i;
        } else {
            i += 1;
        }
    }
    push_leaf(&mut out, &type_name[segment_start..]);
    out
}

fn push_leaf(out: &mut String, segment: &str) {
    let leaf = segment.rsplit("::").next().unwrap_or(segment);
    out.push_str(leaf);
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    mod deep {
        pub struct Reading<T>(pub T);
    }

    #[test]
    fn short_keeps_kind_type_and_name() {
        let key = ChannelKey::internal::<f64>("drive");
        assert_eq!(format_key_short(&key), "[internal] f64 @ \"drive\"");
    }

    #[test]
    fn compact_named_internal_is_name_and_type() {
        let key = ChannelKey::internal::<f64>("drive");
        assert_eq!(format_key_compact(&key), "drive: f64");
    }

    #[test]
    fn compact_unnamed_internal_is_type_only() {
        let key = ChannelKey::internal_of::<f64>();
        assert_eq!(format_key_compact(&key), "f64");
    }

    #[test]
    fn compact_tags_kinds_other_than_internal() {
        let key = ChannelKey::measurement::<f64>("gps");
        assert_eq!(format_key_compact(&key), "gps: f64 (measurement)");
    }

    #[test]
    fn compact_shortens_paths_inside_generics() {
        let key = ChannelKey::internal_of::<deep::Reading<std::sync::Arc<f64>>>();
        assert_eq!(format_key_compact(&key), "Reading<Arc<f64>>");
    }
}
