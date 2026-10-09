//! How names are spelled inside a watcher path.
//!
//! Watchers address everything under a node by a dotted path,
//! `agent.<agent>.<node>.<rest>`. The agent and the node each fill exactly
//! one part, so their names must be [path segments](is_path_segment). A
//! node's observations fill `<rest>` with the leaf the node declared; a
//! node's output channels fill it with the segment built here. Both share one
//! namespace, so the build compares leaves against these segments, and
//! assertion targets use the same rule to find a channel.

use crate::port::ChannelKey;

/// The one level separator in watcher paths.
pub(crate) const PATH_SEPARATOR: &str = ".";

/// Whether `name` can fill exactly one part of a watcher path: it is not
/// empty and holds no [`PATH_SEPARATOR`]. Agent and node names must be; an
/// empty name or a dotted one would make the path impossible to split back
/// into its parts.
pub(crate) fn is_path_segment(name: &str) -> bool {
    !name.is_empty() && !name.contains(PATH_SEPARATOR)
}

/// The path segment for `key`: its instance, else its type's name.
///
/// The instance is preferred because it is the name a person wrote in the
/// config. Each `/` in it becomes `.`, so `oracle/pose` sits at
/// `oracle.pose`: `.` is the one level separator in paths, and `_` stays a
/// word joiner. An unnamed channel falls back to the last part of its type
/// name in snake_case (`FrameAwareState` → `frame_aware_state`).
pub fn channel_to_path_segment(key: &ChannelKey) -> String {
    let raw = key.instance();
    if !raw.trim().is_empty() {
        raw.replace('/', PATH_SEPARATOR)
    } else {
        to_snake_case(last_segment(key.type_name()))
    }
}

/// The part of a type name after its last `::`.
fn last_segment(type_name: &'static str) -> &'static str {
    type_name.rsplit("::").next().unwrap_or(type_name)
}

/// Lowercase, with `_` before each uppercase letter after the first:
/// `FrameAwareState` → `frame_aware_state`, `f64` → `f64`. Enough for Rust
/// type names; not a general snake_case (no digit boundaries, no acronym
/// handling).
fn to_snake_case(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 4);
    for (i, ch) in s.chars().enumerate() {
        if i > 0 && ch.is_uppercase() {
            out.push('_');
        }
        out.extend(ch.to_lowercase());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::InternalChannel;

    fn channel_named<T: 'static>(instance: &'static str) -> ChannelKey {
        InternalChannel::named::<T>(instance).into()
    }

    fn channel_unnamed<T: 'static>() -> ChannelKey {
        InternalChannel::of::<T>().into()
    }

    #[test]
    fn plain_name_is_a_path_segment() {
        assert!(is_path_segment("primary"));
        assert!(is_path_segment("front_lidar_deproject"));
    }

    #[test]
    fn dotted_or_empty_name_is_not_a_path_segment() {
        assert!(!is_path_segment("a.b"));
        assert!(!is_path_segment(".a"));
        assert!(!is_path_segment(""));
    }

    #[test]
    fn channel_to_path_segment_uses_instance() {
        let key = channel_named::<f64>("oracle/pose");
        assert_eq!(channel_to_path_segment(&key), "oracle.pose");
    }

    #[test]
    fn channel_to_path_segment_replaces_all_slashes() {
        let key = channel_named::<f64>("debug/inner/value");
        assert_eq!(channel_to_path_segment(&key), "debug.inner.value");
    }

    #[test]
    fn channel_to_path_segment_keeps_dots_and_underscores() {
        let key = channel_named::<f64>("sensor.gps.front_left");
        assert_eq!(channel_to_path_segment(&key), "sensor.gps.front_left");
    }

    #[test]
    fn channel_to_path_segment_falls_back_to_type_name() {
        let key = channel_unnamed::<f64>();
        assert_eq!(channel_to_path_segment(&key), "f64");
    }

    #[test]
    fn last_segment_strips_module_path() {
        assert_eq!(last_segment("foo::bar::Baz"), "Baz");
        assert_eq!(last_segment("Baz"), "Baz");
        assert_eq!(last_segment("f64"), "f64");
    }

    #[test]
    fn to_snake_case_basic_cases() {
        assert_eq!(to_snake_case("FrameAwareState"), "frame_aware_state");
        assert_eq!(to_snake_case("f64"), "f64");
        assert_eq!(to_snake_case("A"), "a");
        assert_eq!(to_snake_case(""), "");
        assert_eq!(to_snake_case("MyType"), "my_type");
    }
}
