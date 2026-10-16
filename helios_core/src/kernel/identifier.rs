//! The shape of a config-stable identifier: a name written by a person in a
//! config file and used as a key everywhere after (a semantic class, a world
//! placement, an agent, a node, a channel). One rule for each kind of name,
//! so a name valid in one place is valid in every other.
//!
//! Names that address something (agents, nodes, channels, frame leaves) are
//! built from [levels](is_level) joined by [`LEVEL_SEPARATOR`]. A name that
//! fills one part of a longer path is one level; a name that may group its
//! values is [dotted](is_dotted_name). The charset is narrow so every
//! spelling built from names splits back into them, and a name crosses into
//! a Zenoh key expression or an MCAP topic with no escaping.

/// The one separator between the levels of a name: `aiding.gps.nis`,
/// `agent.car.node.estimator`.
pub const LEVEL_SEPARATOR: &str = ".";

/// Whether `name` is one level: non-empty, and only ASCII letters, digits
/// and `_`.
pub fn is_level(name: &str) -> bool {
    !name.is_empty() && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// Whether `name` is one or more [levels](is_level) joined by
/// [`LEVEL_SEPARATOR`]. An empty level (`""`, `.nis`, `nis.`, `a..b`) fails.
pub fn is_dotted_name(name: &str) -> bool {
    name.split(LEVEL_SEPARATOR).all(is_level)
}

/// Whether `name` is `snake_case`: lowercase ASCII letters, digits and single
/// underscores, starting with a letter and not ending with an underscore.
///
/// Characters that separate identifiers elsewhere (`/`, `.`) and whitespace
/// all fail, so a valid name can be joined into a path without escaping.
pub fn is_snake_case(name: &str) -> bool {
    matches!(name.chars().next(), Some(c) if c.is_ascii_lowercase())
        && name
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
        && !name.ends_with('_')
        && !name.contains("__")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_level_is_letters_digits_and_underscores() {
        for name in ["car", "ProvingCar", "car_2", "imu0", "_x"] {
            assert!(is_level(name), "{name} should pass");
        }
    }

    #[test]
    fn a_level_holds_no_separator_and_is_not_empty() {
        for name in ["", "a.b", "oracle/pose", "gps-fix", "a b", "café", "*"] {
            assert!(!is_level(name), "{name} should fail");
        }
    }

    #[test]
    fn a_dotted_name_is_one_or_more_levels() {
        for name in ["estimate", "truth.tf", "sensor.gps.front_left"] {
            assert!(is_dotted_name(name), "{name} should pass");
        }
    }

    #[test]
    fn a_dotted_name_has_no_empty_or_malformed_level() {
        for name in [
            "",
            ".nis",
            "nis.",
            "aiding..nis",
            "oracle/pose",
            "gps.fix-1",
        ] {
            assert!(!is_dotted_name(name), "{name} should fail");
        }
    }

    #[test]
    fn accepts_snake_case() {
        for name in ["traffic_cone", "car", "car2", "a_1_b"] {
            assert!(is_snake_case(name), "{name} should pass");
        }
    }

    #[test]
    fn rejects_everything_else() {
        for name in [
            "",
            "Car",
            "2car",
            "_car",
            "car_",
            "traffic__cone",
            "traffic-cone",
            "crates/a",
            "yard.crate",
            "crate a",
            "café",
        ] {
            assert!(!is_snake_case(name), "{name} should fail");
        }
    }
}
