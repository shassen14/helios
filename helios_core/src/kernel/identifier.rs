//! The shape of a config-stable identifier: a name written by a person in a
//! config file and used as a key everywhere after (a semantic class, a world
//! placement). One rule for all of them, so a name valid in one place is
//! valid in every other.

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
