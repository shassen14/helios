//! [`KeyName`]: the name of a joint, a joint group, or another named
//! trajectory key.

use crate::kernel::identifier::is_level;

use std::error::Error;
use std::fmt::{self, Display};
use std::sync::Arc;

/// A name inside a trajectory key: one level of ASCII letters, digits and
/// `_`, checked once when it is made. Wraps an `Arc<str>` so a key is cheap
/// to clone.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct KeyName(Arc<str>);

impl KeyName {
    /// The name `name`, refused when it is not one level.
    pub fn new(name: &str) -> Result<Self, MalformedKeyName> {
        if !is_level(name) {
            return Err(MalformedKeyName(name.to_owned()));
        }
        Ok(Self(Arc::from(name)))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl Display for KeyName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

/// A spelling refused as a [`KeyName`], carried so the refusal names it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MalformedKeyName(String);

impl MalformedKeyName {
    pub fn spelling(&self) -> &str {
        &self.0
    }
}

impl Display for MalformedKeyName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "'{}' is not one level of ASCII letters, digits and '_'",
            self.0
        )
    }
}

impl Error for MalformedKeyName {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_key_name_is_one_level() {
        for name in ["shoulder_pan", "arm", "j1"] {
            let key_name = KeyName::new(name).expect("a valid name");

            assert_eq!(key_name.as_str(), name);
            assert_eq!(key_name.to_string(), name);
        }
    }

    #[test]
    fn a_name_that_isnt_one_level_is_refused_and_named() {
        // A dotted name is a frame leaf's grammar, not a key name's.
        for name in ["", "arm.left", "shoulder-pan", "wrist 3"] {
            let refused = KeyName::new(name).expect_err("a malformed name");

            assert_eq!(refused.spelling(), name);
            assert!(refused.to_string().contains(&format!("'{name}'")));
        }
    }
}
