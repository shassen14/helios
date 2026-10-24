use helios_core::kernel::identifier::is_level;

use serde::{Deserialize, Serialize};
use std::fmt;
use thiserror::Error;

/// _Shared (pre + per + post)._ The name an assertion or a measure is known
/// by: one level of the name grammar, unique across a run file's assertions
/// and measures together. The report, a `fail_fast` stop and a comparison of
/// two runs identify an entry by it, so inserting an entry never re-pairs
/// two runs' verdicts.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Deserialize, Serialize)]
#[serde(try_from = "String")]
pub struct EntryName(String);

impl EntryName {
    /// The name `name` spells, or why it spells none.
    pub fn parse(name: impl Into<String>) -> Result<Self, MalformedName> {
        let name = name.into();
        if is_level(&name) {
            Ok(Self(name))
        } else {
            Err(MalformedName(name))
        }
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl TryFrom<String> for EntryName {
    type Error = MalformedName;

    fn try_from(name: String) -> Result<Self, Self::Error> {
        Self::parse(name)
    }
}

impl fmt::Display for EntryName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

/// _Pre-step._ A name that isn't one level: empty, or holding a character
/// other than an ASCII letter, digit or `_`.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
#[error("'{0}' is not a name: use ASCII letters, digits and '_' only")]
pub struct MalformedName(String);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_level_is_a_name() {
        assert_eq!(
            EntryName::parse("gps_nis_settled").map(|name| name.to_string()),
            Ok("gps_nis_settled".to_string())
        );
    }

    #[test]
    fn a_dotted_empty_or_spaced_name_is_refused() {
        for name in ["gps.nis", "", "gps nis", "gps-nis"] {
            assert!(EntryName::parse(name).is_err(), "{name:?}");
        }
    }

    #[test]
    fn a_name_is_checked_as_it_is_read() {
        #[derive(Debug, Deserialize)]
        struct Wrap {
            #[allow(dead_code)]
            name: EntryName,
        }
        assert!(toml::from_str::<Wrap>("name = \"ok_name\"").is_ok());
        assert!(toml::from_str::<Wrap>("name = \"not.ok\"").is_err());
    }
}
