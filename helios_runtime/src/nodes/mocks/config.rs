//! [`MockOracleConfig`] — the `[nodes.<name>]` section of a `MockOracle` entry.

use serde::{Deserialize, Serialize};

/// The `[nodes.<name>]` section of a `MockOracle` entry. The node reads only
/// the body's oracle channels, so it takes no keys; the type exists so a
/// misspelled key is an error rather than silently ignored.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct MockOracleConfig {}
