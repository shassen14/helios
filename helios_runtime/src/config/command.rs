//! [`CommandFoldConfig`] — one `[command.<fold>]` table of the stack.

use serde::Deserialize;

/// A command fold: a `Sum` of its members' commands, named by its table key
/// and published on a channel of that name for an allocator to read.
///
/// Members are named by their `[nodes]` table key. A controller writes its
/// command on a channel named after itself and knows nothing of this seam;
/// which fold it joins, and whether the fold requires it, is a fact about the
/// stack, so it is stated here. A stack may have any number of folds,
/// including several of one type.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct CommandFoldConfig {
    /// The command type the fold sums, by its registered name (e.g.
    /// `"DriveForce"`). Every member must write this type.
    #[serde(rename = "type")]
    pub type_name: String,

    /// Members that must all have published for the fold to publish, such as
    /// a feedback leg: a sum missing a term is silently wrong.
    #[serde(default)]
    pub required: Vec<String>,

    /// Members folded in when present, such as a feedforward leg.
    #[serde(default)]
    pub optional: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::collections::BTreeMap;

    #[test]
    fn folds_are_keyed_by_name_and_may_share_a_type() {
        // A skid-steer's two sides: two folds of one type.
        let folds: BTreeMap<String, CommandFoldConfig> = toml::from_str(
            r#"
            [left_cmd]
            type = "DriveForce"
            required = ["left_speed"]

            [right_cmd]
            type = "DriveForce"
            required = ["right_speed"]
            optional = ["right_ff"]
            "#,
        )
        .expect("two folds of one type parse");

        assert_eq!(
            folds.keys().map(String::as_str).collect::<Vec<_>>(),
            ["left_cmd", "right_cmd"]
        );
        assert_eq!(folds["right_cmd"].type_name, "DriveForce");
        assert!(folds["left_cmd"].optional.is_empty());
    }

    #[test]
    fn type_is_required() {
        let result: Result<CommandFoldConfig, _> = toml::from_str(r#"required = ["pid"]"#);

        assert!(result.is_err(), "a fold without a type must not parse");
    }

    #[test]
    fn an_unknown_key_is_rejected() {
        let result: Result<CommandFoldConfig, _> =
            toml::from_str("type = \"DriveForce\"\nfeedback = [\"pid\"]");

        assert!(result.is_err(), "`feedback` is not a fold key");
    }
}
