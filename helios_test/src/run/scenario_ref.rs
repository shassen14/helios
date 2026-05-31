use serde::Deserialize;

/// Deferred reference to a scenario by config path (the `{ from = "..." }`
/// form used throughout `configs/`).
///
/// Currently, `helios_test` is Bevy-free and cannot build a scenario
///  — so this type only captures the reference string and hands it back;
/// nothing resolves it yet. Modeled as a one-field struct rather than a bare
///  `String` to match the composition-by-reference convention and so 
/// `deny_unknown_fields` can reject typos in the run file.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioRef {
    from: String,
}

impl ScenarioRef {
    /// The unresolved config path this reference points at. Named `path`
    /// rather than `from` so it reads as "what the string is"
    /// and to avoid shadowing the conventional `From::from`.
    pub fn path(&self) -> &str {
        &self.from
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_from_reference() {
        let parsed: ScenarioRef =
            toml::from_str(r#"from = "sim.scenarios.parking_lot""#).unwrap();
        assert_eq!(parsed.path(), "sim.scenarios.parking_lot");
    }

    #[test]
    fn rejects_unknown_field() {
        // `deny_unknown_fields` turns a misspelled key into a hard parse error
        // rather than a silently-defaulted field.
        assert!(toml::from_str::<ScenarioRef>(r#"form = "x""#).is_err());
    }

    #[test]
    fn requires_table_not_bare_string() {
        // Locks the `{ from = "..." }` shape: a bare string is not a valid
        // reference. Tested through a wrapper because that is how a run file
        // embeds it (`scenario = { from = "..." }`).
        #[derive(Deserialize)]
        struct Wrap {
            scenario: ScenarioRef,
        }
        let w: Wrap = toml::from_str(r#"scenario = { from = "x" }"#).unwrap();
        assert_eq!(w.scenario.path(), "x");
        assert!(toml::from_str::<Wrap>(r#"scenario = "x""#).is_err());
    }
}
