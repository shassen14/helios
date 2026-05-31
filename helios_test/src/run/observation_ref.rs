use serde::Deserialize;

/// Deferred reference to an observation preset by config path (the
/// `{ from = "..." }` form used throughout `configs/`).
///
/// Like [`ScenarioRef`](super::scenario_ref::ScenarioRef), this only captures
/// the reference string and hands it back; nothing wires up observation yet.
/// The one-field struct matches the composition-by-reference convention and
/// lets `deny_unknown_fields` catch typos in the run file.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservationRef {
    from: String,
}

impl ObservationRef {
    /// The unresolved config path this reference points at. Named `path`
    /// rather than `from` so it reads as "what the string is" at the call
    /// site, and to avoid shadowing the conventional `From::from`.
    pub fn path(&self) -> &str {
        &self.from
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_from_reference() {
        let parsed: ObservationRef =
            toml::from_str(r#"from = "observation.presets.verbose""#).unwrap();
        assert_eq!(parsed.path(), "observation.presets.verbose");
    }

    #[test]
    fn rejects_unknown_field() {
        // `deny_unknown_fields` turns a misspelled key into a hard parse error
        // rather than a silently-defaulted field.
        assert!(toml::from_str::<ObservationRef>(r#"form = "x""#).is_err());
    }

    #[test]
    fn requires_table_not_bare_string() {
        // Locks the `{ from = "..." }` shape: a bare string is not a valid
        // reference. Tested through a wrapper because that is how a run file
        // embeds it (`observation = { from = "..." }`).
        #[derive(Deserialize)]
        struct Wrap {
            observation: ObservationRef,
        }
        let w: Wrap = toml::from_str(r#"observation = { from = "x" }"#).unwrap();
        assert_eq!(w.observation.path(), "x");
        assert!(toml::from_str::<Wrap>(r#"observation = "x""#).is_err());
    }
}
