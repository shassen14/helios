//! [`NoParams`], and how every shape's registry adapter reads an entry's
//! `[evaluate.params]`: strictly, checked by the kind, a problem naming it.

use serde::{de::DeserializeOwned, Deserialize};

/// _Pre-step._ The parameters of a kind that reads none: any key is
/// refused.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NoParams {}

/// `params` read as `P`, a kind's parameters, and checked by `check`, or
/// what is wrong with them, prefixed with the kind's name `kind`.
pub(super) fn read<P: DeserializeOwned>(
    kind: &str,
    params: &toml::Table,
    check: impl FnOnce(&P) -> Result<(), String>,
) -> Result<P, String> {
    P::deserialize(params.clone())
        .map_err(|error| error.message().to_owned())
        .and_then(|params| check(&params).map(|()| params))
        .map_err(|problem| format!("{kind}: {problem}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reads one gain, refusing a negative one.
    #[derive(Debug, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Gain {
        gain: f64,
    }

    fn non_negative(params: &Gain) -> Result<(), String> {
        match params.gain {
            gain if gain < 0.0 => Err("a negative gain".to_owned()),
            _ => Ok(()),
        }
    }

    fn table(key: &str, value: f64) -> toml::Table {
        let mut params = toml::Table::new();
        params.insert(key.to_owned(), toml::Value::Float(value));
        params
    }

    #[test]
    fn params_that_parse_and_pass_the_check_are_read() {
        let params = read("gained", &table("gain", 2.0), non_negative);

        assert_eq!(params.map(|params| params.gain), Ok(2.0));
    }

    #[test]
    fn a_misspelled_key_and_a_refused_check_each_name_the_kind() {
        let misspelled = read("gained", &table("gian", 2.0), non_negative);
        let negative = read("gained", &table("gain", -1.0), non_negative);

        let misspelled = misspelled.expect_err("an unknown key");
        assert!(
            misspelled.starts_with("gained: unknown field `gian`"),
            "{misspelled}"
        );
        assert_eq!(negative.expect_err("refused"), "gained: a negative gain");
    }

    #[test]
    fn no_params_refuses_any_key() {
        let any_key = read::<NoParams>("none", &table("gain", 1.0), |_| Ok(()));
        let none = read::<NoParams>("none", &toml::Table::new(), |_| Ok(()));

        assert!(any_key.is_err());
        assert_eq!(none, Ok(NoParams {}));
    }
}
