//! [`ActuatorSeamConfig`] — the stack's `[actuators]` section.

use serde::Deserialize;

/// Which nodes' partial actuator commands are merged into the one command the
/// body applies.
///
/// Members are named by their `[nodes]` table key. Each writes an
/// `ActuatorCommand` on a channel named after itself, over the actuators its
/// factory declared; whether it reaches the body is a fact about the stack, so
/// it is stated here. The list is explicit even for a lone member, so the
/// resolved config shows the whole topology.
#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct ActuatorSeamConfig {
    /// The members merged, each driving actuators no other member drives.
    pub members: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn members_parse_in_order() {
        let config: ActuatorSeamConfig =
            toml::from_str(r#"members = ["drive", "steer"]"#).expect("a member list parses");

        assert_eq!(config.members, ["drive", "steer"]);
    }

    #[test]
    fn members_are_required() {
        let result: Result<ActuatorSeamConfig, _> = toml::from_str("");

        assert!(result.is_err(), "a section without members must not parse");
    }

    #[test]
    fn an_unknown_key_is_rejected() {
        let result: Result<ActuatorSeamConfig, _> =
            toml::from_str("members = [\"drive\"]\ntype = \"ActuatorCommand\"");

        assert!(result.is_err(), "the seam's type is fixed, not stated");
    }
}
