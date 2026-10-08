//! Component tables: the registries a node factory draws its parts from.
//!
//! Some node kinds are not one algorithm but several independent choices. A
//! recursive estimator is a filter, a dynamics model and a set of measurement
//! models, and each is named by its own `kind` in a sub-table of the node's
//! section. A [`ComponentTable`] maps those kinds to factories the way the node
//! map maps node kinds: the sub-table is parsed strictly into the kind's own
//! config type, its resolved form is kept for the config dump, and every error
//! names the node, the sub-table and the kind.
//!
//! A table is generic over what its factories receive besides config (`X`,
//! the parts the node factory has already built) and what they return (`T`).

use super::factory::{BuildContext, KIND_KEY};

use serde::{de::DeserializeOwned, Deserialize, Serialize};

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt::Display;

/// Kind → factory for one component slot (`filter`, `dynamics`, …).
///
/// Sorted, so an unknown-kind error lists the registered kinds in a stable
/// order.
pub(crate) struct ComponentTable<X, T> {
    /// What the slot is called in errors, e.g. `"filter"`.
    slot: &'static str,
    factories: BTreeMap<String, ErasedComponentFactory<X, T>>,
}

impl<X: 'static, T: 'static> ComponentTable<X, T> {
    /// An empty table for the component slot named `slot`.
    pub(crate) fn new(slot: &'static str) -> Self {
        Self {
            slot,
            factories: BTreeMap::new(),
        }
    }

    /// Every registered kind, sorted.
    pub(crate) fn kinds(&self) -> Vec<String> {
        self.factories.keys().cloned().collect()
    }

    /// Registers `build` under `kind`.
    ///
    /// `C` must carry `#[serde(deny_unknown_fields)]` for a misspelled key to
    /// be an error rather than silently ignored. Fails if `kind` is already
    /// registered in this slot; the first registration is kept.
    pub(crate) fn register<C, F>(
        &mut self,
        kind: impl Into<String>,
        build: F,
    ) -> Result<(), DuplicateComponentKind>
    where
        C: DeserializeOwned + Serialize + 'static,
        F: Fn(C, &BuildContext<'_>, X) -> Result<T, String> + Send + Sync + 'static,
    {
        let kind = kind.into();
        if self.factories.contains_key(&kind) {
            return Err(DuplicateComponentKind {
                slot: self.slot,
                kind,
            });
        }

        self.factories.insert(kind, erase(build));

        Ok(())
    }

    /// Builds the component described by `section`, the sub-table found at
    /// `path` inside node `ctx.node_name()`'s section (e.g. `"filter"` or
    /// `"aiding.gps.model"`).
    ///
    /// Takes `kind` out of the section, parses the rest into that kind's
    /// config and calls its factory with `parts`. The resolved section carries
    /// `kind` back, so the dump shows the whole sub-table.
    pub(crate) fn build(
        &self,
        path: &str,
        mut section: toml::Table,
        ctx: &BuildContext<'_>,
        parts: X,
    ) -> Result<BuiltComponent<T>, ComponentError> {
        let site = || Site {
            node_name: ctx.node_name().to_string(),
            path: path.to_string(),
        };

        let kind = match section.remove(KIND_KEY) {
            Some(toml::Value::String(kind)) => kind,
            Some(_) | None => return Err(ComponentError::MissingKind { site: site() }),
        };

        let Some(factory) = self.factories.get(&kind) else {
            return Err(ComponentError::UnknownKind {
                site: site(),
                slot: self.slot,
                kind,
                registered: self.kinds(),
            });
        };

        let (component, mut resolved) = factory(section, ctx, parts).map_err(|failure| {
            let site = site();
            let kind = kind.clone();
            match failure {
                Failure::InvalidConfig { key, message } => ComponentError::InvalidConfig {
                    site,
                    kind,
                    key,
                    message,
                },
                Failure::BuildFailed(reason) => ComponentError::BuildFailed { site, kind, reason },
                Failure::ResolveFailed(message) => ComponentError::ResolveFailed {
                    site,
                    kind,
                    message,
                },
            }
        })?;

        resolved.insert(KIND_KEY.to_string(), toml::Value::String(kind));

        Ok(BuiltComponent {
            component,
            resolved,
        })
    }
}

/// What [`ComponentTable::build`] returns: the component and its resolved
/// sub-table, defaults filled in and `kind` included, for the config dump.
#[cfg_attr(
    not(test),
    expect(dead_code, reason = "no node factory builds estimator components yet")
)]
pub(crate) struct BuiltComponent<T> {
    pub(crate) component: T,
    pub(crate) resolved: toml::Table,
}

/// The config of a component kind that takes no parameters.
///
/// Still parsed strictly, so a key written under such a kind is an error
/// rather than a setting that silently does nothing.
#[derive(Debug, Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct NoParams {}

/// A component kind was registered twice in one slot.
#[derive(Debug)]
pub struct DuplicateComponentKind {
    /// The slot the kind was registered in, e.g. `"filter"`.
    pub slot: &'static str,
    /// The kind string that was already taken.
    pub kind: String,
}

impl Display for DuplicateComponentKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} kind '{}' is already registered",
            self.slot, self.kind
        )
    }
}

impl Error for DuplicateComponentKind {}

/// Why a component failed to build. Every variant names the node and the
/// sub-table, so the message points at the config entry to fix.
#[derive(Debug)]
pub enum ComponentError {
    /// The sub-table has no `kind` key, or its `kind` is not a string.
    MissingKind { site: Site },
    /// No factory is registered for `kind` in this slot.
    UnknownKind {
        site: Site,
        slot: &'static str,
        kind: String,
        registered: Vec<String>,
    },
    /// The sub-table did not parse into the kind's config type. `key` is the
    /// path to the offending key inside the sub-table; empty or `"."` when the
    /// error is about the sub-table as a whole.
    InvalidConfig {
        site: Site,
        kind: String,
        key: String,
        message: String,
    },
    /// The config parsed, but the kind's factory rejected it.
    BuildFailed {
        site: Site,
        kind: String,
        reason: String,
    },
    /// The parsed config did not serialize back to a table for the dump. A bug
    /// in the kind's config type, not in the user's config.
    ResolveFailed {
        site: Site,
        kind: String,
        message: String,
    },
}

impl Display for ComponentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingKind { site } => write!(f, "{site}: no `{KIND_KEY}` string"),
            Self::UnknownKind {
                site,
                slot,
                kind,
                registered,
            } => {
                let registered = if registered.is_empty() {
                    "(none)".to_string()
                } else {
                    registered.join(", ")
                };
                write!(
                    f,
                    "{site}: unknown {slot} kind '{kind}'; registered {slot} kinds: {registered}"
                )
            }
            Self::InvalidConfig {
                site,
                kind,
                key,
                message,
            } => {
                if key.is_empty() || key == "." {
                    write!(f, "{site} (kind '{kind}'): {message}")
                } else {
                    write!(f, "{site}.{key} (kind '{kind}'): {message}")
                }
            }
            Self::BuildFailed { site, kind, reason } => {
                write!(f, "{site} (kind '{kind}'): build failed: {reason}")
            }
            Self::ResolveFailed {
                site,
                kind,
                message,
            } => write!(
                f,
                "{site} (kind '{kind}'): config type does not serialize back to a TOML table: \
                 {message}"
            ),
        }
    }
}

impl Error for ComponentError {}

/// Where a component's sub-table sits: the node's name and the path inside its
/// section. Displays as the full config path, `nodes.<node>.<path>`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Site {
    pub node_name: String,
    pub path: String,
}

impl Display for Site {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "nodes.{}.{}", self.node_name, self.path)
    }
}

/// A component factory with its config type hidden, so every kind of one slot
/// fits in one map. Returns the component and its resolved sub-table (without
/// `kind`), or a [`Failure`] the table names. Built by [`erase`].
type ErasedComponentFactory<X, T> = Box<
    dyn Fn(toml::Table, &BuildContext<'_>, X) -> Result<(T, toml::Table), Failure> + Send + Sync,
>;

/// A factory failure before the table attaches the site and kind.
enum Failure {
    InvalidConfig { key: String, message: String },
    BuildFailed(String),
    ResolveFailed(String),
}

/// Wraps a kind's typed build function into an [`ErasedComponentFactory`]:
/// parse strictly into `C`, serialize it back as the resolved sub-table, then
/// build. Serializing comes first because `build` takes the config by value.
fn erase<X, T, C, F>(build: F) -> ErasedComponentFactory<X, T>
where
    X: 'static,
    T: 'static,
    C: DeserializeOwned + Serialize + 'static,
    F: Fn(C, &BuildContext<'_>, X) -> Result<T, String> + Send + Sync + 'static,
{
    Box::new(move |section, ctx, parts| {
        let config: C =
            serde_path_to_error::deserialize(section).map_err(|err| Failure::InvalidConfig {
                key: err.path().to_string(),
                message: err.inner().to_string().trim_end().to_string(),
            })?;

        let resolved = toml::Table::try_from(&config)
            .map_err(|err| Failure::ResolveFailed(err.to_string().trim_end().to_string()))?;

        let component = build(config, ctx, parts).map_err(Failure::BuildFailed)?;

        Ok((component, resolved))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::prelude::AgentId;

    use std::collections::HashSet;

    const NODE: &str = "primary";
    const SLOT: &str = "filter";
    const PATH: &str = "filter";
    const KIND: &str = "Gain";
    const OTHER_KIND: &str = "Adder";
    const DEFAULT_OFFSET: f64 = 0.5;
    const REJECTED_GAIN: f64 = -1.0;

    /// A dummy component: multiplies its part by a gain and adds an offset.
    #[derive(Deserialize, Serialize)]
    #[serde(deny_unknown_fields)]
    struct GainConfig {
        gain: f64,
        #[serde(default = "default_offset")]
        offset: f64,
    }

    fn default_offset() -> f64 {
        DEFAULT_OFFSET
    }

    fn build_gain(config: GainConfig, _ctx: &BuildContext<'_>, part: f64) -> Result<f64, String> {
        if config.gain == REJECTED_GAIN {
            return Err("negative gain".to_string());
        }
        Ok(part * config.gain + config.offset)
    }

    fn table_with(kinds: &[&str]) -> ComponentTable<f64, f64> {
        let mut table = ComponentTable::new(SLOT);
        for kind in kinds {
            table
                .register(*kind, build_gain)
                .expect("test kinds are distinct");
        }
        table
    }

    fn build(
        table: &ComponentTable<f64, f64>,
        section: &str,
    ) -> Result<BuiltComponent<f64>, ComponentError> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("car"), NODE, &channels);
        table.build(PATH, section, &ctx, 2.0)
    }

    fn build_err(table: &ComponentTable<f64, f64>, section: &str) -> ComponentError {
        match build(table, section) {
            Ok(_) => panic!("expected the build to fail"),
            Err(err) => err,
        }
    }

    fn site() -> Site {
        Site {
            node_name: NODE.to_string(),
            path: PATH.to_string(),
        }
    }

    /// A registered kind builds with the part it was handed, and the resolved
    /// sub-table shows its defaults and its kind.
    #[test]
    fn registered_kind_builds_with_its_resolved_sub_table() {
        let table = table_with(&[KIND]);
        let Ok(built) = build(&table, &format!("kind = \"{KIND}\"\ngain = 3.0")) else {
            panic!("expected the build to succeed");
        };
        assert_eq!(built.component, 2.0 * 3.0 + DEFAULT_OFFSET);
        assert_eq!(built.resolved["kind"].as_str(), Some(KIND));
        assert_eq!(built.resolved["gain"].as_float(), Some(3.0));
        assert_eq!(built.resolved["offset"].as_float(), Some(DEFAULT_OFFSET));
    }

    #[test]
    fn registering_a_kind_twice_is_rejected() {
        let mut table = table_with(&[KIND]);
        let err = table
            .register(KIND, build_gain)
            .expect_err("a second registration fails");
        assert_eq!((err.slot, err.kind.as_str()), (SLOT, KIND));
        assert_eq!(
            err.to_string(),
            format!("{SLOT} kind '{KIND}' is already registered")
        );
    }

    /// A missing or non-string `kind` names the sub-table.
    #[test]
    fn missing_or_non_string_kind_is_rejected() {
        let table = table_with(&[KIND]);
        for section in ["gain = 1.0", "kind = 3\ngain = 1.0"] {
            let err = build_err(&table, section);
            assert!(
                matches!(&err, ComponentError::MissingKind { site: s } if *s == site()),
                "{err}"
            );
            assert_eq!(
                err.to_string(),
                format!("nodes.{NODE}.{PATH}: no `kind` string")
            );
        }
    }

    /// An unknown kind lists the slot's registered kinds, sorted.
    #[test]
    fn unknown_kind_lists_the_slots_kinds_sorted() {
        let table = table_with(&[KIND, OTHER_KIND]);
        let err = build_err(&table, "kind = \"Gian\"");
        assert_eq!(
            err.to_string(),
            format!(
                "nodes.{NODE}.{PATH}: unknown {SLOT} kind 'Gian'; \
                 registered {SLOT} kinds: {OTHER_KIND}, {KIND}"
            )
        );
    }

    #[test]
    fn unknown_kind_with_none_registered_says_none() {
        let err = build_err(&table_with(&[]), "kind = \"Gain\"");
        assert!(err.to_string().ends_with("kinds: (none)"), "{err}");
    }

    /// A misspelled key is reported at its full config path.
    #[test]
    fn unknown_key_is_reported_at_its_full_path() {
        let table = table_with(&[KIND]);
        let err = build_err(
            &table,
            &format!("kind = \"{KIND}\"\ngain = 1.0\nofset = 2.0"),
        );
        let ComponentError::InvalidConfig { key, .. } = &err else {
            panic!("expected InvalidConfig, got {err}");
        };
        assert_eq!(key, "ofset");
        assert!(
            err.to_string()
                .starts_with(&format!("nodes.{NODE}.{PATH}.ofset (kind '{KIND}'): ")),
            "{err}"
        );
    }

    #[test]
    fn missing_field_is_reported_at_the_sub_table() {
        let table = table_with(&[KIND]);
        let err = build_err(&table, &format!("kind = \"{KIND}\""));
        assert_eq!(
            err.to_string(),
            format!("nodes.{NODE}.{PATH} (kind '{KIND}'): missing field `gain`")
        );
    }

    #[test]
    fn factory_rejection_names_the_sub_table_and_kind() {
        let table = table_with(&[KIND]);
        let err = build_err(
            &table,
            &format!("kind = \"{KIND}\"\ngain = {REJECTED_GAIN:?}"),
        );
        assert_eq!(
            err.to_string(),
            format!("nodes.{NODE}.{PATH} (kind '{KIND}'): build failed: negative gain")
        );
    }

    /// A parameterless kind still rejects a stray key.
    #[test]
    fn no_params_rejects_any_key() {
        let mut table: ComponentTable<(), ()> = ComponentTable::new(SLOT);
        table
            .register(KIND, |_: NoParams, _: &BuildContext<'_>, ()| Ok(()))
            .expect("one registration");
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("car"), NODE, &channels);
        let section: toml::Table =
            toml::from_str(&format!("kind = \"{KIND}\"\nalpha = 1.0")).expect("parses");
        assert!(matches!(
            table.build(PATH, section, &ctx, ()),
            Err(ComponentError::InvalidConfig { .. })
        ));
    }
}
