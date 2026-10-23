use crate::evaluation::path::{DerivedName, DerivedProblem};

use helios_core::prelude::{AgentId, FrameId};
use helios_runtime::{
    observe::path::{channel_path, AgentPathParts, PathPart, PathSubject, SUBJECT_WORDS},
    port::ChannelKey,
};

use serde::{
    de::{self, value::MapAccessDeserializer, MapAccess, Visitor},
    ser::SerializeMap,
    Deserialize, Deserializer, Serialize, Serializer,
};
use std::{fmt, sync::Arc};

/// The keys of a selector's table form.
const PATH_KEY: &str = "path";
const INSTANCE_KEY: &str = "instance";

/// How a selector asks for every instance of a keyed channel.
const EACH_INSTANCE: &str = "*";

/// _Shared (pre + per + post)._ A selector: the path of a series, held whole
/// and in its parts, and, on a keyed channel, which of its instances. A
/// recorded series' path is the watcher path it is recorded under
/// (`agent.<agent>.node.<node>.<leaf>` or `agent.<agent>.channel.<channel>`);
/// a derived one's is `eval.inline.<evaluation>.<metric>`.
///
/// Written as the path alone, or as `{ path = "…", instance = "…" }`: an
/// instance gets its own field because its spelling holds dots (a frame,
/// `agent.car.base_link`) the path grammar can't. `instance = "*"` asks for
/// every instance, each its own series.
///
/// Only a path that splits into its parts becomes a selector, so a run file
/// naming anything else is refused when it is read, before a sim starts.
/// Whether the agent reports what the path names, and has the instance, is
/// checked later, against the built pipelines.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct AssertionTarget {
    path: String,
    root: TargetRoot,
    instance: Option<InstanceSelector>,
}

impl AssertionTarget {
    /// The target `path` names, with no instance, or why it names none.
    pub fn parse(path: impl Into<String>) -> Result<Self, MalformedTarget> {
        let path = path.into();
        let root = if DerivedName::is_derived(&path) {
            DerivedName::parse(&path)
                .map(TargetRoot::Derived)
                .map_err(TargetFault::Derived)
        } else {
            AgentPathParts::split(&path)
                .map(|parts| TargetRoot::Agent {
                    agent: AgentId::new(parts.agent),
                    subject: TargetSubject::from(parts.subject),
                })
                .map_err(TargetFault::Recorded)
        };
        match root {
            Ok(root) => Ok(Self {
                path,
                root,
                instance: None,
            }),
            Err(fault) => Err(MalformedTarget { path, fault }),
        }
    }

    /// The selector of `channel` on `agent`, at `instance` when the channel
    /// is keyed by frame. Its path is spelled from the parts, never parsed,
    /// so it is well-formed by construction.
    pub fn channel(agent: &AgentId, channel: &ChannelKey, instance: Option<&FrameId>) -> Self {
        Self {
            path: channel_path(agent.as_str(), channel),
            root: TargetRoot::Agent {
                agent: agent.clone(),
                subject: TargetSubject::Channel {
                    channel: channel.name().clone(),
                },
            },
            instance: instance.map(|frame| InstanceSelector::One(frame.to_string().into())),
        }
    }

    /// This selector narrowed to `instance`.
    pub fn with_instance(self, instance: InstanceSelector) -> Self {
        Self {
            instance: Some(instance),
            ..self
        }
    }

    /// The watcher path, without the instance: what the host is asked to
    /// record, since a channel path watches every instance.
    pub fn path(&self) -> &str {
        &self.path
    }

    /// Whether it names a recorded series, and whose, or a derived one.
    pub fn root(&self) -> &TargetRoot {
        &self.root
    }

    /// Whether it names a series a host records, which the host is asked
    /// for. A derived series is computed from recorded ones instead.
    pub fn is_recorded(&self) -> bool {
        matches!(self.root, TargetRoot::Agent { .. })
    }

    /// Which instances of a keyed channel it selects. `None` when it names
    /// none, which only a series with no instance or one alone satisfies.
    pub fn instance(&self) -> Option<&InstanceSelector> {
        self.instance.as_ref()
    }

    /// Whether a value recorded at `instance` under this selector's path is
    /// one it selects.
    pub fn covers(&self, instance: Option<&str>) -> bool {
        match &self.instance {
            None | Some(InstanceSelector::Each) => true,
            Some(InstanceSelector::One(one)) => instance == Some(one.as_ref()),
        }
    }
}

impl TryFrom<String> for AssertionTarget {
    type Error = MalformedTarget;

    fn try_from(path: String) -> Result<Self, Self::Error> {
        Self::parse(path)
    }
}

/// The path, then `[instance=…]` when it names one, so every message about
/// it says which instance.
impl fmt::Display for AssertionTarget {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.path)?;
        if let Some(instance) = &self.instance {
            write!(f, "[{INSTANCE_KEY}={instance}]")?;
        }
        Ok(())
    }
}

/// The path, or a table with `path` and an optional `instance`. By hand
/// rather than `untagged`, so a malformed selector is refused with what a
/// selector may be.
impl<'de> Deserialize<'de> for AssertionTarget {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(TargetVisitor)
    }
}

/// As it was written: the path alone, or the table when it names an
/// instance.
impl Serialize for AssertionTarget {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match &self.instance {
            None => serializer.serialize_str(&self.path),
            Some(instance) => {
                let mut map = serializer.serialize_map(Some(2))?;
                map.serialize_entry(PATH_KEY, &self.path)?;
                map.serialize_entry(INSTANCE_KEY, &instance.to_string())?;
                map.end()
            }
        }
    }
}

struct TargetVisitor;

impl<'de> Visitor<'de> for TargetVisitor {
    type Value = AssertionTarget;

    fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "a watcher path, or {{ {PATH_KEY} = \"<path>\", {INSTANCE_KEY} = \"<instance>\" \
             or \"{EACH_INSTANCE}\" }}"
        )
    }

    fn visit_str<E: de::Error>(self, path: &str) -> Result<AssertionTarget, E> {
        AssertionTarget::parse(path).map_err(de::Error::custom)
    }

    fn visit_map<M: MapAccess<'de>>(self, map: M) -> Result<AssertionTarget, M::Error> {
        let table = TargetTable::deserialize(MapAccessDeserializer::new(map))?;
        let target = AssertionTarget::parse(table.path).map_err(de::Error::custom)?;
        Ok(match table.instance {
            Some(instance) => target.with_instance(instance),
            None => target,
        })
    }
}

/// A selector's table form as written.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TargetTable {
    path: String,
    #[serde(default)]
    instance: Option<InstanceSelector>,
}

/// _Shared (pre + per + post)._ Which instances of a keyed channel a selector
/// names: one, by its spelling, or each (`"*"`), every one its own series.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Deserialize)]
#[serde(try_from = "String")]
pub enum InstanceSelector {
    One(Arc<str>),
    Each,
}

impl TryFrom<String> for InstanceSelector {
    type Error = &'static str;

    fn try_from(instance: String) -> Result<Self, Self::Error> {
        match instance.as_str() {
            "" => Err("an instance is never empty; leave `instance` out to name none"),
            EACH_INSTANCE => Ok(InstanceSelector::Each),
            _ => Ok(InstanceSelector::One(instance.into())),
        }
    }
}

impl fmt::Display for InstanceSelector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            InstanceSelector::One(one) => f.write_str(one),
            InstanceSelector::Each => f.write_str(EACH_INSTANCE),
        }
    }
}

/// _Shared (pre + per)._ What a target names: a series an agent records, or
/// one an evaluation derives.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum TargetRoot {
    Agent {
        agent: AgentId,
        subject: TargetSubject,
    },
    Derived(DerivedName),
}

/// _Shared (pre + per)._ What kind of value a target names, with its parts
/// after the subject word.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum TargetSubject {
    /// What a pipeline node said about itself.
    Node {
        node: Arc<str>,
        /// The leaf, dots included.
        leaf: Arc<str>,
    },
    /// The values on one of the agent's channels, by its name: every
    /// instance of a keyed channel.
    Channel { channel: Arc<str> },
}

impl From<PathSubject<'_>> for TargetSubject {
    fn from(subject: PathSubject<'_>) -> Self {
        match subject {
            PathSubject::Node { node, leaf } => TargetSubject::Node {
                node: node.into(),
                leaf: leaf.into(),
            },
            PathSubject::Channel { channel } => TargetSubject::Channel {
                channel: channel.into(),
            },
        }
    }
}

/// _Pre-step._ A run-file target that doesn't split into the parts of a
/// watcher path or a derived key, and what is wrong with it.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("target '{path}' {fault}")]
pub struct MalformedTarget {
    path: String,
    fault: TargetFault,
}

/// _Pre-step._ What is wrong with a malformed target, by the root it was
/// read under.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum TargetFault {
    /// Not under the derived root, and the first part of a watcher path
    /// that is wrong.
    #[error(
        "is not a watcher path agent.<agent>.node.<node>.<leaf> or \
         agent.<agent>.channel.<channel>, nor a derived series \
         eval.inline.<evaluation>.<metric>: {}",
        what_is_wrong(*.0)
    )]
    Recorded(PathPart),
    #[error("is not a derived series eval.inline.<evaluation>.<metric>: {0}")]
    Derived(DerivedProblem),
}

/// What is wrong with a target whose `part` is the first that is.
fn what_is_wrong(part: PathPart) -> String {
    match part {
        PathPart::Root => "it starts with something else".to_owned(),
        PathPart::Agent => "the agent is missing".to_owned(),
        PathPart::Subject => format!(
            "the word after the agent is missing or not one of {}",
            SUBJECT_WORDS.join(", ")
        ),
        PathPart::Node => "the node is missing".to_owned(),
        PathPart::Leaf => "the leaf is missing".to_owned(),
        PathPart::Channel => "the channel is missing".to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::EntryName;

    use helios_runtime::channels;

    #[test]
    fn parse_keeps_the_path_and_its_parts() {
        let target = AssertionTarget::parse("agent.car.node.estimator.aiding.gps.nis")
            .expect("valid target");

        assert_eq!(target.path(), "agent.car.node.estimator.aiding.gps.nis");
        assert!(target.is_recorded());
        assert_eq!(
            target.root(),
            &TargetRoot::Agent {
                agent: AgentId::new("car"),
                subject: TargetSubject::Node {
                    node: "estimator".into(),
                    leaf: "aiding.gps.nis".into(),
                },
            }
        );
    }

    #[test]
    fn parse_keeps_a_channel_target() {
        let target = AssertionTarget::parse("agent.car.channel.truth.tf").expect("valid target");

        assert_eq!(
            target.root(),
            &TargetRoot::Agent {
                agent: AgentId::new("car"),
                subject: TargetSubject::Channel {
                    channel: "truth.tf".into(),
                },
            }
        );
    }

    #[test]
    fn parse_keeps_a_derived_target() {
        let target = AssertionTarget::parse("eval.inline.twist.surge_error").expect("valid target");

        assert!(!target.is_recorded());
        assert_eq!(
            target.root(),
            &TargetRoot::Derived(DerivedName::new(
                EntryName::parse("twist").expect("valid name"),
                "surge_error"
            ))
        );
    }

    #[test]
    fn a_malformed_derived_target_says_what_is_wrong() {
        let error = AssertionTarget::parse("eval.inline.twist").expect_err("no metric");

        assert_eq!(
            error.to_string(),
            "target 'eval.inline.twist' is not a derived series \
             eval.inline.<evaluation>.<metric>: the metric is missing or not one level"
        );
    }

    #[test]
    fn parse_refuses_a_path_with_no_leaf() {
        assert_eq!(
            AssertionTarget::parse("agent.car.node.estimator"),
            Err(MalformedTarget {
                path: "agent.car.node.estimator".to_owned(),
                fault: TargetFault::Recorded(PathPart::Leaf),
            })
        );
    }

    #[test]
    fn a_malformed_target_says_which_part_is_wrong() {
        let error = AssertionTarget::parse("agent.car.node").expect_err("no node or leaf");

        assert!(
            error.to_string().ends_with("the node is missing"),
            "{error}"
        );
    }

    #[test]
    fn a_target_without_a_subject_says_which_subjects_exist() {
        let error =
            AssertionTarget::parse("agent.car.estimator.speed").expect_err("no subject word");

        assert!(
            error.to_string().ends_with("not one of node, channel"),
            "{error}"
        );
    }

    #[test]
    fn parse_refuses_a_path_outside_both_roots_naming_both() {
        let error = AssertionTarget::parse("car.estimator.speed.x").expect_err("no root");

        assert!(error.to_string().contains("eval.inline."), "{error}");
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct Wrap {
        target: AssertionTarget,
    }

    fn read(text: &str) -> Result<AssertionTarget, toml::de::Error> {
        toml::from_str::<Wrap>(&format!("target = {text}")).map(|wrap| wrap.target)
    }

    const TF: &str = "agent.car.channel.truth.tf";

    #[test]
    fn a_selector_is_a_path_or_a_table_naming_an_instance() {
        let path = AssertionTarget::parse(TF).expect("valid target");
        assert_eq!(read(&format!("\"{TF}\"")).unwrap(), path);
        assert_eq!(read(&format!("{{ path = \"{TF}\" }}")).unwrap(), path);

        let base = read(&format!(
            "{{ path = \"{TF}\", instance = \"agent.car.base_link\" }}"
        ))
        .unwrap();
        assert_eq!(
            base.instance(),
            Some(&InstanceSelector::One("agent.car.base_link".into()))
        );
        assert_eq!(base.path(), TF);

        let each = read(&format!("{{ path = \"{TF}\", instance = \"*\" }}")).unwrap();
        assert_eq!(each.instance(), Some(&InstanceSelector::Each));
        assert_eq!(each.to_string(), format!("{TF}[instance=*]"));
    }

    #[test]
    fn a_malformed_selector_is_refused_at_parse() {
        for text in [
            "\"agent.car\"".to_string(),
            format!("{{ path = \"{TF}\", instance = \"\" }}"),
            format!("{{ path = \"{TF}\", frame = \"base\" }}"),
            "{ instance = \"base\" }".to_string(),
            "3".to_string(),
        ] {
            assert!(read(&text).is_err(), "{text}");
        }
    }

    #[test]
    fn a_selector_serializes_as_it_was_written() {
        for text in [
            format!("\"{TF}\""),
            format!("{{ path = \"{TF}\", instance = \"agent.car.base_link\" }}"),
            format!("{{ path = \"{TF}\", instance = \"*\" }}"),
        ] {
            let target = read(&text).unwrap();
            let written = toml::to_string(&Wrap {
                target: target.clone(),
            })
            .unwrap();
            assert_eq!(toml::from_str::<Wrap>(&written).unwrap().target, target);
        }
    }

    #[test]
    fn a_selector_built_from_its_parts_equals_the_one_its_spelling_parses_to() {
        let car = AgentId::new("car");
        let base_link = FrameId::base_link(car.clone());

        let built = AssertionTarget::channel(&car, &channels::truth_tf(), Some(&base_link));
        let parsed = read(&format!(
            "{{ path = \"{TF}\", instance = \"agent.car.base_link\" }}"
        ))
        .unwrap();

        assert_eq!(built, parsed);
    }

    #[test]
    fn a_selector_covers_the_instances_it_names() {
        let path = AssertionTarget::parse(TF).expect("valid target");
        let base = path
            .clone()
            .with_instance(InstanceSelector::One("base".into()));
        let each = path.clone().with_instance(InstanceSelector::Each);

        assert!(path.covers(Some("imu")) && path.covers(None));
        assert!(each.covers(Some("imu")));
        assert!(base.covers(Some("base")));
        assert!(!base.covers(Some("imu")) && !base.covers(None));
    }
}
