//! The inspector's portable data model — Layer 1 of the panel.
//!
//! An [`InspectorModel`] is the erased, serializable projection of one *subject* —
//! any selectable thing: an agent, a pedestrian, a tree, a sign. It aggregates
//! [`Section`]s from three sources: the subject's own truth (identity, pose), the
//! subject's own autonomy if it carries a pipeline (estimator, controller), and —
//! in future — what *other* agents perceive about it (tracker estimates + error).
//! "Agent-ness" is therefore never a property of the model; it is a property of a
//! section, expressed as a gatherer that yields nothing when it does not apply.
//!
//! The model is data only — no Bevy handles, no widgets — so it survives every
//! boundary (playback, digital twin, hardware, zenoh). The renderer is a separate,
//! disposable consumer. Values are canonical SI, always; display localization
//! (degrees, km/h) is the renderer's job, never the model's.

use std::{borrow::Cow, sync::Arc};

use helios_core::data::MonotonicTime;

use serde::{Deserialize, Serialize};

/// One subject's full inspector projection at a single instant.
///
/// Rebuilt each frame for the currently-selected subject. `agent` (the subject id)
/// and `at` tag the projection so the one type also serves multi-agent recording
/// and time-shifted playback unchanged. `at` is [`MonotonicTime`] — the same clock
/// the bus payloads carry — so a section's "estimate produced at t=…" lines up with
/// the model's own timestamp.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InspectorModel {
    pub subject: SubjectId,
    pub at: MonotonicTime,
    pub sections: Vec<Section>,
}

/// Identifies the inspected subject — *any* selectable, not just agents. Derived
/// from the entity's display identity, distinct from the `helios_test` `AgentId`. A
/// tree has one of these; it simply matches fewer gatherers than an agent does.
/// When stable ids land, this becomes the id a remote tracker's estimate is matched
/// against.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Deserialize, Serialize)]
pub struct SubjectId(pub Arc<str>);

/// One collapsible group in the panel: a single source's contribution about the
/// subject.
///
/// A gatherer produces a `Section` when it applies to the selected subject and
/// nothing when it does not — absence *is* the applies-predicate, so no per-type
/// enum is needed. A tree yields only identity/pose sections; an agent adds its
/// autonomy sections; a perceived subject adds one section per observing agent.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Section {
    pub id: SubsystemPath,
    pub title: Cow<'static, str>,
    pub rows: Vec<Row>,
}

/// A symbolic path identifying a section's *source*. Autonomy and perception
/// sections follow the `agent.<id>.<node>.<channel>` grammar shared with assertions
/// and observability — a tracker section is `agent.<observer>.tracker.<subject>`, so
/// "which agents see X" is the glob `agent.*.tracker.X`. Pure-display sections (a
/// tree's pose) use looser ids outside that namespace. Serializes as a bare string
/// so a glob subscriber matches the raw text.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SubsystemPath(pub Arc<str>);

/// One labeled line in a section.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Row {
    pub label: Cow<'static, str>,
    pub value: Value,
}

/// The erased row value — SI-canonical and display-agnostic. Parallels
/// `helios_test`'s `AssertionValue` but is *dimensioned*: assertions are
/// comparison-tuned and dimensionless, this is display-tuned. Externally tagged (not
/// `untagged`) so the wire form is self-describing and an older consumer can skip an
/// unknown variant instead of failing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Value {
    /// A dimensioned scalar in its SI unit (never a display unit).
    Scalar { value: f64, kind: Dimension },
    /// A 3-vector with its reference frame carried as data.
    Vector(FramedVec3),
    /// A boolean state (armed, enabled).
    Flag(bool),
    /// A discrete-state token with no unit (gear, control mode).
    Label(String),
}

/// The semantic dimension of a [`Value::Scalar`]; the SI unit is implied. Kept
/// minimal — only what identity, pose, velocity, estimate-vs-truth, and actuation
/// need today. `uom` and typed frames enforce this statically *below* the model; at
/// this boundary it is a runtime tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Dimension {
    Length,
    Velocity,
    Acceleration,
    Angle,
    AngularRate,
    Time,
    Ratio,
    Torque,
    Force,
}

/// A 3-vector plus the frame it lives in, carried as *data* — the runtime shadow of
/// the typed-frame family, so a consumer never has to guess ENU vs NED.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FramedVec3 {
    pub value: [f64; 3],
    pub frame: Frame,
}

/// Reference frame of a [`FramedVec3`], carried so a consumer never guesses world vs
/// body. `Enu` is the world frame (pose, state estimate); `Flu` is the body frame
/// (control outputs). NED/ECEF join when a consumer needs them. Any conversion into
/// these frames happens in gather, never here.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Frame {
    Enu,
    Flu,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A model exercising every `Value` variant inside one section, so a single
    /// round-trip covers the whole shape.
    fn sample_model() -> InspectorModel {
        InspectorModel {
            subject: SubjectId("human_8".into()),
            at: MonotonicTime(12.5),
            sections: vec![Section {
                id: SubsystemPath("agent.robot_3.tracker.human_8".into()),
                title: "Tracked by robot_3".into(),
                rows: vec![
                    Row {
                        label: "position".into(),
                        value: Value::Vector(FramedVec3 {
                            value: [1.0, 2.0, 3.0],
                            frame: Frame::Enu,
                        }),
                    },
                    Row {
                        label: "speed".into(),
                        value: Value::Scalar {
                            value: 1.4,
                            kind: Dimension::Velocity,
                        },
                    },
                    Row {
                        label: "identified".into(),
                        value: Value::Flag(true),
                    },
                    Row {
                        label: "class".into(),
                        value: Value::Label("pedestrian".into()),
                    },
                ],
            }],
        }
    }

    /// The model *is* the wire format — a recording / zenoh / playback consumer
    /// reads back exactly what was captured only if serialize → deserialize is
    /// value-identical. JSON stands in for any self-describing transport. If this
    /// ever fails on a newly added variant, that variant broke the contract, not
    /// the test.
    #[test]
    fn model_round_trips_through_json() {
        let model = sample_model();
        let json = serde_json::to_string(&model).expect("model serializes");
        let back: InspectorModel = serde_json::from_str(&json).expect("model deserializes");
        assert_eq!(model, back);
    }

    /// The path newtypes must serialize as *bare strings*, not wrapped structures.
    /// A glob subscriber matches on the raw `agent.<id>.<node>.<channel>` text, so if
    /// `SubsystemPath` ever wrapped its string in extra structure, observability
    /// addressing would silently break. This pins serde's transparent
    /// newtype-struct behavior as a guarded contract, not an accident.
    #[test]
    fn path_newtypes_serialize_as_plain_strings() {
        let path = SubsystemPath("agent.robot_3.tracker.human_8".into());
        assert_eq!(
            serde_json::to_string(&path).expect("path serializes"),
            "\"agent.robot_3.tracker.human_8\"",
        );

        let subject = SubjectId("human_8".into());
        assert_eq!(
            serde_json::to_string(&subject).expect("subject serializes"),
            "\"human_8\"",
        );
    }
}
