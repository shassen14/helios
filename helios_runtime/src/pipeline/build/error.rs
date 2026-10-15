//! [`PipelineBuildError`], everything [`build`](super::PipelineBuilder::build)
//! can reject, [`ChannelUses`], who writes and reads one channel,
//! [`CycleEdge`], one read inside a dependency loop, and [`MountRefusal`], why
//! a mount the body states is refused.

use crate::{
    pipeline::autonomy_pipeline::PIPELINE_LEAF_GROUP,
    port::{ChannelKey, ChannelKind, InputNeed, Writer},
};

use helios_core::spatial::FrameId;

use std::error::Error;

/// Errors produced by [`PipelineBuilder::build`](super::PipelineBuilder::build).
///
/// `build` returns every error found in a stage as a `Vec` rather than
/// stopping at the first. Wiring errors are reported before ordering runs,
/// so a pipeline with both reports the wiring errors first. A node's own
/// config is checked earlier, by its kind's factory; this type checks DAG
/// structure, and carries the failures of nodes built by plan once the
/// structure is sound.
#[derive(Debug)]
pub enum PipelineBuildError {
    /// A set of nodes wait on each other's outputs in the same tick, so no
    /// order runs each after its producers. One error is raised per loop.
    ///
    /// `participants` names the nodes in the loop, sorted. A node that is
    /// only downstream of the loop is not listed, though it cannot be placed
    /// either. `edges` lists every same-tick read between participants,
    /// sorted by producer then consumer; a node reading its own output is a
    /// loop of one with a single edge.
    Cycle {
        participants: Vec<String>,
        edges: Vec<CycleEdge>,
    },
    /// A node reads a sensor or internal channel that has no supplier: no
    /// node writes it, the body does not publish it, and it is not declared
    /// as an outside input. Raised for optional inputs too, since optional
    /// means the node runs without a value, not that the wiring is unchecked.
    /// For a sensor channel, often a misspelled name or a sensor missing from
    /// the body's fit.
    UnsatisfiedInput {
        node_name: String,
        channel: ChannelKey,
        need: InputNeed,
    },
    /// A node reads an oracle or health channel that the body does not
    /// publish. Only a body supplies these.
    UnsatisfiedBodyCapabilities {
        node_name: String,
        channel_key: ChannelKey,
        need: InputNeed,
    },
    /// Two suppliers provide the same channel: two nodes, or a node and the
    /// body, or a node and an outside input. Each channel has exactly one
    /// supplier, so every read has one writer.
    MultipleSuppliers {
        channel: ChannelKey,
        first: Writer,
        second: Writer,
    },
    /// Two or more nodes share a name. Errors, logs and node order within a
    /// level all identify a node by its name, so names must be unique.
    DuplicateNodeName { name: String },
    /// A unit the body declares is not one level: empty, or holding something
    /// other than ASCII letters, digits and `_`. Errors and the channel
    /// catalog name a body channel's writer by its unit, so it follows the
    /// node-name grammar. Reported once per unit.
    MalformedUnitName { unit: String },
    /// The body states a mount for a channel that is not a frame on this
    /// agent's body off its spine, with a well-formed leaf. A mount is where
    /// an installed part measures. Reported once per channel.
    RefusedBodyMount {
        unit: String,
        channel: ChannelKey,
        mount: FrameId,
        refusal: MountRefusal,
    },
    /// A node mounts an output at the mount of a channel it does not read in
    /// the same tick, so no mount could be resolved for it before the node is
    /// built.
    MountFromUnreadInput {
        node_name: String,
        output: ChannelKey,
        input: ChannelKey,
    },
    /// A node's name is not one level: empty, or holding something other
    /// than ASCII letters, digits and `_`. The name fills one part of every
    /// watcher path under the node (`agent.<agent>.node.<node>.<leaf>`), and
    /// `.` separates the parts, so such a path could not be split back.
    MalformedNodeName { name: String },
    /// A channel's name is not one or more levels joined by `.`: it is empty
    /// (an unnamed channel), has an empty level (`gps.`, `a..b`), or holds
    /// something other than ASCII letters, digits and `_` (`oracle/pose`,
    /// `gps-fix`). The name is the channel's address in every watcher path,
    /// recording and bridge, so each must split back into its levels.
    /// Reported once per name, with every channel under it.
    MalformedChannelName {
        name: String,
        uses: Vec<ChannelUses>,
    },
    /// One name names more than one channel on the agent: the channels
    /// differ in kind or payload type. A name is the one address of a
    /// channel, so it names exactly one. Often a reader and a writer that
    /// disagree on the type, which `uses` shows side by side.
    ChannelNameReused {
        name: String,
        uses: Vec<ChannelUses>,
    },
    /// A node declares a watchable leaf that isn't one or more levels joined
    /// by `.`: it is empty, has an empty level (`.nis`, `nis.`,
    /// `aiding..nis`), or holds something other than ASCII letters, digits
    /// and `_`. Paths split on `.`, so such a leaf would mis-split in a glob
    /// or a tree-shaped sink.
    MalformedObservable { node_name: String, leaf: String },
    /// A node declares a watchable leaf in the pipeline's own group
    /// ([`PIPELINE_LEAF_GROUP`]), which holds the leaves the pipeline adds to
    /// every node. Refused even when the pipeline doesn't add that leaf yet,
    /// so adding one later never breaks a node.
    ReservedObservable { node_name: String, leaf: String },
    /// A node's output channel watches at a path in the pipeline's own group
    /// ([`PIPELINE_LEAF_GROUP`]), such as a channel named `tick.duration`.
    /// Refused like a declared leaf there, so the pipeline's leaves never
    /// share a path with a channel.
    ReservedOutputPath {
        node_name: String,
        channel: ChannelKey,
        segment: String,
    },
    /// A node declares the same watchable leaf more than once, so a
    /// watcher's path would point at two values. Reported once per leaf per
    /// node.
    DuplicateObservable { node_name: String, leaf: String },
    /// A node declares a leaf that has the same path as one of its own output
    /// channels: under `agent.<agent>.node.<node>`, `leaf` and the channel's
    /// name are the same string, so a watcher or an assertion target
    /// would name two things. Only an exact match is refused; a shared
    /// dotted prefix (`estimate` beside `estimate.cov`) is a group.
    ObservableCollidesWithOutput {
        node_name: String,
        leaf: String,
        channel: ChannelKey,
    },
    /// The level sort stopped with nodes left but no loop was found among
    /// them. Wiring and ordering together rule this out, so it means a bug in
    /// the build, not a mistake in the pipeline's configuration. `nodes`
    /// names the nodes left, in no particular order.
    StuckWithoutCycle { nodes: Vec<String> },
    /// A planned node failed to build once wiring and ordering passed.
    /// `error` is the constructor's own and is displayed alone, so it names
    /// the node: a factory's error names the node's config entry.
    ConstructionFailed {
        node_name: String,
        error: Box<dyn Error + Send + Sync>,
    },
    /// A planned node was built under a name other than its plan's. The plan's
    /// name is the one wiring checked, so the node is refused.
    ConstructedNodeMisnamed {
        node_name: String,
        built_name: String,
    },
}

// default source() returns None which is okay
impl Error for PipelineBuildError {}

impl std::fmt::Display for PipelineBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PipelineBuildError::Cycle {
                participants,
                edges,
            } => {
                write!(
                    f,
                    "nodes [{}] wait on each other's outputs in the same tick, so none can \
                     run first:",
                    participants.join(", ")
                )?;
                for edge in edges {
                    write!(f, " {edge};")?;
                }
                Ok(())
            }
            PipelineBuildError::UnsatisfiedInput {
                node_name,
                channel,
                need,
            } => {
                let need = need_label(*need);
                match channel.kind() {
                    ChannelKind::Sensor => write!(
                        f,
                        "node \"{node_name}\" reads channel {channel} ({need}) but nothing \
                         supplies it: the body does not publish it and no node writes it"
                    ),
                    _ => write!(
                        f,
                        "node \"{node_name}\" reads channel {channel} ({need}) but nothing \
                         supplies it: no node writes it, the body does not publish it, and it is \
                         not declared as an outside input"
                    ),
                }
            }
            PipelineBuildError::UnsatisfiedBodyCapabilities {
                node_name,
                channel_key,
                need,
            } => {
                let need = need_label(*need);
                write!(
                    f,
                    "node \"{node_name}\" reads channel {channel_key} ({need}); the body does not \
                     publish it"
                )
            }
            PipelineBuildError::MultipleSuppliers {
                channel,
                first,
                second,
            } => {
                write!(
                    f,
                    "channel {channel} is supplied by both {first} and {second} — each channel \
                     has exactly one supplier"
                )
            }
            PipelineBuildError::DuplicateNodeName { name } => {
                write!(
                    f,
                    "more than one node is named \"{name}\" — node names must be unique"
                )
            }
            PipelineBuildError::MalformedUnitName { unit } => {
                write!(
                    f,
                    "the body declares unit \"{unit}\", which is not one level — a unit name \
                     is non-empty and holds only letters, digits and '_'"
                )
            }
            PipelineBuildError::RefusedBodyMount {
                unit,
                channel,
                mount,
                refusal,
            } => {
                write!(
                    f,
                    "unit \"{unit}\" states channel {channel} is mounted at {mount}, {refusal}"
                )
            }
            PipelineBuildError::MountFromUnreadInput {
                node_name,
                output,
                input,
            } => {
                write!(
                    f,
                    "node \"{node_name}\" mounts output {output} at the mount of {input}, which \
                     it does not read in the same tick"
                )
            }
            PipelineBuildError::MalformedNodeName { name } => {
                write!(
                    f,
                    "node name \"{name}\" is not one level — a node name is one part of a \
                     watcher path, so it is non-empty and holds only letters, digits and '_'"
                )
            }
            PipelineBuildError::MalformedChannelName { name, uses } => {
                write!(
                    f,
                    "channel name \"{name}\" is not one or more levels joined by '.', each \
                     only letters, digits and '_':"
                )?;
                write_uses(f, uses)
            }
            PipelineBuildError::ChannelNameReused { name, uses } => {
                write!(
                    f,
                    "channel name \"{name}\" names {} channels — a name names one channel \
                     per agent:",
                    uses.len()
                )?;
                write_uses(f, uses)
            }
            PipelineBuildError::MalformedObservable { node_name, leaf } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\", which is not a \
                     path — a leaf is one or more levels joined by '.', each only letters, \
                     digits and '_'"
                )
            }
            PipelineBuildError::ReservedObservable { node_name, leaf } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\" in the \
                     \"{PIPELINE_LEAF_GROUP}\" group, which the pipeline owns and adds to every \
                     node — use another name"
                )
            }
            PipelineBuildError::ReservedOutputPath {
                node_name,
                channel,
                segment,
            } => {
                write!(
                    f,
                    "node \"{node_name}\" writes channel {channel}, which watchers would ask \
                     for at \"{segment}\", in the \"{PIPELINE_LEAF_GROUP}\" group the pipeline \
                     owns — rename the channel"
                )
            }
            PipelineBuildError::DuplicateObservable { node_name, leaf } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\" more than once — \
                     each leaf names one value"
                )
            }
            PipelineBuildError::ObservableCollidesWithOutput {
                node_name,
                leaf,
                channel,
            } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\", which is also \
                     the path of its output channel {channel} — rename the leaf or the channel"
                )
            }
            PipelineBuildError::StuckWithoutCycle { nodes } => {
                write!(
                    f,
                    "internal error: ordering stopped with nodes left but found no cycle \
                     among them: [{}]",
                    nodes.join(", ")
                )
            }
            PipelineBuildError::ConstructionFailed { error, .. } => write!(f, "{error}"),
            PipelineBuildError::ConstructedNodeMisnamed {
                node_name,
                built_name,
            } => {
                write!(
                    f,
                    "node \"{node_name}\" was planned under that name but built as \
                     \"{built_name}\"; a node keeps the name its plan declared"
                )
            }
        }
    }
}

/// Why the build refuses a mount the body states, as reported in
/// [`PipelineBuildError::RefusedBodyMount`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountRefusal {
    /// The frame belongs to another agent.
    OtherAgent,
    /// The frame belongs to no agent.
    Global,
    /// The frame is the agent's `map`, `odom` or `base_link`, not an
    /// installed part's own.
    Spine,
    /// The frame's leaf is not levels joined by `.`.
    MalformedLeaf,
}

impl std::fmt::Display for MountRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MountRefusal::OtherAgent => write!(
                f,
                "a frame of another agent — a body mounts its channels on its own frames"
            ),
            MountRefusal::Global => write!(
                f,
                "a global frame — a body mounts its channels on its own frames"
            ),
            MountRefusal::Spine => write!(
                f,
                "a frame of the agent's spine — a mount is the installed part's own frame"
            ),
            MountRefusal::MalformedLeaf => write!(
                f,
                "whose leaf is not one or more levels joined by '.', each only letters, digits \
                 and '_'"
            ),
        }
    }
}

/// One channel under a name, with everything that writes it and every node
/// that reads it, as listed in [`PipelineBuildError::MalformedChannelName`]
/// and [`PipelineBuildError::ChannelNameReused`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelUses {
    pub key: ChannelKey,
    /// Each writer of `key`, in the order the build met them.
    pub writers: Vec<Writer>,
    /// The name of each node that reads `key`.
    pub readers: Vec<String>,
}

impl std::fmt::Display for ChannelUses {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.key)?;
        if !self.writers.is_empty() {
            let writers: Vec<String> = self.writers.iter().map(ToString::to_string).collect();
            write!(f, " written by {}", writers.join(", "))?;
        }
        if !self.readers.is_empty() {
            let readers: Vec<String> = self
                .readers
                .iter()
                .map(|reader| format!("node \"{reader}\""))
                .collect();
            write!(f, " read by {}", readers.join(", "))?;
        }
        Ok(())
    }
}

/// One same-tick read inside a dependency loop, as listed in a
/// [`PipelineBuildError::Cycle`] error: `consumer` reads `channel`, which
/// `producer` writes, so `consumer` must run after `producer`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CycleEdge {
    /// The node that writes `channel`.
    pub producer: String,
    /// The node that reads `channel` in the same tick.
    pub consumer: String,
    /// The channel read.
    pub channel: ChannelKey,
}

impl std::fmt::Display for CycleEdge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "\"{}\" reads {} from \"{}\"",
            self.consumer, self.channel, self.producer
        )
    }
}

/// Each of `uses` after an error's opening sentence, one per channel.
fn write_uses(f: &mut std::fmt::Formatter<'_>, uses: &[ChannelUses]) -> std::fmt::Result {
    for channel in uses {
        write!(f, " {channel};")?;
    }
    Ok(())
}

/// The word an error message uses for an input's need.
fn need_label(need: InputNeed) -> &'static str {
    match need {
        InputNeed::Required => "required",
        InputNeed::Optional => "optional",
    }
}
