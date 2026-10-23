//! [`PipelineBuildError`], everything [`build`](super::PipelineBuilder::build)
//! can reject, [`ChannelUses`], who writes and reads one channel,
//! [`CycleEdge`], one read inside a dependency loop, [`MountRefusal`], why
//! a mount the body states is refused, and [`FrameOwner`], who owns a frame.

use crate::{
    pipeline::autonomy_pipeline::PIPELINE_LEAF_GROUP,
    port::{ChannelKey, ChannelKind, InputNeed, InstanceKey, NodeClass, Writer},
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
    /// A node reads a measurement or internal channel that has no supplier:
    /// no node writes it, the body does not publish it, and it is not
    /// declared as an outside input. Raised for optional inputs too, since
    /// optional means the node runs without a value, not that the wiring is
    /// unchecked. For a measurement, often a misspelled name or a sensor
    /// missing from the body's fit.
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
    /// A writer of a keyed channel declares no instances, so every write it
    /// makes would be refused. Reported once per writer.
    KeyedWithoutInstances { channel: ChannelKey, writer: Writer },
    /// Two writers declare one instance of a keyed channel. Each instance has
    /// exactly one writer, as each single-value channel does.
    InstanceWrittenTwice {
        channel: ChannelKey,
        instance: InstanceKey,
        first: Writer,
        second: Writer,
    },
    /// One writer declares an instance of a keyed channel twice, so two
    /// declarations, as a transform's two parents, could disagree.
    InstanceDeclaredTwice {
        channel: ChannelKey,
        instance: InstanceKey,
        writer: Writer,
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
    /// agent's body other than `base_link`, with a well-formed leaf. A mount
    /// is where an installed part measures. Reported once per channel.
    RefusedBodyMount {
        unit: String,
        channel: ChannelKey,
        mount: FrameId,
        refusal: MountRefusal,
    },
    /// A body channel states a reference that is not one of the host's
    /// world frames. A reference is a world frame, and only the host knows
    /// where its world is. Reported once per channel.
    ReferenceNotWorldFrame {
        unit: String,
        channel: ChannelKey,
        reference: FrameId,
    },
    /// A node pins a frame it defines through an input whose writer states no
    /// world frame its values are relative to: a node output (a mux, a
    /// converter), a body channel about the body alone, or an instance
    /// expressed in a frame that isn't one of the host's world frames
    /// (`stated`). The frame would be placed nowhere. Reported once per pin
    /// whose input has a writer.
    PinWithoutReference {
        node_name: String,
        frame: FrameId,
        channel: ChannelKey,
        instance: Option<InstanceKey>,
        writer: Writer,
        stated: Option<FrameId>,
    },
    /// A node that defines a frame reads inputs relative to two or more
    /// world frames, so the frame can't be placed in one. `references` names
    /// each such input with its world frame, in input order. Reported once
    /// per node.
    ReferencesDisagree {
        node_name: String,
        references: Vec<(ChannelKey, FrameId)>,
    },
    /// A frame has two owners: two nodes define it, a node defines one the
    /// body owns (its `base_link`, a frame it mounts a channel at, or another
    /// of its frames) or the host owns (a world frame), or the host and the
    /// body both name it. A frame means what its one owner says it does, in
    /// the axes its owner gives it, so a pose held in it can't be two things.
    /// Reported once per extra owner.
    FrameDefinedTwice {
        frame: FrameId,
        first: FrameOwner,
        second: FrameOwner,
    },
    /// The body owns a frame, its `base_link` or a frame it mounts a channel
    /// at, but does not list it in
    /// [`BodyCapabilities::frames`](crate::BodyCapabilities::frames), so the
    /// frame has no axis convention. Reported once per frame.
    FrameWithoutConvention { frame: FrameId, owner: FrameOwner },
    /// A node reads a channel whose kind its class may not read, such as an
    /// algorithm reading truth. Isolating an algorithm under perfect upstream
    /// swaps the node upstream of it for a mock instead. Reported once per
    /// input.
    KindNotReadable {
        node_name: String,
        class: NodeClass,
        channel: ChannelKey,
    },
    /// A node writes a channel whose kind its class may not write: a node
    /// writing truth or health, which only a body knows, or a mock writing a
    /// measurement. Reported once per output.
    KindNotWritable {
        node_name: String,
        class: NodeClass,
        channel: ChannelKey,
    },
    /// A node mounts an output at the mount of a channel it does not read in
    /// the same tick, so no mount could be resolved for it before the node is
    /// built.
    MountFromUnreadInput {
        node_name: String,
        output: ChannelKey,
        input: ChannelKey,
    },
    /// A node resolves the mount of an input through the tf tree, but no
    /// writer of that input states a mount, so the node would look up
    /// whatever frame each value names, unchecked: the header check and the
    /// host's reach and convention checks pass over an unmounted channel.
    MountReadWithoutMount {
        node_name: String,
        channel: ChannelKey,
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
    /// A node declares the same watchable leaf more than once, so a
    /// watcher's path would point at two values. Reported once per leaf per
    /// node.
    DuplicateObservable { node_name: String, leaf: String },
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
                    ChannelKind::Measurement => write!(
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
            PipelineBuildError::KeyedWithoutInstances { channel, writer } => {
                write!(
                    f,
                    "{writer} writes keyed channel {channel} but declares no instances of it — \
                     a keyed output names the instances it writes"
                )
            }
            PipelineBuildError::InstanceWrittenTwice {
                channel,
                instance,
                first,
                second,
            } => {
                write!(
                    f,
                    "instance \"{instance}\" of channel {channel} is written by both {first} and \
                     {second} — each instance has exactly one writer"
                )
            }
            PipelineBuildError::InstanceDeclaredTwice {
                channel,
                instance,
                writer,
            } => {
                write!(
                    f,
                    "{writer} declares instance \"{instance}\" of channel {channel} twice — a \
                     writer declares each instance once"
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
            PipelineBuildError::ReferenceNotWorldFrame {
                unit,
                channel,
                reference,
            } => {
                write!(
                    f,
                    "unit \"{unit}\" states channel {channel} is expressed relative to \
                     {reference}, which is not one of the host's world frames; the host \
                     declares each frame its values name"
                )
            }
            PipelineBuildError::PinWithoutReference {
                node_name,
                frame,
                channel,
                instance,
                writer,
                stated,
            } => {
                write!(f, "node \"{node_name}\" pins {frame} through ")?;
                if let Some(instance) = instance {
                    write!(f, "instance \"{instance}\" of ")?;
                }
                write!(f, "{channel}, but {writer} ")?;
                match stated {
                    Some(stated) => write!(
                        f,
                        "states its values relative to {stated}, which is not one of the \
                         host's world frames"
                    )?,
                    None => write!(f, "states no world frame its values are relative to")?,
                }
                write!(
                    f,
                    "; a pinned frame is placed in the world frame its input names"
                )
            }
            PipelineBuildError::ReferencesDisagree {
                node_name,
                references,
            } => {
                write!(
                    f,
                    "node \"{node_name}\" defines a frame but reads inputs relative to \
                     different world frames:"
                )?;
                for (channel, reference) in references {
                    write!(f, " {channel} relative to {reference};")?;
                }
                write!(f, " a frame it defines is placed in one world frame")
            }
            PipelineBuildError::FrameDefinedTwice {
                frame,
                first,
                second,
            } => {
                write!(
                    f,
                    "frame {frame} is defined by {first} and by {second}; a frame has one owner"
                )
            }
            PipelineBuildError::FrameWithoutConvention { frame, owner } => {
                write!(
                    f,
                    "frame {frame} is owned by {owner}, but the body lists no axis convention \
                     for it; the body lists each of its frames with its convention"
                )
            }
            PipelineBuildError::KindNotReadable {
                node_name,
                class,
                channel,
            } => {
                write!(
                    f,
                    "{class} node \"{node_name}\" reads channel {channel}, but a {class} node \
                     may not read {kind} channels",
                    kind = channel.kind()
                )
            }
            PipelineBuildError::KindNotWritable {
                node_name,
                class,
                channel,
            } => {
                write!(
                    f,
                    "{class} node \"{node_name}\" writes channel {channel}, but a {class} node \
                     may not write {kind} channels",
                    kind = channel.kind()
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
            PipelineBuildError::MountReadWithoutMount { node_name, channel } => {
                write!(
                    f,
                    "node \"{node_name}\" reads the mount of {channel} through the tf tree, but \
                     no writer of {channel} states a mount"
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
            PipelineBuildError::DuplicateObservable { node_name, leaf } => {
                write!(
                    f,
                    "node \"{node_name}\" declares watchable leaf \"{leaf}\" more than once — \
                     each leaf names one value"
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
    /// The frame is the agent's `base_link`, not an installed part's own.
    BaseLink,
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
            MountRefusal::BaseLink => write!(
                f,
                "the agent's base_link — a mount is the installed part's own frame"
            ),
            MountRefusal::MalformedLeaf => write!(
                f,
                "whose leaf is not one or more levels joined by '.', each only letters, digits \
                 and '_'"
            ),
        }
    }
}

/// Who owns a frame, as reported in [`PipelineBuildError::FrameDefinedTwice`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameOwner {
    /// The body, as its `base_link` or another of its frames.
    Body,
    /// The body unit that mounts a channel at the frame.
    Unit { unit: String },
    /// The node that declares it defines the frame, such as an estimator
    /// holding its state in it.
    Node { node: String },
    /// The host, as one of its world frames.
    Host,
}

impl std::fmt::Display for FrameOwner {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FrameOwner::Body => write!(f, "the body"),
            FrameOwner::Unit { unit } => write!(f, "unit \"{unit}\""),
            FrameOwner::Node { node } => write!(f, "node \"{node}\""),
            FrameOwner::Host => write!(f, "the host"),
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
