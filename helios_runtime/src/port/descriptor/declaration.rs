//! [`PortDescriptor`]: what a node declares it reads from and writes to the
//! bus, with each input as an [`InputPort`] record of channel, need, timing
//! and, on a keyed channel, the instances it reads,
//! and each output as an [`OutputPort`] record of channel, the actuators it
//! drives and the [`MountRule`] that gives its mount. It also
//! declares what the node can emit for watchers, each value an [`Observable`]
//! record of leaf name, [`Determinism`] and [`ObservableValue`] (a number
//! in its [`SiUnit`], or a channel's payload type), and the
//! frames the node defines, each [`Pin`] placing one of them in the world an
//! input names, and the [`NodeClass`] the node runs under.
//!
//! The build reads these declarations to allocate bus slots, order nodes, and
//! check that every channel has exactly one producer and that the node's class
//! may read and write each channel's kind. Outside this crate a descriptor is
//! built only through [`PortDescriptorBuilder`](crate::port::PortDescriptorBuilder).

use crate::{
    observe::observation::ObservedValueKind,
    port::{
        channel::ChannelKey,
        keyed::{DeclaredInstances, InstanceKey},
        kind::NodeClass,
    },
};

use helios_core::control::actuators::ActuatorDrive;
use helios_core::kernel::si_unit::SiUnit;
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

use std::{any::TypeId, sync::Arc};

/// Declares what a pipeline node reads from and writes to the bus.
///
/// Every input, required or optional, must have a supplier (a node output, a
/// body channel, or a declared outside input) for `PipelineBuilder::build()`
/// to succeed. Every same-tick input, required or optional, also orders the
/// node after its supplier. This does NOT guarantee a value is
/// present at runtime — cold-start, sensor dropout, and rate-gated upstream
/// nodes mean every consumer must handle `None`. The standard pattern is an
/// early-return.
///
/// `optional_inputs` channels are consumed if present; the node runs without
/// them.
///
/// No two nodes may declare the same `outputs` channel — enforced at build time.
///
/// Each input is stored as an [`InputPort`] record carrying its channel, its
/// [`InputNeed`] and its [`InputTiming`], and, on a keyed channel, the
/// instances the node reads by name. [`inputs`](Self::inputs) yields the
/// records; [`required_inputs`](Self::required_inputs) and
/// [`optional_inputs`](Self::optional_inputs) yield just the channel keys.
///
/// Each output is stored as an [`OutputPort`] record. An `ActuatorCommand`
/// output also carries the actuators it drives, read with
/// [`drives`](Self::drives), so the actuator seam can check them against the
/// body without knowing what kind of node wrote them.
/// [`output_ports`](Self::output_ports) yields the records;
/// [`outputs`](Self::outputs) yields just the channel keys.
///
/// Each input whose mount the node resolves through the tf tree is declared
/// as a [`MountReading`], read with [`mount_readings`](Self::mount_readings),
/// so a host can check the tree holds that frame in the axes the node reads
/// it in before the first tick.
///
/// Each frame the node defines, one whose meaning it alone gives (the root an
/// estimator holds its state in), is listed with its axis convention in
/// [`defined_frames`](Self::defined_frames), so the build can refuse a frame
/// with two owners and give each frame its owner's convention. Each input
/// that places one of those frames in the world frame its values are
/// relative to is declared as a [`Pin`], read with [`pins`](Self::pins).
///
/// Each value the node can emit for watchers is declared as an [`Observable`],
/// read with [`observables`](Self::observables). These never touch the bus:
/// they are the node's catalog of watchable values, and an emit of a leaf the
/// node didn't declare is dropped.
///
/// The node's [`NodeClass`], read with [`class`](Self::class), decides which
/// channel kinds it may read and write; the build refuses any other.
///
/// Fields are private and read through accessors, so the shape of a declared
/// input can change without touching every node. Outside this crate the only
/// way to construct one is
/// [`PortDescriptorBuilder`](crate::port::PortDescriptorBuilder).
#[derive(Debug)]
pub struct PortDescriptor {
    class: NodeClass,
    required_inputs: Vec<InputPort>,
    optional_inputs: Vec<InputPort>,
    mount_readings: Vec<MountReading>,
    outputs: Vec<OutputPort>,
    observables: Vec<Observable>,
    defined_frames: Vec<(FrameId, Convention)>,
    pins: Vec<Pin>,
    rate: Option<f64>,
}

impl PortDescriptor {
    /// Raw constructor for an algorithm node. Node descriptors go through the
    /// builder; this is for the builder itself and for in-crate tests that
    /// only need a bus with slots for a given set of channels.
    ///
    /// Every input is recorded as [`InputTiming::SameTick`]; the need comes
    /// from which list the key arrives in. Every output is recorded as driving
    /// no actuators.
    pub(crate) fn new(
        required_inputs: Vec<ChannelKey>,
        optional_inputs: Vec<ChannelKey>,
        outputs: Vec<ChannelKey>,
        rate: Option<f64>,
    ) -> Self {
        let required = required_inputs
            .into_iter()
            .map(|r| InputPort::new(r, InputNeed::Required, InputTiming::SameTick))
            .collect();

        let optional = optional_inputs
            .into_iter()
            .map(|o| InputPort::new(o, InputNeed::Optional, InputTiming::SameTick))
            .collect();

        Self {
            class: NodeClass::Algorithm,
            required_inputs: required,
            optional_inputs: optional,
            mount_readings: Vec::new(),
            outputs: outputs.into_iter().map(OutputPort::undriven).collect(),
            observables: Vec::new(),
            defined_frames: Vec::new(),
            pins: Vec::new(),
            rate,
        }
    }

    /// Records the class the node runs under. For the builder.
    pub(crate) fn with_class(mut self, class: NodeClass) -> Self {
        self.class = class;
        self
    }

    /// Replaces the outputs with full records, drives included. For the
    /// builder, which checks each command channel's type before recording it,
    /// after the inputs.
    ///
    /// # Panics
    ///
    /// If an output forwards a channel that is not a declared input, or one
    /// of another type: a node forwarding what it never reads is a bug in the
    /// node.
    pub(crate) fn with_outputs(mut self, outputs: Vec<OutputPort>) -> Self {
        for port in &outputs {
            if let Some(input) = &port.forwards {
                assert!(
                    self.inputs().any(|read| read.channel == *input),
                    "{} forwards {input}, which the node does not read",
                    port.channel
                );
                assert!(
                    input.type_id() == port.channel.type_id(),
                    "{} forwards {input}, which carries another type",
                    port.channel
                );
            }
        }
        self.outputs = outputs;
        self
    }

    /// Records the values the node can emit for watchers, in declaration
    /// order. For the builder.
    pub(crate) fn with_observables(mut self, observables: Vec<Observable>) -> Self {
        self.observables = observables;
        self
    }

    /// Records the frames the node defines. For the builder.
    pub(crate) fn with_defined_frames(mut self, frames: Vec<(FrameId, Convention)>) -> Self {
        self.defined_frames = frames;
        self
    }

    /// Records the axes the node reads each listed input's mount in. For the
    /// builder.
    ///
    /// # Panics
    ///
    /// If a listed channel is not a declared input: a node stating how it
    /// reads the mount of a channel it never reads is a bug in the node.
    pub(crate) fn with_mount_readings(mut self, readings: Vec<(ChannelKey, Convention)>) -> Self {
        for (channel, convention) in readings {
            assert!(
                self.inputs().any(|input| input.channel == channel),
                "a mount reading is declared for {channel}, which the node does not read"
            );
            self.mount_readings.push(MountReading {
                channel,
                convention,
            });
        }
        self
    }

    /// Records the instances the node reads of each listed keyed input, each
    /// once. For the builder, after the inputs.
    ///
    /// # Panics
    ///
    /// If a listed channel is not a declared input: a node naming what it
    /// reads of a channel it never reads is a bug in the node.
    pub(crate) fn with_instance_readings(
        mut self,
        readings: Vec<(ChannelKey, InstanceKey)>,
    ) -> Self {
        for (channel, instance) in readings {
            let mut read = false;
            for input in self
                .required_inputs
                .iter_mut()
                .chain(&mut self.optional_inputs)
                .filter(|input| input.channel == channel)
            {
                read = true;
                if !input.instances.contains(&instance) {
                    input.instances.push(instance.clone());
                }
            }
            assert!(
                read,
                "instance \"{instance}\" of {channel} is named, but the node does not read \
                 that channel"
            );
        }
        self
    }

    /// Records the inputs that pin the frames the node defines. For the
    /// builder, after the inputs, the instances they read and the defined
    /// frames.
    ///
    /// # Panics
    ///
    /// If a pin names a frame the node doesn't define, a channel it doesn't
    /// read, or an instance it doesn't read: each is a bug in the node.
    pub(crate) fn with_pins(mut self, pins: Vec<Pin>) -> Self {
        for pin in &pins {
            assert!(
                self.defined_frames
                    .iter()
                    .any(|(frame, _)| *frame == pin.frame),
                "{} is pinned, but the node does not define it",
                pin.frame
            );
            assert!(
                self.inputs().any(|input| input.channel == pin.channel),
                "a pin is declared through {}, which the node does not read",
                pin.channel
            );
            if let Some(instance) = &pin.instance {
                assert!(
                    self.inputs()
                        .any(|input| input.channel == pin.channel
                            && input.instances.contains(instance)),
                    "a pin is declared through instance \"{instance}\" of {}, which the node \
                     does not read",
                    pin.channel
                );
            }
        }
        self.pins = pins;
        self
    }

    /// The class the node runs under, which decides the channel kinds it may
    /// read and write.
    pub fn class(&self) -> NodeClass {
        self.class
    }

    /// Every declared input as a full record, required inputs first, then
    /// optional, each in declaration order.
    pub fn inputs(&self) -> impl Iterator<Item = &InputPort> {
        self.required_inputs.iter().chain(&self.optional_inputs)
    }

    /// Channels the node cannot run without, in declaration order. Need does
    /// not decide ordering; [`same_tick_inputs`](Self::same_tick_inputs) does.
    pub fn required_inputs(&self) -> impl Iterator<Item = &ChannelKey> {
        self.required_inputs.iter().map(|i| &i.channel)
    }

    /// Channels the node uses if a value is present, in declaration order. The
    /// node runs without a value, but the build still requires each channel to
    /// have a supplier.
    pub fn optional_inputs(&self) -> impl Iterator<Item = &ChannelKey> {
        self.optional_inputs.iter().map(|i| &i.channel)
    }

    /// Channels whose value must be written this tick before the node reads
    /// it, required and optional alike, in [`inputs`](Self::inputs) order. The
    /// build places the node in a later level than each channel's producing
    /// node. An [`InputTiming::PreviousTick`] input is left out, which is how a
    /// loop is broken.
    pub fn same_tick_inputs(&self) -> impl Iterator<Item = &ChannelKey> {
        self.inputs().filter_map(|i| {
            if i.timing == InputTiming::SameTick {
                Some(&i.channel)
            } else {
                None
            }
        })
    }

    /// The inputs whose mounts the node resolves through the tf tree, each
    /// with the axes it reads the mount in, in declaration order. One channel
    /// may appear more than once, read by two parts of the node.
    pub fn mount_readings(&self) -> &[MountReading] {
        &self.mount_readings
    }

    /// Every output record, in declaration order.
    pub fn output_ports(&self) -> &[OutputPort] {
        &self.outputs
    }

    /// Channels this node writes when it executes.
    pub fn outputs(&self) -> impl Iterator<Item = &ChannelKey> {
        self.outputs.iter().map(|o| &o.channel)
    }

    /// The actuators `output` drives, as the node declared with its
    /// `ActuatorCommand` output. Empty for an output that declared none,
    /// including every output of another type.
    pub fn drives(&self, output: &ChannelKey) -> &[ActuatorDrive] {
        self.outputs
            .iter()
            .find(|o| &o.channel == output)
            .map_or(&[], |o| o.drives.as_slice())
    }

    /// The values this node declares it can emit for watchers, in declaration
    /// order. Empty for a node that declares none.
    pub fn observables(&self) -> &[Observable] {
        &self.observables
    }

    /// The frames this node defines, each with the axis convention it gives
    /// it, in declaration order. Empty for a node that defines none.
    pub fn defined_frames(&self) -> &[(FrameId, Convention)] {
        &self.defined_frames
    }

    /// The inputs that place a frame this node defines in the world frame
    /// their values are relative to, in declaration order. Empty for a node
    /// that pins nothing.
    pub fn pins(&self) -> &[Pin] {
        &self.pins
    }

    /// Execution rate in Hz. `None` means every tick.
    pub fn rate(&self) -> Option<f64> {
        self.rate
    }
}

/// One declared input of a node: which channel it reads, whether the node
/// can run without it, which tick's value it expects, and, on a keyed
/// channel, the instances it reads by name. An input naming none reads
/// whatever instances there are, as a mux over every keyed sensor would.
///
/// Need and timing are independent — an optional input is not implicitly
/// delayed, and a delayed input is not implicitly optional. Only the
/// descriptor builder creates records.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct InputPort {
    channel: ChannelKey,
    need: InputNeed,
    timing: InputTiming,
    instances: Vec<InstanceKey>,
}

impl InputPort {
    pub(crate) fn new(channel: ChannelKey, need: InputNeed, timing: InputTiming) -> Self {
        Self {
            channel,
            need,
            timing,
            instances: Vec::new(),
        }
    }

    /// The channel this input reads.
    pub fn channel(&self) -> &ChannelKey {
        &self.channel
    }

    /// Whether the node can run without a value on this input.
    pub fn need(&self) -> InputNeed {
        self.need
    }

    /// Which tick's value this input expects.
    pub fn timing(&self) -> InputTiming {
        self.timing
    }

    /// The instances of its keyed channel the node reads by name, in
    /// declaration order. The build refuses one no writer declares. Empty for
    /// a channel that holds one value, and for a node reading every instance.
    pub fn instances(&self) -> &[InstanceKey] {
        &self.instances
    }
}

/// A node's statement that it resolves an input channel's mount through the
/// tf tree, reading the frame in the axes `convention` names. Only the
/// descriptor builder creates records, for channels the node reads.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MountReading {
    channel: ChannelKey,
    convention: Convention,
}

impl MountReading {
    /// The input channel whose mount is read.
    pub fn channel(&self) -> &ChannelKey {
        &self.channel
    }

    /// The axes the node reads the mount in.
    pub fn convention(&self) -> Convention {
        self.convention
    }
}

/// A node's statement that input `channel` places `what` of `frame`, a frame
/// the node defines, in the world frame the input's values are relative to:
/// a GPS fix pins the position of an estimator's root to the datum it is
/// given in. On a keyed channel the pin reads one `instance`, whose values
/// are expressed in that world frame. Only the descriptor builder creates
/// records, for channels the node reads.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pin {
    frame: FrameId,
    what: Pinned,
    channel: ChannelKey,
    instance: Option<InstanceKey>,
}

impl Pin {
    pub(crate) fn new(
        frame: FrameId,
        what: Pinned,
        channel: ChannelKey,
        instance: Option<InstanceKey>,
    ) -> Self {
        Self {
            frame,
            what,
            channel,
            instance,
        }
    }

    /// The frame the node defines that the input places.
    pub fn frame(&self) -> &FrameId {
        &self.frame
    }

    /// What of the frame the input places.
    pub fn what(&self) -> Pinned {
        self.what
    }

    /// The input channel that places it.
    pub fn channel(&self) -> &ChannelKey {
        &self.channel
    }

    /// The instance read, on a keyed channel; `None` on a channel that holds
    /// one value.
    pub fn instance(&self) -> Option<&InstanceKey> {
        self.instance.as_ref()
    }
}

/// What of a frame an input pins in the world. A frame is anchored only when
/// its position is pinned; a heading or an altitude alone leaves it free to
/// move, so each is its own variant when its first model arrives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Pinned {
    /// The frame's origin: a position measured relative to the world frame.
    Position,
}

/// One declared output of a node: which channel it writes, the actuators its
/// commands drive when the channel carries `ActuatorCommand`, the rule that
/// gives the channel's mount, the instances it writes when the channel is
/// keyed, and the input whose values it writes unchanged, if it forwards one.
///
/// Only the descriptor builder creates records, so only an `ActuatorCommand`
/// output carries drives and only a keyed output carries instances.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutputPort {
    channel: ChannelKey,
    drives: Vec<ActuatorDrive>,
    mount: MountRule,
    instances: DeclaredInstances,
    forwards: Option<ChannelKey>,
}

impl OutputPort {
    /// An output with no mount.
    pub(crate) fn new(channel: ChannelKey, drives: Vec<ActuatorDrive>) -> Self {
        Self {
            channel,
            drives,
            mount: MountRule::None,
            instances: DeclaredInstances::default(),
            forwards: None,
        }
    }

    /// An output that drives no actuators: every output not of
    /// `ActuatorCommand` type.
    pub(crate) fn undriven(channel: ChannelKey) -> Self {
        Self::new(channel, Vec::new())
    }

    /// The channel this output writes.
    pub fn channel(&self) -> &ChannelKey {
        &self.channel
    }

    /// The output with its mount given by `rule`.
    pub(crate) fn with_mount(mut self, rule: MountRule) -> Self {
        self.mount = rule;
        self
    }

    /// The output writing `instances` of its keyed channel.
    pub(crate) fn with_instances(mut self, instances: DeclaredInstances) -> Self {
        self.instances = instances;
        self
    }

    /// The output writing each value of `input` unchanged.
    pub(crate) fn forwarding(mut self, input: ChannelKey) -> Self {
        self.forwards = Some(input);
        self
    }

    /// The actuators this output's commands drive. Empty unless the channel
    /// carries `ActuatorCommand`.
    pub fn drives(&self) -> &[ActuatorDrive] {
        &self.drives
    }

    /// The rule that gives this output's mount.
    pub fn mount(&self) -> &MountRule {
        &self.mount
    }

    /// The instances of its keyed channel this output writes, in declaration
    /// order. Empty for a channel that holds one value.
    pub fn instances(&self) -> &[InstanceKey] {
        self.instances.keys()
    }

    /// The instances as declared, each with the frame its values are
    /// expressed in when the payload names one.
    pub fn declared_instances(&self) -> &DeclaredInstances {
        &self.instances
    }

    /// The input whose values this output writes unchanged, if it forwards
    /// one, so whatever describes the input's values describes its own.
    pub fn forwards(&self) -> Option<&ChannelKey> {
        self.forwards.as_ref()
    }
}

/// Where a node output's mount, the frame its values are measured in, comes
/// from. The node states the rule; the builder resolves it once the graph is
/// ordered, so the node never needs to know the frame itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MountRule {
    /// The output has no mount: its values carry their frame, or have none
    /// (a state, a path, a map, a command).
    None,
    /// The output is measured in the mount of one of the node's same-tick
    /// inputs: a cloud flattened from a range field, a de-skewed scan. The
    /// build refuses a rule naming a channel the node does not read so.
    FromInput(ChannelKey),
}

/// Whether a node can run without a value on an input.
///
/// This is about the node, not the wiring: either way the channel names
/// something that must exist. A sensor dropout is a runtime `None` the node
/// handles; it is never a reason to skip checking that the channel is wired.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InputNeed {
    /// The node does nothing useful without it and early-returns on `None`.
    Required,
    /// The node uses it when present and carries on without it.
    Optional,
}

/// Which tick's value an input expects to read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InputTiming {
    /// The value written this tick: the producer must run before the reader.
    /// Feedforward and feedback summed together are both same-tick, so they
    /// describe the same instant.
    SameTick,
    /// The value as it stood at the start of the tick, wherever the producer
    /// runs. This is how a loop is broken: the delayed edge is not an ordering
    /// constraint. No builder declares it yet, because reads still go to the
    /// live slot and would not actually see the start-of-tick value.
    PreviousTick,
}

/// One value a watcher can ask for: its name, whether it is
/// [deterministic](Determinism), and [what it arrives as](ObservableValue).
///
/// A node declares its leaves as observables, named relative to the node
/// (`aiding.gps.nis`). The pipeline's catalog also lists each channel as
/// one, named by the channel. The name is a label: the catalog pairs each
/// record with the [`Subject`](crate::observe::key::Subject) that identifies
/// it. Only the descriptor builder and the pipeline's catalog create records.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Observable {
    name: Arc<str>,
    determinism: Determinism,
    value: ObservableValue,
}

impl Observable {
    /// A value that arrives as a number in `si_unit`: every leaf a node
    /// declares, and the pipeline's own measurements.
    pub(crate) fn scalar(
        name: impl Into<Arc<str>>,
        determinism: Determinism,
        si_unit: SiUnit,
    ) -> Self {
        Self {
            name: name.into(),
            determinism,
            value: ObservableValue::Scalar { si_unit },
        }
    }

    /// A value that arrives as what was written to `channel`.
    pub(crate) fn typed(
        name: impl Into<Arc<str>>,
        determinism: Determinism,
        channel: &ChannelKey,
    ) -> Self {
        Self {
            name: name.into(),
            determinism,
            value: ObservableValue::Typed {
                type_id: channel.type_id(),
                type_name: channel.type_name(),
            },
        }
    }

    /// The leaf a node emits this value under, relative to the node, or the
    /// channel's name for a channel.
    pub fn name(&self) -> &Arc<str> {
        &self.name
    }

    /// Whether the value depends only on the node's inputs, the tick's time
    /// and its seed.
    pub fn determinism(&self) -> Determinism {
        self.determinism
    }

    /// What the value arrives as: a number in its unit, or a payload.
    pub fn value(&self) -> ObservableValue {
        self.value
    }

    /// The kind of [`ObservedValue`](crate::observe::observation::ObservedValue)
    /// this leaf arrives as.
    pub fn kind(&self) -> ObservedValueKind {
        self.value.kind()
    }
}

/// What an [`Observable`]'s values arrive as. A number has one unit; a
/// channel's payload has none of its own, since it may hold no number or
/// several in different units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ObservableValue {
    /// A number measured in `si_unit`: every leaf a node declares, and the
    /// pipeline's own measurements.
    Scalar { si_unit: SiUnit },
    /// What was written to a channel, a payload of this type.
    Typed {
        type_id: TypeId,
        type_name: &'static str,
    },
}

impl ObservableValue {
    /// The [`ObservedValueKind`] values of this arrive as: the same
    /// variants without the unit.
    pub fn kind(self) -> ObservedValueKind {
        match self {
            Self::Scalar { .. } => ObservedValueKind::Scalar,
            Self::Typed { type_id, type_name } => ObservedValueKind::Typed { type_id, type_name },
        }
    }
}

/// Whether a value depends only on its node's inputs, the tick's time and its
/// seed.
///
/// A promise about the node, not the run: a [`Deterministic`](Self::Deterministic)
/// node given the same inputs reports the same value, bit for bit. Whether a
/// host feeds it the same inputs twice is the host's own property. Only the
/// node knows which kind a value is, since both arrive as plain numbers.
///
/// A channel's values carry their writer's label: a node's outputs are
/// deterministic, the body's are deterministic while its provenance is exact
/// or simulated, and an outside input, which an operator may send, is not.
///
/// A sink comparing two runs (a regression check, a seeded Monte Carlo batch,
/// the everything-watched vs nothing-watched gate) expects every deterministic
/// value to match and skips every nondeterministic one. An assertion on a
/// nondeterministic value must say it accepts one.
///
/// Exhaustive on purpose: a new kind of value, such as one deterministic only
/// within a float tolerance (a parallel solver whose last bits vary, which a
/// comparison would check within that tolerance rather than skip), must make
/// every reader's `match` decide what to do with it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Determinism {
    /// Depends only on the node's inputs, the tick's time and its seed: NIS,
    /// a drop count, a channel the node writes. Seeded randomness counts,
    /// since the seed is an input.
    Deterministic,
    /// Also depends on something outside the node's inputs: wall-clock time
    /// or machine load (how long a node took to run), thread scheduling (a
    /// parallel sum with no fixed order), or hardware state. Differs between
    /// runs; a comparison skips it.
    Nondeterministic,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::channel::ChannelKey;

    fn ikey<T: 'static>() -> ChannelKey {
        ChannelKey::internal_of::<T>()
    }

    fn ikey_named<T: 'static>(name: &'static str) -> ChannelKey {
        ChannelKey::internal::<T>(name)
    }

    #[test]
    fn new_records_need_from_list_and_same_tick_timing() {
        let req = ikey::<u32>();
        let opt = ikey_named::<u32>("opt");
        let d = PortDescriptor::new(vec![req.clone()], vec![opt.clone()], vec![], None);

        let records: Vec<&InputPort> = d.inputs().collect();
        assert_eq!(
            records,
            vec![
                &InputPort::new(req, InputNeed::Required, InputTiming::SameTick),
                &InputPort::new(opt, InputNeed::Optional, InputTiming::SameTick),
            ]
        );
    }

    #[test]
    fn inputs_yields_required_then_optional_in_declaration_order() {
        let r1 = ikey_named::<u32>("r1");
        let r2 = ikey_named::<u32>("r2");
        let o1 = ikey_named::<u32>("o1");
        let o2 = ikey_named::<u32>("o2");
        let d = PortDescriptor::new(
            vec![r1.clone(), r2.clone()],
            vec![o1.clone(), o2.clone()],
            vec![],
            None,
        );

        let channels: Vec<&ChannelKey> = d.inputs().map(InputPort::channel).collect();
        assert_eq!(channels, vec![&r1, &r2, &o1, &o2]);
    }

    #[test]
    fn required_and_optional_accessors_split_by_need() {
        let req = ikey::<u32>();
        let opt = ikey_named::<u32>("opt");
        let d = PortDescriptor::new(vec![req.clone()], vec![opt.clone()], vec![], None);

        assert_eq!(d.required_inputs().collect::<Vec<_>>(), vec![&req]);
        assert_eq!(d.optional_inputs().collect::<Vec<_>>(), vec![&opt]);
    }

    #[test]
    fn same_tick_inputs_include_optional_inputs() {
        let req = ikey::<u32>();
        let opt = ikey_named::<u32>("opt");
        let d = PortDescriptor::new(vec![req.clone()], vec![opt.clone()], vec![], None);

        assert_eq!(d.same_tick_inputs().collect::<Vec<_>>(), vec![&req, &opt]);
    }

    #[test]
    fn same_tick_inputs_leave_out_previous_tick_inputs() {
        // No builder declares a previous-tick input yet, so build the record
        // directly: timing alone decides, whatever the need.
        let now = ikey_named::<u32>("now");
        let delayed_req = ikey_named::<u32>("delayed_req");
        let delayed_opt = ikey_named::<u32>("delayed_opt");
        let d = PortDescriptor {
            class: NodeClass::Algorithm,
            required_inputs: vec![
                InputPort::new(now.clone(), InputNeed::Required, InputTiming::SameTick),
                InputPort::new(delayed_req, InputNeed::Required, InputTiming::PreviousTick),
            ],
            optional_inputs: vec![InputPort::new(
                delayed_opt,
                InputNeed::Optional,
                InputTiming::PreviousTick,
            )],
            mount_readings: vec![],
            outputs: vec![],
            observables: vec![],
            defined_frames: vec![],
            pins: vec![],
            rate: None,
        };

        assert_eq!(d.same_tick_inputs().collect::<Vec<_>>(), vec![&now]);
    }

    #[test]
    fn new_declares_no_observables() {
        let d = PortDescriptor::new(vec![], vec![], vec![ikey::<u32>()], None);
        assert!(d.observables().is_empty());
    }

    #[test]
    fn with_observables_keeps_declaration_order() {
        let nis = Observable::scalar(
            "aiding.gps.nis",
            Determinism::Deterministic,
            SiUnit::Dimensionless,
        );
        let duration =
            Observable::scalar("duration", Determinism::Nondeterministic, SiUnit::Seconds);
        let d = PortDescriptor::new(vec![], vec![], vec![], None)
            .with_observables(vec![nis.clone(), duration.clone()]);

        assert_eq!(d.observables(), &[nis, duration]);
    }

    #[test]
    fn observable_reads_back_its_name_determinism_and_unit() {
        let o = Observable::scalar(
            "aiding.gps.nis",
            Determinism::Deterministic,
            SiUnit::Dimensionless,
        );
        assert_eq!(o.name().as_ref(), "aiding.gps.nis");
        assert_eq!(o.determinism(), Determinism::Deterministic);
        assert_eq!(
            o.value(),
            ObservableValue::Scalar {
                si_unit: SiUnit::Dimensionless
            }
        );
        assert_eq!(o.kind(), ObservedValueKind::Scalar);
    }

    #[test]
    fn a_typed_observable_carries_its_channels_payload_type_and_no_unit() {
        let o = Observable::typed("est.cov", Determinism::Deterministic, &ikey::<u32>());
        assert_eq!(
            o.kind(),
            ObservedValueKind::Typed {
                type_id: TypeId::of::<u32>(),
                type_name: std::any::type_name::<u32>(),
            }
        );
    }

    #[test]
    fn descriptor_without_inputs_yields_nothing() {
        let d = PortDescriptor::new(vec![], vec![], vec![ikey::<u32>()], None);
        assert!(d.inputs().next().is_none());
        assert!(d.required_inputs().next().is_none());
        assert!(d.optional_inputs().next().is_none());
    }
}
