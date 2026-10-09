//! [`PortDescriptor`]: what a node declares it reads from and writes to the
//! bus, with each input as an [`InputPort`] record of channel, need and timing,
//! and each `ActuatorCommand` output with the actuators it drives. It also
//! declares what the node can emit for watchers, each value an [`Observable`]
//! record of leaf name and [`Determinism`].
//!
//! The build reads these declarations to allocate bus slots, order nodes, and
//! check that every channel has exactly one producer. Outside this crate a
//! descriptor is built only through
//! [`AlgorithmNodePortDescriptor`](crate::port::AlgorithmNodePortDescriptor) or
//! [`MockNodePortDescriptor`](crate::port::MockNodePortDescriptor).

use crate::port::channel::ChannelKey;

use helios_core::control::actuators::ActuatorDrive;

use std::sync::Arc;

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
/// [`InputNeed`] and its [`InputTiming`]. [`inputs`](Self::inputs) yields the
/// records; [`required_inputs`](Self::required_inputs) and
/// [`optional_inputs`](Self::optional_inputs) yield just the channel keys.
///
/// An `ActuatorCommand` output also carries the actuators it drives, read
/// with [`drives`](Self::drives), so the actuator seam can check them against
/// the body without knowing what kind of node wrote them.
///
/// Each value the node can emit for watchers is declared as an [`Observable`],
/// read with [`observables`](Self::observables). These never touch the bus:
/// they are the node's catalog of watchable values, and an emit of a leaf the
/// node didn't declare is dropped.
///
/// Fields are private and read through accessors, so the shape of a declared
/// input can change without touching every node. Outside this crate the only
/// way to construct one is
/// [`AlgorithmNodePortDescriptor`](crate::port::AlgorithmNodePortDescriptor)
/// or [`MockNodePortDescriptor`](crate::port::MockNodePortDescriptor):
/// those builders are the kind fence that keeps oracle truth out of algorithm
/// nodes.
#[derive(Debug)]
pub struct PortDescriptor {
    required_inputs: Vec<InputPort>,
    optional_inputs: Vec<InputPort>,
    outputs: Vec<ChannelKey>,
    drives: Vec<(ChannelKey, Vec<ActuatorDrive>)>,
    observables: Vec<Observable>,
    rate: Option<f64>,
}

impl PortDescriptor {
    /// Raw constructor with no kind checks. Node descriptors go through the
    /// builders; this is for the builders themselves and for in-crate tests
    /// that only need a bus with slots for a given set of channels.
    ///
    /// Every input is recorded as [`InputTiming::SameTick`]; the need comes
    /// from which list the key arrives in.
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
            required_inputs: required,
            optional_inputs: optional,
            outputs,
            drives: Vec::new(),
            observables: Vec::new(),
            rate,
        }
    }

    /// Records the actuators each `ActuatorCommand` output drives. For the
    /// builders, which check each channel's type before recording it.
    pub(crate) fn with_drives(mut self, drives: Vec<(ChannelKey, Vec<ActuatorDrive>)>) -> Self {
        self.drives = drives;
        self
    }

    /// Records the values the node can emit for watchers, in declaration
    /// order. For the builders.
    pub(crate) fn with_observables(mut self, observables: Vec<Observable>) -> Self {
        self.observables = observables;
        self
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

    /// Channels this node writes when it executes.
    pub fn outputs(&self) -> &[ChannelKey] {
        &self.outputs
    }

    /// The actuators `output` drives, as the node declared with its
    /// `ActuatorCommand` output. Empty for an output that declared none,
    /// including every output of another type.
    pub fn drives(&self, output: &ChannelKey) -> &[ActuatorDrive] {
        self.drives
            .iter()
            .find(|(channel, _)| channel == output)
            .map_or(&[], |(_, drives)| drives.as_slice())
    }

    /// The values this node declares it can emit for watchers, in declaration
    /// order. Empty for a node that declares none.
    pub fn observables(&self) -> &[Observable] {
        &self.observables
    }

    /// Execution rate in Hz. `None` means every tick.
    pub fn rate(&self) -> Option<f64> {
        self.rate
    }
}

/// One declared input of a node: which channel it reads, whether the node
/// can run without it, and which tick's value it expects.
///
/// Need and timing are independent — an optional input is not implicitly
/// delayed, and a delayed input is not implicitly optional. Only the
/// descriptor builders create records, so the kind fence on channels holds.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct InputPort {
    channel: ChannelKey,
    need: InputNeed,
    timing: InputTiming,
}

impl InputPort {
    pub(crate) fn new(channel: ChannelKey, need: InputNeed, timing: InputTiming) -> Self {
        Self {
            channel,
            need,
            timing,
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

/// One value a node declares it can emit for watchers: its leaf name and
/// whether it repeats across runs.
///
/// The leaf name is relative to the node (`aiding.gps.nis`); the pipeline adds
/// the node's name and the host adds the agent. Only the descriptor builders
/// create records, so every declaration goes through one place.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Observable {
    leaf_name: Arc<str>,
    determinism: Determinism,
}

impl Observable {
    pub(crate) fn new(leaf_name: impl Into<Arc<str>>, determinism: Determinism) -> Self {
        Self {
            leaf_name: leaf_name.into(),
            determinism,
        }
    }

    /// The name the node emits this value under, relative to the node.
    pub fn leaf_name(&self) -> &Arc<str> {
        &self.leaf_name
    }

    /// Whether two runs with the same seed and inputs give the same samples.
    pub fn determinism(&self) -> Determinism {
        self.determinism
    }
}

/// Whether a declared value repeats exactly across runs.
///
/// A sink comparing two runs (a regression check, a seeded Monte Carlo batch,
/// the everything-watched vs nothing-watched gate) expects every
/// [`Reproducible`](Self::Reproducible) value to match bit for bit, and skips
/// [`WallClock`](Self::WallClock) values, which differ on every run. Only the
/// node knows which kind a value is, since both arrive as plain numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Determinism {
    /// The same on every run with the same seed and inputs: a statistic of the
    /// computation, such as NIS or a drop count.
    Reproducible,
    /// Depends on the machine and its load, such as how long a node took to
    /// run. Differs between runs; a comparison skips it.
    WallClock,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::channel::InternalChannel;

    fn ikey<T: 'static>() -> ChannelKey {
        InternalChannel::of::<T>().into()
    }

    fn ikey_named<T: 'static>(instance: &'static str) -> ChannelKey {
        InternalChannel::named::<T>(instance).into()
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
            required_inputs: vec![
                InputPort::new(now.clone(), InputNeed::Required, InputTiming::SameTick),
                InputPort::new(delayed_req, InputNeed::Required, InputTiming::PreviousTick),
            ],
            optional_inputs: vec![InputPort::new(
                delayed_opt,
                InputNeed::Optional,
                InputTiming::PreviousTick,
            )],
            outputs: vec![],
            drives: vec![],
            observables: vec![],
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
        let nis = Observable::new("aiding.gps.nis", Determinism::Reproducible);
        let duration = Observable::new("duration", Determinism::WallClock);
        let d = PortDescriptor::new(vec![], vec![], vec![], None)
            .with_observables(vec![nis.clone(), duration.clone()]);

        assert_eq!(d.observables(), &[nis, duration]);
    }

    #[test]
    fn observable_reads_back_its_name_and_determinism() {
        let o = Observable::new("aiding.gps.nis", Determinism::Reproducible);
        assert_eq!(o.leaf_name().as_ref(), "aiding.gps.nis");
        assert_eq!(o.determinism(), Determinism::Reproducible);
    }

    #[test]
    fn descriptor_without_inputs_yields_nothing() {
        let d = PortDescriptor::new(vec![], vec![], vec![ikey::<u32>()], None);
        assert!(d.inputs().next().is_none());
        assert!(d.required_inputs().next().is_none());
        assert!(d.optional_inputs().next().is_none());
    }
}
