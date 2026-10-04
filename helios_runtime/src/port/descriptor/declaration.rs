//! [`PortDescriptor`]: what a node declares it reads from and writes to the
//! bus, with each input as an [`InputPort`] record of channel, need and timing.
//!
//! The build reads these declarations to allocate bus slots, order nodes, and
//! check that every channel has exactly one producer. Outside this crate a
//! descriptor is built only through
//! [`AlgorithmNodePortDescriptor`](crate::port::AlgorithmNodePortDescriptor) or
//! [`MockNodePortDescriptor`](crate::port::MockNodePortDescriptor).

use crate::port::channel::ChannelKey;

/// Declares what a pipeline node reads from and writes to the bus.
///
/// Every input, required or optional, must have a supplier (a node output, a
/// body channel, or a declared outside input) for `PipelineBuilder::build()`
/// to succeed. `required_inputs` also order the node after its suppliers. This
/// does NOT guarantee a value is
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
            rate,
        }
    }

    /// Every declared input as a full record, required inputs first, then
    /// optional, each in declaration order.
    pub fn inputs(&self) -> impl Iterator<Item = &InputPort> {
        self.required_inputs.iter().chain(&self.optional_inputs)
    }

    /// Channels the node cannot run without, in declaration order. The node is
    /// placed in a later level than each channel's producing node.
    pub fn required_inputs(&self) -> impl Iterator<Item = &ChannelKey> {
        self.required_inputs.iter().map(|i| &i.channel)
    }

    /// Channels the node uses if a value is present, in declaration order. The
    /// node runs without a value, but the build still requires each channel to
    /// have a supplier.
    pub fn optional_inputs(&self) -> impl Iterator<Item = &ChannelKey> {
        self.optional_inputs.iter().map(|i| &i.channel)
    }

    /// Channels this node writes when it executes.
    pub fn outputs(&self) -> &[ChannelKey] {
        &self.outputs
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
    fn descriptor_without_inputs_yields_nothing() {
        let d = PortDescriptor::new(vec![], vec![], vec![ikey::<u32>()], None);
        assert!(d.inputs().next().is_none());
        assert!(d.required_inputs().next().is_none());
        assert!(d.optional_inputs().next().is_none());
    }
}
