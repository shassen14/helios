//! [`PortDescriptorBuilder`], the one way to build a [`PortDescriptor`] from
//! outside this crate: its fields are private and its raw constructor is
//! crate-only.
//!
//! The builder starts from the class the node runs under:
//! [`algorithm`](PortDescriptorBuilder::algorithm) or
//! [`mock`](PortDescriptorBuilder::mock). Every channel carries its kind in its
//! [`ChannelKey`], so the builder takes channels of every kind alike; the
//! pipeline build checks each against the table in
//! [`kind`](crate::port::kind) and refuses an algorithm reading truth, naming
//! the node and the channel.
//!
//! An output on a keyed channel is declared with
//! [`output_keyed`](PortDescriptorBuilder::output_keyed), which also takes
//! the instances the node writes.
//!
//! An `ActuatorCommand` output is declared with
//! [`output_actuator_command`](PortDescriptorBuilder::output_actuator_command),
//! which also takes the actuators it drives: the actuator seam reads them off
//! the descriptor.
//!
//! The builder also declares what the node can emit for watchers, one leaf
//! per [`observable`](PortDescriptorBuilder::observable) call. Leaves with a
//! config-derived part (`aiding.<entry>.nis`) are formatted by the node as it
//! builds its descriptor.
//!
//! Every input the builder records is same-tick: [`input`](PortDescriptorBuilder::input)
//! records a required input and [`optional`](PortDescriptorBuilder::optional)
//! an optional one. There is no method for a previous-tick input until the
//! pipeline can actually hand a reader the start-of-tick value.

use crate::port::{
    ChannelKey, ChannelKind, Determinism, InstanceKey, Keyed, MountRule, NodeClass, Observable,
    OutputPort, PortDescriptor,
};

use helios_core::control::actuators::{ActuatorCommand, ActuatorDrive};
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

use std::{any::TypeId, sync::Arc};

/// Builder for a node's [`PortDescriptor`]. Construct with
/// [`Self::algorithm`] or [`Self::mock`], chain the input, output and rate
/// methods, finish with [`Self::build`].
#[derive(Debug, Default)]
pub struct PortDescriptorBuilder {
    class: NodeClass,
    required_inputs: Vec<ChannelKey>,
    optional_inputs: Vec<ChannelKey>,
    mount_readings: Vec<(ChannelKey, Convention)>,
    outputs: Vec<OutputPort>,
    observables: Vec<Observable>,
    defined_frames: Vec<FrameId>,
    rate: Option<f64>,
}

// The full declaration vocabulary is kept deliberately, though not every
// method is exercised by current nodes.
impl PortDescriptorBuilder {
    /// An empty descriptor for an algorithm node, which runs the same in sim
    /// and on hardware: no inputs, no outputs, fires every tick.
    pub fn algorithm() -> Self {
        Self::default()
    }

    /// An empty descriptor for a mock node, which stands in for an algorithm
    /// and may read truth: no inputs, no outputs, fires every tick.
    pub fn mock() -> Self {
        Self {
            class: NodeClass::Mock,
            ..Self::default()
        }
    }

    /// Requires `channel`: the build fails unless something supplies it.
    pub fn input(mut self, channel: ChannelKey) -> Self {
        self.required_inputs.push(channel);
        self
    }

    /// Reads `channel` when it holds a value; the node runs without it. The
    /// build still fails unless something supplies it.
    pub fn optional(mut self, channel: ChannelKey) -> Self {
        self.optional_inputs.push(channel);
        self
    }

    /// Requires each of `required` and reads each of `optional` when it
    /// holds a value, in slice order: for callers that assemble their inputs
    /// as lists, such as the input builders.
    pub fn inputs_from_slices(mut self, required: &[ChannelKey], optional: &[ChannelKey]) -> Self {
        self.required_inputs.extend_from_slice(required);
        self.optional_inputs.extend_from_slice(optional);
        self
    }

    /// Declares that the node resolves the mount of input `channel` through
    /// the tf tree, reading the frame in the axes `convention` names. The host
    /// refuses a mount the tree holds in other axes before the first tick.
    /// Checked at [`build`](Self::build): `channel` must be declared as an
    /// input by then.
    pub fn reads_mount_in(mut self, channel: &ChannelKey, convention: Convention) -> Self {
        self.mount_readings.push((channel.clone(), convention));
        self
    }

    /// Declares a channel this node writes. One producer per channel.
    ///
    /// A node writes a measurement when it derives one without estimating (a
    /// range field flattened to a cloud, a de-skewed scan), so consumers read
    /// it as they would a host channel. Algorithm results (state, maps, paths,
    /// commands) are internal.
    pub fn output(mut self, channel: ChannelKey) -> Self {
        self.outputs.push(OutputPort::undriven(channel));
        self
    }

    /// Declares each of `outputs`, in slice order, as [`output`](Self::output)
    /// does.
    pub fn outputs_from_slice(mut self, outputs: &[ChannelKey]) -> Self {
        self.outputs
            .extend(outputs.iter().cloned().map(OutputPort::undriven));
        self
    }

    /// Declares a derived measurement, as [`output`](Self::output) does,
    /// measured in the mount of `input`, which the node must read in the same
    /// tick. The build gives the output that mount, so its readers learn the
    /// frame from the channel without the node knowing it.
    ///
    /// # Panics
    ///
    /// If `channel` or `input` is not a measurement: only a measurement is
    /// measured in a mount.
    pub fn output_at_mount_of(mut self, channel: ChannelKey, input: ChannelKey) -> Self {
        for key in [&channel, &input] {
            assert!(
                key.kind() == ChannelKind::Measurement,
                "an output at an input's mount and that input must be measurements, got {key}"
            );
        }
        self.outputs
            .push(OutputPort::undriven(channel).with_mount(MountRule::FromInput(input)));
        self
    }

    /// Declares the `instances` of keyed channel `channel` this node writes,
    /// each spelled as `T` spells its instances. The build refuses an
    /// instance two writers declare, and the bus refuses a write of one no
    /// writer declared. A keyed channel declared through
    /// [`output`](Self::output) has no instances, which the build refuses.
    ///
    /// # Panics
    ///
    /// If `channel` is not keyed or does not carry `T`.
    pub fn output_keyed<T: Keyed>(
        mut self,
        channel: ChannelKey,
        instances: impl IntoIterator<Item = T::Instance>,
    ) -> Self {
        assert!(
            channel.is_keyed() && channel.type_id() == TypeId::of::<T>(),
            "a keyed output must be a keyed channel of {}, got {channel}",
            std::any::type_name::<T>()
        );
        let instances = instances
            .into_iter()
            .map(|instance| InstanceKey::of::<T>(&instance))
            .collect();
        self.outputs
            .push(OutputPort::undriven(channel).with_instances(instances));
        self
    }

    /// Declares an `ActuatorCommand` channel this node writes, and the
    /// actuators its commands drive, each with the setpoint kind written
    /// there. The actuator seam checks them against the body and against its
    /// other members. One producer per channel.
    ///
    /// # Panics
    ///
    /// If `channel` does not carry `ActuatorCommand`.
    pub fn output_actuator_command(
        mut self,
        channel: ChannelKey,
        drives: Vec<ActuatorDrive>,
    ) -> Self {
        assert!(
            channel.type_id() == TypeId::of::<ActuatorCommand>(),
            "actuator command output must carry ActuatorCommand, got {} for {}",
            channel.type_name(),
            channel.name()
        );
        self.outputs.push(OutputPort::new(channel, drives));
        self
    }

    /// Declares a number the node can emit for watchers, under `leaf`
    /// (relative to the node, e.g. `aiding.gps.nis`). Declaring makes a leaf
    /// watchable; an emit of a leaf the node didn't declare is dropped. A
    /// leaf declared twice, or one that collides with an output channel,
    /// fails the build.
    pub fn observable(mut self, leaf: impl Into<Arc<str>>, determinism: Determinism) -> Self {
        self.observables.push(Observable::scalar(leaf, determinism));
        self
    }

    /// Declares a frame this node defines: one whose meaning it alone gives,
    /// such as the root an estimator holds its state in. The build refuses a
    /// frame two nodes define, or one the body owns.
    pub fn defines_frame(mut self, frame: FrameId) -> Self {
        self.defined_frames.push(frame);
        self
    }

    /// Gates execution to `hz`. Without it the node fires every tick.
    pub fn rate_hz(mut self, hz: f64) -> Self {
        self.rate = Some(hz);
        self
    }

    /// Finishes the descriptor.
    ///
    /// # Panics
    ///
    /// If [`reads_mount_in`](Self::reads_mount_in) named a channel that is
    /// not a declared input.
    pub fn build(self) -> PortDescriptor {
        PortDescriptor::new(
            self.required_inputs,
            self.optional_inputs,
            Vec::new(),
            self.rate,
        )
        .with_class(self.class)
        .with_mount_readings(self.mount_readings)
        .with_outputs(self.outputs)
        .with_observables(self.observables)
        .with_defined_frames(self.defined_frames)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::{InputNeed, InputTiming};

    use helios_core::control::actuators::{ActuatorId, SetpointKind};

    struct State;
    struct Reading;
    struct Pose;

    struct Edge {
        child: String,
    }

    impl Keyed for Edge {
        type Instance = String;

        fn instance(&self) -> &String {
            &self.child
        }
    }

    fn edges() -> ChannelKey {
        ChannelKey::keyed::<Edge>(ChannelKind::Internal, "edges")
    }

    #[test]
    fn builder_assembles_descriptor() {
        let d = PortDescriptorBuilder::algorithm()
            .input(ChannelKey::measurement::<Reading>("reading"))
            .input(ChannelKey::internal_of::<State>())
            .output(ChannelKey::internal::<State>("smoothed"))
            .rate_hz(50.0)
            .build();
        assert_eq!(d.required_inputs().count(), 2);
        assert_eq!(d.outputs().count(), 1);
        assert_eq!(d.rate(), Some(50.0));
    }

    #[test]
    fn each_constructor_records_its_class() {
        assert_eq!(
            PortDescriptorBuilder::algorithm().build().class(),
            NodeClass::Algorithm
        );
        assert_eq!(
            PortDescriptorBuilder::mock().build().class(),
            NodeClass::Mock
        );
    }

    /// The builder records an oracle input for any class; the pipeline build
    /// is what refuses one an algorithm declares.
    #[test]
    fn the_builder_records_every_kind() {
        let pose = ChannelKey::oracle::<Pose>("pose");
        let d = PortDescriptorBuilder::algorithm()
            .input(pose.clone())
            .build();
        assert_eq!(d.required_inputs().collect::<Vec<_>>(), [&pose]);
    }

    #[test]
    fn builder_keeps_observables_in_declaration_order() {
        let d = PortDescriptorBuilder::algorithm()
            .observable("aiding.gps.nis", Determinism::Deterministic)
            .observable(
                format!("aiding.{}.dropped", "gps"),
                Determinism::Nondeterministic,
            )
            .build();

        assert_eq!(
            d.observables(),
            [
                Observable::scalar("aiding.gps.nis", Determinism::Deterministic),
                Observable::scalar("aiding.gps.dropped", Determinism::Nondeterministic),
            ]
        );
    }

    #[test]
    fn builder_declares_no_observables_by_default() {
        assert!(PortDescriptorBuilder::algorithm()
            .build()
            .observables()
            .is_empty());
    }

    #[test]
    fn actuator_command_output_carries_its_drives() {
        let command = ChannelKey::internal::<ActuatorCommand>("drive");
        let other = ChannelKey::internal::<State>("drive");
        let drives = vec![ActuatorDrive::new(
            ActuatorId::new("wheels"),
            SetpointKind::Torque,
        )];
        let d = PortDescriptorBuilder::algorithm()
            .output(other.clone())
            .output_actuator_command(command.clone(), drives.clone())
            .build();

        assert_eq!(
            d.outputs().cloned().collect::<Vec<_>>(),
            [other.clone(), command.clone()]
        );
        assert_eq!(d.drives(&command), drives);
        assert!(d.drives(&other).is_empty());
    }

    #[test]
    #[should_panic(expected = "actuator command output must carry ActuatorCommand")]
    fn actuator_command_output_panics_on_another_type() {
        let _ = PortDescriptorBuilder::algorithm()
            .output_actuator_command(ChannelKey::internal::<State>("drive"), vec![]);
    }

    /// An output declared at an input's mount carries that rule; every other
    /// output has none.
    #[test]
    fn an_output_at_an_inputs_mount_names_that_input() {
        let range = ChannelKey::measurement::<Reading>("lidar");
        let cloud = ChannelKey::measurement::<Reading>("lidar.cloud");
        let d = PortDescriptorBuilder::algorithm()
            .input(range.clone())
            .output_at_mount_of(cloud, range.clone())
            .output(ChannelKey::internal_of::<State>())
            .build();

        let rules: Vec<&MountRule> = d.output_ports().iter().map(OutputPort::mount).collect();
        assert_eq!(rules, [&MountRule::FromInput(range), &MountRule::None]);
    }

    #[test]
    #[should_panic(expected = "must be measurements")]
    fn an_internal_output_at_an_inputs_mount_panics() {
        let range = ChannelKey::measurement::<Reading>("lidar");
        let _ = PortDescriptorBuilder::algorithm()
            .output_at_mount_of(ChannelKey::internal::<Reading>("cloud"), range);
    }

    /// Each stated mount reading is kept, in order, one channel as often as
    /// it is stated: two parts of one node may read the same mount.
    #[test]
    fn a_keyed_output_carries_its_instances_in_order() {
        let d = PortDescriptorBuilder::algorithm()
            .output_keyed::<Edge>(edges(), ["wheel".to_string(), "base".to_string()])
            .output(ChannelKey::internal::<State>("state"))
            .build();
        let spelled: Vec<&str> = d.output_ports()[0]
            .instances()
            .iter()
            .map(InstanceKey::as_str)
            .collect();
        assert_eq!(spelled, ["wheel", "base"]);
        assert!(d.output_ports()[1].instances().is_empty());
    }

    #[test]
    #[should_panic(expected = "a keyed output must be a keyed channel")]
    fn a_keyed_output_on_a_single_value_channel_panics() {
        let _ = PortDescriptorBuilder::algorithm()
            .output_keyed::<Edge>(ChannelKey::internal::<Edge>("edges"), ["base".to_string()]);
    }

    #[test]
    fn mount_readings_are_kept_for_the_inputs_they_name() {
        let imu = ChannelKey::measurement::<Reading>("imu");
        let gps = ChannelKey::measurement::<Reading>("gps");
        let d = PortDescriptorBuilder::algorithm()
            .inputs_from_slices(std::slice::from_ref(&imu), std::slice::from_ref(&gps))
            .reads_mount_in(&imu, Convention::Flu)
            .reads_mount_in(&imu, Convention::Enu)
            .build();

        let readings: Vec<(&ChannelKey, Convention)> = d
            .mount_readings()
            .iter()
            .map(|reading| (reading.channel(), reading.convention()))
            .collect();
        assert_eq!(readings, [(&imu, Convention::Flu), (&imu, Convention::Enu)]);
    }

    #[test]
    #[should_panic(expected = "which the node does not read")]
    fn a_mount_reading_for_a_channel_not_read_panics() {
        let _ = PortDescriptorBuilder::algorithm()
            .reads_mount_in(&ChannelKey::measurement::<Reading>("gps"), Convention::Flu)
            .build();
    }

    #[test]
    fn outputs_from_slice_keeps_order() {
        let outputs = vec![
            ChannelKey::measurement::<Reading>("cloud"),
            ChannelKey::internal_of::<State>(),
        ];
        let d = PortDescriptorBuilder::algorithm()
            .outputs_from_slice(&outputs)
            .build();
        assert_eq!(d.outputs().cloned().collect::<Vec<_>>(), outputs);
    }

    // --- Input records ---

    fn needs_and_timings(d: &PortDescriptor) -> Vec<(ChannelKey, InputNeed, InputTiming)> {
        d.inputs()
            .map(|i| (i.channel().clone(), i.need(), i.timing()))
            .collect()
    }

    #[test]
    fn input_methods_record_need_with_same_tick_timing() {
        let imu = ChannelKey::measurement::<Reading>("imu");
        let state = ChannelKey::internal_of::<State>();
        let gps = ChannelKey::measurement::<Reading>("gps");
        let reference = ChannelKey::internal::<State>("reference");
        let d = PortDescriptorBuilder::algorithm()
            .input(imu.clone())
            .optional(gps.clone())
            .input(state.clone())
            .optional(reference.clone())
            .build();

        // Required inputs come first, then optional, each in call order.
        assert_eq!(
            needs_and_timings(&d),
            vec![
                (imu, InputNeed::Required, InputTiming::SameTick),
                (state, InputNeed::Required, InputTiming::SameTick),
                (gps, InputNeed::Optional, InputTiming::SameTick),
                (reference, InputNeed::Optional, InputTiming::SameTick),
            ]
        );
    }

    #[test]
    fn inputs_from_slices_records_need_from_slice_and_keeps_order() {
        let required = vec![
            ChannelKey::internal::<State>("b"),
            ChannelKey::measurement::<Reading>("a"),
        ];
        let optional = vec![ChannelKey::internal::<State>("c")];
        let d = PortDescriptorBuilder::algorithm()
            .inputs_from_slices(&required, &optional)
            .build();

        assert_eq!(
            needs_and_timings(&d),
            vec![
                (
                    required[0].clone(),
                    InputNeed::Required,
                    InputTiming::SameTick
                ),
                (
                    required[1].clone(),
                    InputNeed::Required,
                    InputTiming::SameTick
                ),
                (
                    optional[0].clone(),
                    InputNeed::Optional,
                    InputTiming::SameTick
                ),
            ]
        );
    }

    #[test]
    fn no_builder_input_is_previous_tick() {
        let d = PortDescriptorBuilder::mock()
            .input(ChannelKey::measurement::<Reading>("reading"))
            .input(ChannelKey::oracle::<Pose>("pose"))
            .optional(ChannelKey::internal::<State>("opt"))
            .optional(ChannelKey::oracle::<Pose>("twist"))
            .build();

        assert!(d.inputs().all(|i| i.timing() == InputTiming::SameTick));
    }
}
