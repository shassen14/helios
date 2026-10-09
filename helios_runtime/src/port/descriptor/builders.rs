//! Constructor-fence builders for [`PortDescriptor`].
//!
//! Two builders, two surfaces:
//!
//! - [`AlgorithmNodePortDescriptor`] accepts `SensorChannel` and
//!   `InternalChannel` as inputs/outputs (a `Sensor` output is a derived
//!   measurement, such as a range field flattened to a cloud). There is **no** method that
//!   accepts an `OracleChannel` or `HealthChannel`. The compiler refuses
//!   to construct an algorithm-node descriptor that names oracle truth as
//!   input — the fence keeps the brain portable to hardware where no
//!   oracle exists.
//!
//! - [`MockNodePortDescriptor`] is the algorithm surface plus
//!   `input_oracle` / `optional_oracle`. Mocks are explicitly licensed to
//!   read reference truth.
//!
//! Both builders emit the same [`PortDescriptor`] shape — the kind tag
//! lives inside each [`ChannelKey`], so the partition is preserved in the
//! final data structure and visible to build-time validation.
//!
//! These builders are the only way to construct a [`PortDescriptor`] from
//! outside this crate: its fields are private and its raw constructor is
//! crate-only.
//!
//! An `ActuatorCommand` output is declared with
//! [`AlgorithmNodePortDescriptor::output_actuator_command`], which also takes
//! the actuators it drives: the actuator seam reads them off the descriptor.
//!
//! Both builders also declare what the node can emit for watchers, one leaf
//! per [`observable`](AlgorithmNodePortDescriptor::observable) call. Leaves
//! with a config-derived part (`aiding.<entry>.nis`) are formatted by the node
//! as it builds its descriptor.
//!
//! Every input a builder records is same-tick: `input_*` methods record a
//! required input and `optional_*` methods an optional one. There is no
//! method for a previous-tick input until the pipeline can actually hand a
//! reader the start-of-tick value.
//!
//! ## Pass-through for erased keys
//!
//! `InputBuilder` traits (estimator / controller / planner / path-follower)
//! return `&[ChannelKey]` — a heterogeneous mix of sensor and internal
//! channels assembled inside the input builder. The descriptor builder
//! exposes [`AlgorithmNodePortDescriptor::inputs_from_slices`] for that
//! pass-through, which asserts every channel is `Sensor` or `Internal`,
//! in release builds too, since this is the one path past the typed
//! methods' kind fence. Compile-time strictness on this path would require an
//! `AlgorithmInputChannel` enum on the input-builder trait — deferred
//! until the trait has another reason to change.
//!
//! The same holds for any caller that holds erased keys rather than typed
//! channels, such as kind-agnostic test nodes exercising the DAG engine:
//! both builders offer `inputs_from_slices` and `outputs_from_slice`, each
//! asserted against that builder's surface. A wrong kind panics when the
//! node is constructed, at startup, never mid-run.

use crate::port::{
    ChannelKey, ChannelKind, Determinism, HealthChannel, InternalChannel, Observable,
    OracleChannel, PortDescriptor, SensorChannel,
};

use helios_core::control::actuators::{ActuatorCommand, ActuatorDrive};

use std::sync::Arc;

use std::any::TypeId;

/// Builder for an algorithm node's [`PortDescriptor`].
///
/// Accepts only Sensor and Internal channels. Construct via [`Self::new`],
/// chain the input/output/rate methods, finish with [`Self::build`].
///
/// # Compile-time fence
///
/// The builder has no method accepting an [`OracleChannel`] or
/// [`HealthChannel`]. The following must NOT compile:
///
/// ```compile_fail
/// use helios_runtime::port::AlgorithmNodePortDescriptor;
/// use helios_runtime::port::OracleChannel;
///
/// struct Pose;
/// let oracle = OracleChannel::named::<Pose>("oracle/pose");
/// // No `input_oracle` method on AlgorithmNodePortDescriptor:
/// let _ = AlgorithmNodePortDescriptor::new().input_oracle(oracle);
/// ```
#[derive(Debug, Default)]
pub struct AlgorithmNodePortDescriptor {
    required_inputs: Vec<ChannelKey>,
    optional_inputs: Vec<ChannelKey>,
    outputs: Vec<ChannelKey>,
    drives: Vec<(ChannelKey, Vec<ActuatorDrive>)>,
    observables: Vec<Observable>,
    rate: Option<f64>,
}

// Symmetric port-descriptor builder vocabulary; not every input/optional
// variant is exercised by current nodes, but the full set is kept deliberately.
impl AlgorithmNodePortDescriptor {
    /// An empty descriptor: no inputs, no outputs, fires every tick.
    pub fn new() -> Self {
        Self::default()
    }

    /// Requires a sensor channel: the build fails unless something produces it.
    pub fn input_sensor(mut self, c: SensorChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    /// Requires an internal channel: the build fails unless a node produces it.
    pub fn input_internal(mut self, c: InternalChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    /// Reads a sensor channel when it holds a value; the node runs without it.
    pub fn optional_sensor(mut self, c: SensorChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    /// Reads an internal channel when it holds a value; the node runs without it.
    pub fn optional_internal(mut self, c: InternalChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    /// Declares an internal channel this node writes. One producer per channel.
    pub fn output_internal(mut self, c: InternalChannel) -> Self {
        self.outputs.push(c.into());
        self
    }

    /// Declares an `ActuatorCommand` channel this node writes, and the
    /// actuators its commands drive, each with the setpoint kind written
    /// there. The actuator seam checks them against the body and against its
    /// other members. One producer per channel.
    ///
    /// # Panics
    ///
    /// If `c` is not an `ActuatorCommand` channel.
    pub fn output_actuator_command(
        mut self,
        c: InternalChannel,
        drives: Vec<ActuatorDrive>,
    ) -> Self {
        assert!(
            c.type_id() == TypeId::of::<ActuatorCommand>(),
            "actuator command output must carry ActuatorCommand, got {} for {}",
            c.type_name(),
            c.instance()
        );
        let key = ChannelKey::from(c);
        self.outputs.push(key.clone());
        self.drives.push((key, drives));
        self
    }

    /// Declares a derived measurement as output: a measurement-to-measurement
    /// node (a range field flattened to a cloud, a de-skewed scan) writes a
    /// `Sensor` channel so consumers read it exactly as they would a host
    /// channel. Channel kind says what the data is, not who produced it.
    /// Algorithm results (state, maps, paths, commands) are `Internal`.
    pub fn output_sensor(mut self, c: SensorChannel) -> Self {
        self.outputs.push(c.into());
        self
    }

    /// Declares a value the node can emit for watchers, under `leaf` (relative
    /// to the node, e.g. `aiding.gps.nis`). Declaring makes a leaf watchable;
    /// an emit of a leaf the node didn't declare is dropped. A leaf declared
    /// twice, or one that collides with an output channel, fails the build.
    pub fn observable(mut self, leaf: impl Into<Arc<str>>, determinism: Determinism) -> Self {
        self.observables.push(Observable::new(leaf, determinism));
        self
    }

    /// Gates execution to `hz`. Without it the node fires every tick.
    pub fn rate_hz(mut self, hz: f64) -> Self {
        self.rate = Some(hz);
        self
    }

    /// Pass-through for input-builder declarations. Caller is responsible
    /// for keeping the slices to `Sensor` / `Internal` kinds. Checked when
    /// the descriptor is built rather than by the compiler, because the
    /// input builder traits return `&[ChannelKey]` for ergonomic reasons.
    ///
    /// # Panics
    ///
    /// If any key is an `Oracle` or `Health` channel.
    pub fn inputs_from_slices(mut self, required: &[ChannelKey], optional: &[ChannelKey]) -> Self {
        for c in required {
            assert!(
                matches!(c.kind(), ChannelKind::Sensor | ChannelKind::Internal),
                "algorithm node required input must be Sensor or Internal, got {:?} for {}",
                c.kind(),
                c
            );
            self.required_inputs.push(c.clone());
        }
        for c in optional {
            assert!(
                matches!(c.kind(), ChannelKind::Sensor | ChannelKind::Internal),
                "algorithm node optional input must be Sensor or Internal, got {:?} for {}",
                c.kind(),
                c
            );
            self.optional_inputs.push(c.clone());
        }
        self
    }

    /// Output-side twin of [`inputs_from_slices`](Self::inputs_from_slices),
    /// for callers that hold erased [`ChannelKey`]s rather than typed
    /// channels. Same contract: `Sensor` / `Internal` only.
    ///
    /// # Panics
    ///
    /// If any key is an `Oracle` or `Health` channel.
    pub fn outputs_from_slice(mut self, outputs: &[ChannelKey]) -> Self {
        for c in outputs {
            assert!(
                matches!(c.kind(), ChannelKind::Sensor | ChannelKind::Internal),
                "algorithm node output must be Sensor or Internal, got {:?} for {}",
                c.kind(),
                c
            );
            self.outputs.push(c.clone());
        }
        self
    }

    /// Finishes the descriptor.
    pub fn build(self) -> PortDescriptor {
        PortDescriptor::new(
            self.required_inputs,
            self.optional_inputs,
            self.outputs,
            self.rate,
        )
        .with_drives(self.drives)
        .with_observables(self.observables)
    }
}

/// Builder for a mock node's [`PortDescriptor`].
///
/// Algorithm surface plus `input_oracle` / `optional_oracle`. Mocks are
/// licensed to read reference truth from oracle channels.
#[derive(Debug, Default)]
pub struct MockNodePortDescriptor {
    required_inputs: Vec<ChannelKey>,
    optional_inputs: Vec<ChannelKey>,
    outputs: Vec<ChannelKey>,
    observables: Vec<Observable>,
    rate: Option<f64>,
}

// Symmetric port-descriptor builder vocabulary for mock/test nodes; the full
// input/optional set is kept deliberately even where unexercised.
impl MockNodePortDescriptor {
    /// An empty descriptor: no inputs, no outputs, fires every tick.
    pub fn new() -> Self {
        Self::default()
    }

    /// Requires a sensor channel: the build fails unless something produces it.
    pub fn input_sensor(mut self, c: SensorChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    /// Requires an internal channel: the build fails unless a node produces it.
    pub fn input_internal(mut self, c: InternalChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    /// Requires a reference-truth channel the body publishes.
    pub fn input_oracle(mut self, c: OracleChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    /// Reads a sensor channel when it holds a value; the node runs without it.
    pub fn optional_sensor(mut self, c: SensorChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    /// Reads an internal channel when it holds a value; the node runs without it.
    pub fn optional_internal(mut self, c: InternalChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    /// Reads a reference-truth channel when it holds a value.
    pub fn optional_oracle(mut self, c: OracleChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    /// Declares an internal channel this node writes. One producer per channel.
    pub fn output_internal(mut self, c: InternalChannel) -> Self {
        self.outputs.push(c.into());
        self
    }

    /// Declares a value the node can emit for watchers, as
    /// [`AlgorithmNodePortDescriptor::observable`] does.
    pub fn observable(mut self, leaf: impl Into<Arc<str>>, determinism: Determinism) -> Self {
        self.observables.push(Observable::new(leaf, determinism));
        self
    }

    /// Gates execution to `hz`. Without it the node fires every tick.
    pub fn rate_hz(mut self, hz: f64) -> Self {
        self.rate = Some(hz);
        self
    }

    /// Pass-through for callers that hold erased [`ChannelKey`]s. The mock
    /// surface admits `Oracle` beside `Sensor` / `Internal`.
    ///
    /// # Panics
    ///
    /// If any key is a `Health` channel.
    pub fn inputs_from_slices(mut self, required: &[ChannelKey], optional: &[ChannelKey]) -> Self {
        for c in required {
            assert!(
                Self::is_mock_input_kind(c),
                "mock node required input must be Sensor, Internal, or Oracle, got {:?} for {}",
                c.kind(),
                c
            );
            self.required_inputs.push(c.clone());
        }
        for c in optional {
            assert!(
                Self::is_mock_input_kind(c),
                "mock node optional input must be Sensor, Internal, or Oracle, got {:?} for {}",
                c.kind(),
                c
            );
            self.optional_inputs.push(c.clone());
        }
        self
    }

    /// Output-side pass-through for erased [`ChannelKey`]s. Mocks publish
    /// `Internal` channels only, matching [`output_internal`](Self::output_internal).
    ///
    /// # Panics
    ///
    /// If any key is not an `Internal` channel.
    pub fn outputs_from_slice(mut self, outputs: &[ChannelKey]) -> Self {
        for c in outputs {
            assert!(
                matches!(c.kind(), ChannelKind::Internal),
                "mock node output must be Internal, got {:?} for {}",
                c.kind(),
                c
            );
            self.outputs.push(c.clone());
        }
        self
    }

    /// Finishes the descriptor.
    pub fn build(self) -> PortDescriptor {
        PortDescriptor::new(
            self.required_inputs,
            self.optional_inputs,
            self.outputs,
            self.rate,
        )
        .with_observables(self.observables)
    }

    fn is_mock_input_kind(c: &ChannelKey) -> bool {
        matches!(
            c.kind(),
            ChannelKind::Sensor | ChannelKind::Internal | ChannelKind::Oracle
        )
    }
}

/// `HealthChannel` has no descriptor-builder surface today. The variant is
/// reserved in [`ChannelKey`] so its `From` impl and equality semantics are
/// already in place; the consumer-side fence lands when the safety
/// supervisor arrives.
#[doc(hidden)]
#[allow(dead_code)]
pub(crate) fn _health_kind_reserved(_: HealthChannel) {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::{InputNeed, InputTiming, InternalChannel, OracleChannel, SensorChannel};

    use helios_core::control::actuators::{ActuatorId, SetpointKind};

    struct State;
    struct Reading;
    struct Pose;

    #[test]
    fn algorithm_builder_assembles_descriptor() {
        let d = AlgorithmNodePortDescriptor::new()
            .input_sensor(SensorChannel::of::<Reading>())
            .input_internal(InternalChannel::of::<State>())
            .output_internal(InternalChannel::named::<State>("smoothed"))
            .rate_hz(50.0)
            .build();
        assert_eq!(d.required_inputs().count(), 2);
        assert_eq!(d.outputs().len(), 1);
        assert_eq!(d.rate(), Some(50.0));
    }

    #[test]
    fn algorithm_builder_keeps_observables_in_declaration_order() {
        let d = AlgorithmNodePortDescriptor::new()
            .observable("aiding.gps.nis", Determinism::Reproducible)
            .observable(
                format!("aiding.{}.dropped", "gps"),
                Determinism::Reproducible,
            )
            .build();

        assert_eq!(
            d.observables(),
            [
                Observable::new("aiding.gps.nis", Determinism::Reproducible),
                Observable::new("aiding.gps.dropped", Determinism::Reproducible),
            ]
        );
    }

    #[test]
    fn mock_builder_keeps_observables_in_declaration_order() {
        let d = MockNodePortDescriptor::new()
            .observable("value", Determinism::Reproducible)
            .observable("search_time", Determinism::WallClock)
            .build();

        assert_eq!(
            d.observables(),
            [
                Observable::new("value", Determinism::Reproducible),
                Observable::new("search_time", Determinism::WallClock),
            ]
        );
    }

    #[test]
    fn builders_declare_no_observables_by_default() {
        assert!(AlgorithmNodePortDescriptor::new()
            .build()
            .observables()
            .is_empty());
        assert!(MockNodePortDescriptor::new()
            .build()
            .observables()
            .is_empty());
    }

    #[test]
    fn actuator_command_output_carries_its_drives() {
        let command = InternalChannel::named::<ActuatorCommand>("drive");
        let other = InternalChannel::named::<State>("drive");
        let drives = vec![ActuatorDrive::new(
            ActuatorId::new("wheels"),
            SetpointKind::Torque,
        )];
        let d = AlgorithmNodePortDescriptor::new()
            .output_internal(other.clone())
            .output_actuator_command(command.clone(), drives.clone())
            .build();

        assert_eq!(d.outputs(), [other.clone().into(), command.clone().into()]);
        assert_eq!(d.drives(&command.into()), drives);
        assert!(d.drives(&other.into()).is_empty());
    }

    #[test]
    #[should_panic(expected = "actuator command output must carry ActuatorCommand")]
    fn actuator_command_output_panics_on_another_type() {
        let _ = AlgorithmNodePortDescriptor::new()
            .output_actuator_command(InternalChannel::named::<State>("drive"), vec![]);
    }

    #[test]
    fn mock_builder_accepts_oracle_input() {
        let d = MockNodePortDescriptor::new()
            .input_oracle(OracleChannel::named::<Pose>("oracle/pose"))
            .output_internal(InternalChannel::of::<State>())
            .build();
        assert_eq!(d.required_inputs().count(), 1);
        assert_eq!(d.outputs().len(), 1);
    }

    #[test]
    fn inputs_from_slices_accepts_sensor_and_internal() {
        let required: Vec<ChannelKey> = vec![
            SensorChannel::of::<Reading>().into(),
            InternalChannel::of::<State>().into(),
        ];
        let d = AlgorithmNodePortDescriptor::new()
            .inputs_from_slices(&required, &[])
            .output_internal(InternalChannel::of::<State>())
            .build();
        assert_eq!(d.required_inputs().count(), 2);
    }

    #[test]
    #[should_panic(expected = "must be Sensor or Internal")]
    fn inputs_from_slices_panics_on_oracle() {
        let bad: Vec<ChannelKey> = vec![OracleChannel::named::<Pose>("oracle/pose").into()];
        let _ = AlgorithmNodePortDescriptor::new().inputs_from_slices(&bad, &[]);
    }

    #[test]
    fn algorithm_outputs_from_slice_accepts_sensor_and_internal() {
        let outputs: Vec<ChannelKey> = vec![
            SensorChannel::named::<Reading>("cloud").into(),
            InternalChannel::of::<State>().into(),
        ];
        let d = AlgorithmNodePortDescriptor::new()
            .outputs_from_slice(&outputs)
            .build();
        assert_eq!(d.outputs(), outputs);
    }

    #[test]
    #[should_panic(expected = "algorithm node output must be Sensor or Internal")]
    fn algorithm_outputs_from_slice_panics_on_oracle() {
        let bad: Vec<ChannelKey> = vec![OracleChannel::named::<Pose>("oracle/pose").into()];
        let _ = AlgorithmNodePortDescriptor::new().outputs_from_slice(&bad);
    }

    #[test]
    fn mock_inputs_from_slices_accepts_oracle_beside_algorithm_kinds() {
        let required: Vec<ChannelKey> = vec![
            OracleChannel::named::<Pose>("oracle/pose").into(),
            SensorChannel::of::<Reading>().into(),
        ];
        let optional: Vec<ChannelKey> = vec![InternalChannel::of::<State>().into()];
        let d = MockNodePortDescriptor::new()
            .inputs_from_slices(&required, &optional)
            .build();
        assert_eq!(d.required_inputs().cloned().collect::<Vec<_>>(), required);
        assert_eq!(d.optional_inputs().cloned().collect::<Vec<_>>(), optional);
    }

    #[test]
    #[should_panic(expected = "mock node output must be Internal")]
    fn mock_outputs_from_slice_panics_on_sensor() {
        let bad: Vec<ChannelKey> = vec![SensorChannel::of::<Reading>().into()];
        let _ = MockNodePortDescriptor::new().outputs_from_slice(&bad);
    }

    // --- Input records ---

    fn needs_and_timings(d: &PortDescriptor) -> Vec<(ChannelKey, InputNeed, InputTiming)> {
        d.inputs()
            .map(|i| (i.channel().clone(), i.need(), i.timing()))
            .collect()
    }

    #[test]
    fn algorithm_input_methods_record_need_with_same_tick_timing() {
        let sensor: ChannelKey = SensorChannel::named::<Reading>("imu").into();
        let internal: ChannelKey = InternalChannel::of::<State>().into();
        let opt_sensor: ChannelKey = SensorChannel::named::<Reading>("gps").into();
        let opt_internal: ChannelKey = InternalChannel::named::<State>("reference").into();
        let d = AlgorithmNodePortDescriptor::new()
            .input_sensor(SensorChannel::named::<Reading>("imu"))
            .optional_sensor(SensorChannel::named::<Reading>("gps"))
            .input_internal(InternalChannel::of::<State>())
            .optional_internal(InternalChannel::named::<State>("reference"))
            .build();

        // Required inputs come first, then optional, each in call order.
        assert_eq!(
            needs_and_timings(&d),
            vec![
                (sensor, InputNeed::Required, InputTiming::SameTick),
                (internal, InputNeed::Required, InputTiming::SameTick),
                (opt_sensor, InputNeed::Optional, InputTiming::SameTick),
                (opt_internal, InputNeed::Optional, InputTiming::SameTick),
            ]
        );
    }

    #[test]
    fn mock_oracle_methods_record_need_with_same_tick_timing() {
        let pose: ChannelKey = OracleChannel::named::<Pose>("oracle/pose").into();
        let twist: ChannelKey = OracleChannel::named::<Pose>("oracle/twist").into();
        let d = MockNodePortDescriptor::new()
            .input_oracle(OracleChannel::named::<Pose>("oracle/pose"))
            .optional_oracle(OracleChannel::named::<Pose>("oracle/twist"))
            .build();

        assert_eq!(
            needs_and_timings(&d),
            vec![
                (pose, InputNeed::Required, InputTiming::SameTick),
                (twist, InputNeed::Optional, InputTiming::SameTick),
            ]
        );
    }

    #[test]
    fn inputs_from_slices_records_need_from_slice_and_keeps_order() {
        let required: Vec<ChannelKey> = vec![
            InternalChannel::named::<State>("b").into(),
            SensorChannel::named::<Reading>("a").into(),
        ];
        let optional: Vec<ChannelKey> = vec![InternalChannel::named::<State>("c").into()];
        let d = AlgorithmNodePortDescriptor::new()
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
        let d = MockNodePortDescriptor::new()
            .input_sensor(SensorChannel::of::<Reading>())
            .input_internal(InternalChannel::of::<State>())
            .input_oracle(OracleChannel::named::<Pose>("oracle/pose"))
            .optional_sensor(SensorChannel::named::<Reading>("opt"))
            .optional_internal(InternalChannel::named::<State>("opt"))
            .optional_oracle(OracleChannel::named::<Pose>("oracle/opt"))
            .build();

        assert!(d.inputs().all(|i| i.timing() == InputTiming::SameTick));
    }
}
