//! Constructor-fence builders for [`PortDescriptor`].
//!
//! Two builders, two surfaces:
//!
//! - [`AlgorithmNodePortDescriptor`] accepts `SensorChannel` and
//!   `InternalChannel` as inputs/outputs. There is **no** method that
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
//! ## Pass-through for input builders
//!
//! `InputBuilder` traits (estimator / controller / planner / path-follower)
//! return `&[ChannelKey]` — a heterogeneous mix of sensor and internal
//! channels assembled inside the input builder. The descriptor builder
//! exposes [`AlgorithmNodePortDescriptor::inputs_from_slices`] for that
//! pass-through, which `debug_assert!`s every channel is `Sensor` or
//! `Internal`. Compile-time strictness on this path would require an
//! `AlgorithmInputChannel` enum on the input-builder trait — deferred
//! until the trait has another reason to change.

use crate::port::{
    ChannelKey, ChannelKind, HealthChannel, InternalChannel, OracleChannel, PortDescriptor,
    SensorChannel,
};

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
/// use helios_runtime::pipeline::descriptor::AlgorithmNodePortDescriptor;
/// use helios_runtime::port::OracleChannel;
///
/// struct Pose;
/// let oracle = OracleChannel::named::<Pose>("oracle/pose");
/// // No `input_oracle` method on AlgorithmNodePortDescriptor:
/// let _ = AlgorithmNodePortDescriptor::new().input_oracle(oracle);
/// ```
#[derive(Debug, Default)]
pub(crate) struct AlgorithmNodePortDescriptor {
    required_inputs: Vec<ChannelKey>,
    optional_inputs: Vec<ChannelKey>,
    outputs: Vec<ChannelKey>,
    rate: Option<f64>,
}

// Symmetric port-descriptor builder vocabulary; not every input/optional
// variant is exercised by current nodes, but the full set is kept deliberately.
#[allow(dead_code)]
impl AlgorithmNodePortDescriptor {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn input_sensor(mut self, c: SensorChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    pub(crate) fn input_internal(mut self, c: InternalChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    pub(crate) fn optional_sensor(mut self, c: SensorChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    pub(crate) fn optional_internal(mut self, c: InternalChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    pub(crate) fn output_internal(mut self, c: InternalChannel) -> Self {
        self.outputs.push(c.into());
        self
    }

    pub(crate) fn rate_hz(mut self, hz: f64) -> Self {
        self.rate = Some(hz);
        self
    }

    /// Pass-through for input-builder declarations. Caller is responsible
    /// for keeping the slices to `Sensor` / `Internal` kinds; misuse is
    /// `debug_assert!`-checked. Deferred to runtime because the input
    /// builder traits return `&[ChannelKey]` for ergonomic reasons.
    pub(crate) fn inputs_from_slices(
        mut self,
        required: &[ChannelKey],
        optional: &[ChannelKey],
    ) -> Self {
        for c in required {
            debug_assert!(
                matches!(c.kind(), ChannelKind::Sensor | ChannelKind::Internal),
                "algorithm node required input must be Sensor or Internal, got {:?} for {}",
                c.kind(),
                c
            );
            self.required_inputs.push(c.clone());
        }
        for c in optional {
            debug_assert!(
                matches!(c.kind(), ChannelKind::Sensor | ChannelKind::Internal),
                "algorithm node optional input must be Sensor or Internal, got {:?} for {}",
                c.kind(),
                c
            );
            self.optional_inputs.push(c.clone());
        }
        self
    }

    pub(crate) fn build(self) -> PortDescriptor {
        PortDescriptor {
            required_inputs: self.required_inputs,
            optional_inputs: self.optional_inputs,
            outputs: self.outputs,
            rate: self.rate,
        }
    }
}

/// Builder for a mock node's [`PortDescriptor`].
///
/// Algorithm surface plus `input_oracle` / `optional_oracle`. Mocks are
/// licensed to read reference truth from oracle channels.
#[derive(Debug, Default)]
pub(crate) struct MockNodePortDescriptor {
    required_inputs: Vec<ChannelKey>,
    optional_inputs: Vec<ChannelKey>,
    outputs: Vec<ChannelKey>,
    rate: Option<f64>,
}

// Symmetric port-descriptor builder vocabulary for mock/test nodes; the full
// input/optional set is kept deliberately even where unexercised.
#[allow(dead_code)]
impl MockNodePortDescriptor {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn input_sensor(mut self, c: SensorChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    pub(crate) fn input_internal(mut self, c: InternalChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    pub(crate) fn input_oracle(mut self, c: OracleChannel) -> Self {
        self.required_inputs.push(c.into());
        self
    }

    pub(crate) fn optional_sensor(mut self, c: SensorChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    pub(crate) fn optional_internal(mut self, c: InternalChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    pub(crate) fn optional_oracle(mut self, c: OracleChannel) -> Self {
        self.optional_inputs.push(c.into());
        self
    }

    pub(crate) fn output_internal(mut self, c: InternalChannel) -> Self {
        self.outputs.push(c.into());
        self
    }

    pub(crate) fn rate_hz(mut self, hz: f64) -> Self {
        self.rate = Some(hz);
        self
    }

    pub(crate) fn build(self) -> PortDescriptor {
        PortDescriptor {
            required_inputs: self.required_inputs,
            optional_inputs: self.optional_inputs,
            outputs: self.outputs,
            rate: self.rate,
        }
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
    use crate::port::{InternalChannel, OracleChannel, SensorChannel};

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
        assert_eq!(d.required_inputs.len(), 2);
        assert_eq!(d.outputs.len(), 1);
        assert_eq!(d.rate, Some(50.0));
    }

    #[test]
    fn mock_builder_accepts_oracle_input() {
        let d = MockNodePortDescriptor::new()
            .input_oracle(OracleChannel::named::<Pose>("oracle/pose"))
            .output_internal(InternalChannel::of::<State>())
            .build();
        assert_eq!(d.required_inputs.len(), 1);
        assert_eq!(d.outputs.len(), 1);
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
        assert_eq!(d.required_inputs.len(), 2);
    }

    #[test]
    #[should_panic(expected = "must be Sensor or Internal")]
    fn inputs_from_slices_panics_on_oracle_in_debug() {
        let bad: Vec<ChannelKey> = vec![OracleChannel::named::<Pose>("oracle/pose").into()];
        let _ = AlgorithmNodePortDescriptor::new().inputs_from_slices(&bad, &[]);
    }
}
