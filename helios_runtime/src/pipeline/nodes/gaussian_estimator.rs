//! [`GaussianEstimatorNode`] — pipeline adapter for any Gaussian-family filter
//! (EKF, UKF, ESKF, IF).
//!
//! One node type per algorithm family, generic over the family trait object.
//! EKF and UKF both wear the same node here; their differences
//! (sigma points vs. Jacobians) live behind [`GaussianStateEstimator`].
//!
//! ## Execution skeleton
//!
//! 1. **Predict.** Ask the [`EstimatorInputBuilder`] to assemble the control
//!    vector from the bus. If `None`, skip predict (cold-start / sensor dropout)
//!    and proceed to updates anyway.
//! 2. **Update.** For each [`AidingHandler`]: read its sensor channel, sort
//!    readings by timestamp, and sequentially apply each one to the filter.
//! 3. **Publish.** Snapshot the filter state and write `FrameAwareState @ ""`
//!    on the bus.

use std::marker::PhantomData;
use std::sync::atomic::Ordering;
use std::sync::Mutex;

use atomic_float::AtomicF64;
use helios_core::data::envelope::SensorReading;
use helios_core::data::sensor::SensorPayload;
use helios_core::estimation::measurement::MeasurementModel;
use helios_core::estimation::GaussianStateEstimator;
use helios_core::frames::FrameAwareState;
use helios_core::ports::TfProvider;
use nalgebra::DMatrix;

use crate::pipeline::builders::estimator::EstimatorInputBuilder;
use crate::pipeline::descriptor::AlgorithmNodePortDescriptor;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, InternalChannel, PortBus, PortDescriptor, SensorChannel};
use crate::runtime::{AgentRuntime, TfProviderAdapter};
use crate::stamped::{Health, Stamped};

/// Per-channel aiding handler: reads one sensor channel from the bus and feeds
/// each reading into a [`GaussianStateEstimator`] via its measurement model.
///
/// The generic payload type `T` is erased behind this trait so the node can hold
/// a `Vec<Box<dyn AidingHandler>>` across sensor types.
pub trait AidingHandler: Send + Sync {
    /// The bus channel this handler reads from.
    fn channel(&self) -> &ChannelKey;

    /// Pull all readings on this channel and sequentially apply them.
    ///
    /// Implementations must sort by reading timestamp before applying so the
    /// filter sees measurements in causal order. `tf` is forwarded as-is to
    /// [`GaussianStateEstimator::update`] — `None` is valid; the filter and
    /// measurement model handle it.
    fn drain_and_apply(
        &self,
        bus: &PortBus,
        estimator: &mut dyn GaussianStateEstimator,
        tf: Option<&dyn TfProvider>,
    );
}

/// Generic [`AidingHandler`] for one [`SensorPayload`] type.
///
/// Owns the [`MeasurementModel`] (the math) and the noise covariance `R`
/// (per-sensor). Both are constructed once and reused across every reading on
/// the channel.
pub struct TypedAidingHandler<T: SensorPayload> {
    /// Cached enum-form key for `bus.read` calls. Built once from the
    /// kinded `SensorChannel` passed to [`Self::new`].
    channel: ChannelKey,
    model: Box<dyn MeasurementModel>,
    r: DMatrix<f64>,
    /// Highest per-reading [`SensorReading::timestamp`] applied to the
    /// filter so far. Readings with `timestamp <= last_applied_ts` are
    /// skipped — re-applying the same measurement would over-tighten the
    /// EKF's posterior as if independent observations had been received.
    last_applied_ts: AtomicF64,
    // phantom data in order to avoid compile error that T isn't used
    // fn() -> T to say output-only, non-owned, covariant data
    _phantom: PhantomData<fn() -> T>,
}

impl<T: SensorPayload> TypedAidingHandler<T> {
    /// Build a handler that reads `Vec<SensorReading<T>>` from `channel`.
    ///
    /// `r` must be square with side equal to `model.dim()`; the filter silently
    /// skips updates with mismatched `R` so a wrong-sized matrix becomes a
    /// no-op rather than a panic.
    pub fn new(channel: SensorChannel, model: Box<dyn MeasurementModel>, r: DMatrix<f64>) -> Self {
        Self {
            channel: channel.into(),
            model,
            r,
            last_applied_ts: AtomicF64::new(f64::NEG_INFINITY),
            _phantom: PhantomData,
        }
    }
}

impl<T: SensorPayload> AidingHandler for TypedAidingHandler<T> {
    fn channel(&self) -> &ChannelKey {
        &self.channel
    }

    fn drain_and_apply(
        &self,
        bus: &PortBus,
        estimator: &mut dyn GaussianStateEstimator,
        tf: Option<&dyn TfProvider>,
    ) {
        let Some(stamped) = bus.read::<Vec<SensorReading<T>>>(self.channel.clone()) else {
            return;
        };
        if stamped.value.is_empty() {
            return;
        }

        // Sort by timestamp so the filter sees readings in causal order even if
        // the producer batched out-of-order arrivals into one tick.
        let mut indices: Vec<usize> = (0..stamped.value.len()).collect();
        indices.sort_by(|&a, &b| {
            stamped.value[a]
                .timestamp
                .0
                .total_cmp(&stamped.value[b].timestamp.0)
        });

        // Skip readings already applied on a prior tick. Bus slots are
        // last-known-good, so the same batch can show up on consecutive
        // ticks; re-applying would treat one measurement as several
        // independent observations and overstate confidence.
        let last_applied = self.last_applied_ts.load(Ordering::Relaxed);
        let mut max_applied = last_applied;
        for idx in indices {
            let reading_ts = stamped.value[idx].timestamp.0;
            if reading_ts <= last_applied {
                continue;
            }
            let z = stamped.value[idx].data.to_measurement_vector();
            estimator.update(&z, &*self.model, &self.r, tf);
            if reading_ts > max_applied {
                max_applied = reading_ts;
            }
        }
        if max_applied > last_applied {
            self.last_applied_ts.store(max_applied, Ordering::Relaxed);
        }
    }
}

/// Pipeline node wrapping any Gaussian-family estimator.
///
/// Construction is via [`Self::new`]. The port descriptor is derived from the
/// input builder and aiding handlers — callers don't compose channel keys
/// directly. Output is `FrameAwareState @ ""`.
pub struct GaussianEstimatorNode {
    name: String,
    estimator: Mutex<Box<dyn GaussianStateEstimator>>,
    input_builder: Box<dyn EstimatorInputBuilder>,
    aiding: Vec<Box<dyn AidingHandler>>,
    descriptor: PortDescriptor,
}

impl GaussianEstimatorNode {
    pub fn new(
        name: impl Into<String>,
        estimator: Box<dyn GaussianStateEstimator>,
        input_builder: Box<dyn EstimatorInputBuilder>,
        aiding: Vec<Box<dyn AidingHandler>>,
    ) -> Self {
        let mut builder = AlgorithmNodePortDescriptor::new()
            .inputs_from_slices(
                input_builder.required_channels(),
                input_builder.optional_channels(),
            )
            .output_internal(InternalChannel::of::<FrameAwareState>());
        for handler in &aiding {
            // Aiding handlers always read a SensorChannel; the cached
            // enum-form is unwrapped back via `kind()`-checked optional.
            builder = builder.inputs_from_slices(&[], &[handler.channel().clone()]);
        }
        let descriptor = builder.build();
        Self {
            name: name.into(),
            estimator: Mutex::new(estimator),
            input_builder,
            aiding,
            descriptor,
        }
    }
}

impl PipelineNode for GaussianEstimatorNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, runtime: &dyn AgentRuntime, tick: TickContext) {
        let tf_adapter = TfProviderAdapter(runtime);
        let tf: Option<&dyn TfProvider> = Some(&tf_adapter);

        // Skip the tick on a poisoned mutex rather than propagating the panic
        let Ok(mut estimator) = self.estimator.lock() else {
            return;
        };

        // 1. Predict (skip if input builder can't assemble — cold-start, dropout).
        if let Some(inputs) = self.input_builder.assemble(bus, runtime, &tick) {
            estimator.predict(tick.dt, &inputs);
        }

        // 2. Update from each aiding sensor.
        for handler in &self.aiding {
            handler.drain_and_apply(bus, &mut **estimator, tf);
        }

        // 3. Publish snapshot.
        let snapshot = estimator.state().clone();
        let stamped = Stamped {
            value: snapshot,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(InternalChannel::of::<FrameAwareState>().into(), stamped);
    }
}

#[cfg(test)]
mod tests {
    //! Tests for [`GaussianEstimatorNode`] using minimal mocks. EKF/UKF behavior
    //! is covered in `helios_core/src/estimation/filters/`; here we verify only
    //! the node's wiring: predict-side input assembly, aiding-handler dispatch,
    //! and bus publish.

    use super::*;
    use helios_core::data::envelope::SensorReading;
    use helios_core::data::primitives::{FrameHandle, MonotonicTime};
    use helios_core::data::sensor::LinearAcceleration3D;
    use helios_core::estimation::EstimatorInputs;
    use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
    use nalgebra::{DMatrix, DVector, Isometry3};
    use std::sync::Mutex as StdMutex;

    // --- Mock AgentRuntime ---

    struct MockRuntime {
        now: f64,
    }

    impl AgentRuntime for MockRuntime {
        fn get_transform(&self, _: FrameHandle, _: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn world_pose(&self, _: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn now(&self) -> MonotonicTime {
            MonotonicTime(self.now)
        }
    }

    // --- Mock GaussianStateEstimator that counts calls ---

    #[derive(Default)]
    struct MockEstimatorCounts {
        predict_calls: u32,
        update_calls: u32,
        last_dt: f64,
    }

    struct MockEstimator {
        state: FrameAwareState,
        counts: StdMutex<MockEstimatorCounts>,
    }

    impl MockEstimator {
        fn new() -> Self {
            let layout = vec![StateVariable::Px(FrameId::World)];
            Self {
                state: FrameAwareState::new(layout, 1.0, 0.0),
                counts: StdMutex::new(Default::default()),
            }
        }
    }

    impl GaussianStateEstimator for MockEstimator {
        fn predict(&mut self, dt: f64, _inputs: &EstimatorInputs) {
            let mut c = self.counts.lock().unwrap();
            c.predict_calls += 1;
            c.last_dt = dt;
        }
        fn update(
            &mut self,
            _z: &DVector<f64>,
            _model: &dyn MeasurementModel,
            _r: &DMatrix<f64>,
            _tf: Option<&dyn TfProvider>,
        ) {
            self.counts.lock().unwrap().update_calls += 1;
        }
        fn state(&self) -> &FrameAwareState {
            &self.state
        }
    }

    // --- Mock MeasurementModel ---

    struct OnePassModel;

    impl MeasurementModel for OnePassModel {
        fn dim(&self) -> usize {
            3
        }
        fn predict_measurement(
            &self,
            _state: &FrameAwareState,
            _tf: Option<&dyn TfProvider>,
        ) -> Option<DVector<f64>> {
            Some(DVector::zeros(3))
        }
    }

    // --- Mock EstimatorInputBuilder ---

    struct AlwaysReadyBuilder {
        required: Vec<ChannelKey>,
    }
    impl AlwaysReadyBuilder {
        fn new() -> Self {
            Self { required: vec![] }
        }
    }

    impl EstimatorInputBuilder for AlwaysReadyBuilder {
        fn assemble(
            &self,
            _bus: &PortBus,
            _runtime: &dyn AgentRuntime,
            _tick: &TickContext,
        ) -> Option<EstimatorInputs> {
            Some(EstimatorInputs {
                control: DVector::zeros(0),
            })
        }
        fn required_channels(&self) -> &[ChannelKey] {
            &self.required
        }
        fn optional_channels(&self) -> &[ChannelKey] {
            &[]
        }
    }

    struct NeverReadyBuilder {
        required: Vec<ChannelKey>,
    }
    impl EstimatorInputBuilder for NeverReadyBuilder {
        fn assemble(
            &self,
            _bus: &PortBus,
            _runtime: &dyn AgentRuntime,
            _tick: &TickContext,
        ) -> Option<EstimatorInputs> {
            None
        }
        fn required_channels(&self) -> &[ChannelKey] {
            &self.required
        }
        fn optional_channels(&self) -> &[ChannelKey] {
            &[]
        }
    }

    // --- Helpers ---

    fn accel_sensor_channel() -> SensorChannel {
        SensorChannel::of::<Vec<SensorReading<LinearAcceleration3D>>>()
    }

    fn accel_channel() -> ChannelKey {
        accel_sensor_channel().into()
    }

    fn state_channel() -> ChannelKey {
        InternalChannel::of::<FrameAwareState>().into()
    }

    fn make_bus(extra_outputs: Vec<ChannelKey>) -> PortBus {
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: {
                let mut v = vec![state_channel(), accel_channel()];
                v.extend(extra_outputs);
                v
            },
            rate: None,
        };
        PortBus::new(&[descriptor])
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 0,
        }
    }

    // --- Tests ---

    #[test]
    fn descriptor_outputs_frame_aware_state() {
        let node = GaussianEstimatorNode::new(
            "ekf",
            Box::new(MockEstimator::new()),
            Box::new(AlwaysReadyBuilder::new()),
            vec![],
        );
        assert_eq!(node.port_descriptor().outputs.len(), 1);
        assert_eq!(node.port_descriptor().outputs[0], state_channel());
    }

    #[test]
    fn descriptor_lists_aiding_channels_as_optional() {
        let handler = TypedAidingHandler::<LinearAcceleration3D>::new(
            accel_sensor_channel(),
            Box::new(OnePassModel),
            DMatrix::identity(3, 3),
        );
        let node = GaussianEstimatorNode::new(
            "ekf",
            Box::new(MockEstimator::new()),
            Box::new(AlwaysReadyBuilder::new()),
            vec![Box::new(handler)],
        );
        assert!(node
            .port_descriptor()
            .optional_inputs
            .contains(&accel_channel()));
    }

    #[test]
    fn execute_publishes_state_with_correct_stamp() {
        let node = GaussianEstimatorNode::new(
            "ekf",
            Box::new(MockEstimator::new()),
            Box::new(AlwaysReadyBuilder::new()),
            vec![],
        );
        let bus = make_bus(vec![]);
        let runtime = MockRuntime { now: 1.0 };

        node.execute(&bus, &runtime, tick_at(1.0, 0.1));

        let published = bus
            .read::<FrameAwareState>(state_channel())
            .expect("node must publish FrameAwareState");
        assert!((published.timestamp.0 - 1.0).abs() < 1e-9);
        assert_eq!(published.producer, 0);
    }

    #[test]
    fn execute_publishes_even_when_input_builder_returns_none() {
        // Cold-start: predict is skipped but state still gets published so
        // downstream consumers see *something* (the prior).
        let node = GaussianEstimatorNode::new(
            "ekf",
            Box::new(MockEstimator::new()),
            Box::new(NeverReadyBuilder { required: vec![] }),
            vec![],
        );
        let bus = make_bus(vec![]);
        let runtime = MockRuntime { now: 0.0 };

        node.execute(&bus, &runtime, tick_at(0.5, 0.1));

        assert!(bus.read::<FrameAwareState>(state_channel()).is_some());
    }

    #[test]
    fn aiding_handler_drains_and_applies_in_timestamp_order() {
        // Drive only the handler directly so we can inspect the estimator's
        // call counts without needing access through Box<dyn ...>.
        let mut estimator = MockEstimator::new();

        let handler = TypedAidingHandler::<LinearAcceleration3D>::new(
            accel_sensor_channel(),
            Box::new(OnePassModel),
            DMatrix::identity(3, 3),
        );

        let bus = make_bus(vec![]);
        let readings = vec![
            SensorReading {
                sensor_handle: FrameHandle(1),
                timestamp: MonotonicTime(2.0),
                data: LinearAcceleration3D::default(),
            },
            SensorReading {
                sensor_handle: FrameHandle(1),
                timestamp: MonotonicTime(1.0),
                data: LinearAcceleration3D::default(),
            },
        ];
        bus.write(
            accel_channel(),
            Stamped {
                value: readings,
                timestamp: MonotonicTime(2.0),
                health: Health::Ok,
                producer: 99,
            },
        )
        .unwrap();

        handler.drain_and_apply(&bus, &mut estimator, None);

        let counts = estimator.counts.lock().unwrap();
        assert_eq!(counts.update_calls, 2, "two readings → two update calls");
    }

    #[test]
    fn aiding_handler_no_op_on_empty_channel() {
        let mut estimator = MockEstimator::new();
        let handler = TypedAidingHandler::<LinearAcceleration3D>::new(
            accel_sensor_channel(),
            Box::new(OnePassModel),
            DMatrix::identity(3, 3),
        );
        let bus = make_bus(vec![]);
        handler.drain_and_apply(&bus, &mut estimator, None);
        assert_eq!(estimator.counts.lock().unwrap().update_calls, 0);
    }
}
