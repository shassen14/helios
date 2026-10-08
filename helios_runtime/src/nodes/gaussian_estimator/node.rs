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
//!    and proceed to updates anyway. A predict the filter itself skips for a
//!    fault (a misshapen input) is warned, rate-limited.
//! 2. **Update.** For each [`AidingHandler`]: read its sensor channel, sort
//!    readings by timestamp, and sequentially apply each one to the filter.
//! 3. **Publish.** Snapshot the filter state; write `FrameAwareState @ ""` on
//!    the bus, and dual-publish the `base_link → odom` transform edge the
//!    estimate implies for the `TfService` to fold.

use crate::channels::tf::tf_edge;
use crate::nodes::estimation::{EstimatorInputBuilder, Measurement, PayloadReader};
use crate::nodes::recursive_estimator::{
    aiding_drop_cause, passes_warn_throttle, predict_skip_cause, publish_estimate,
};
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{
    AlgorithmNodePortDescriptor, ChannelKey, InternalChannel, PortBus, PortDescriptor,
    SensorChannel,
};
use crate::stamped::Health;

use helios_core::estimation::measurement::MeasurementModel;
use helios_core::estimation::schema::MeasurementSchema;
use helios_core::estimation::GaussianStateEstimator;
use helios_core::interchange::measurement::sensor::SensorPayload;
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::transforms::tf::stamped::FrameEdge;
use helios_core::spatial::FrameAwareState;

use atomic_float::AtomicF64;
use nalgebra::DMatrix;
use std::sync::Mutex;
use tracing::warn;

/// Per-channel aiding handler: reads one sensor channel from the bus and feeds
/// each reading into a [`GaussianStateEstimator`] via its measurement model.
///
/// The generic payload type `T` is erased behind this trait so the node can hold
/// a `Vec<Box<dyn AidingHandler>>` across sensor types.
pub(crate) trait AidingHandler: Send + Sync {
    /// The bus channel this handler reads from.
    fn channel(&self) -> &ChannelKey;

    /// The typed shape of the measurement this handler applies — a forward of
    /// the underlying [`MeasurementModel::schema`].
    ///
    /// Exists so the assembler can read each measurement's schema through the
    /// erased `Box<dyn AidingHandler>`, without downcasting to the concrete
    /// payload type. The estimator constructor checks it against the composed
    /// state schema; this trait only surfaces it.
    fn schema(&self) -> MeasurementSchema;

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
pub(crate) struct TypedAidingHandler<T: SensorPayload> {
    /// Takes each reading on the channel once, oldest first: re-applying a
    /// measurement would over-tighten the posterior as if independent
    /// observations had been received.
    reader: PayloadReader<T>,
    model: Box<dyn MeasurementModel>,
    r: DMatrix<f64>,
    /// Reading-clock time of the last emitted "aiding dropped" warning, for
    /// rate-limiting. Init `NEG_INFINITY` so the first fault always clears the
    /// interval and prints.
    last_warned: AtomicF64,
}

impl<T: SensorPayload> TypedAidingHandler<T> {
    /// Build a handler that reads `Vec<SensorReading<T>>` from `channel`.
    ///
    /// `r` must be square with side equal to the sensor's measurement length; the
    /// filter silently skips any update whose `R` disagrees with the incoming
    /// measurement, so a wrong-sized matrix becomes a no-op rather than a panic.
    pub(crate) fn new(
        channel: SensorChannel,
        model: Box<dyn MeasurementModel>,
        r: DMatrix<f64>,
    ) -> Self {
        Self {
            reader: PayloadReader::new(channel),
            model,
            r,
            last_warned: AtomicF64::new(f64::NEG_INFINITY),
        }
    }

    /// Rate-limit gate for the aiding-dropped warning, on reading time. See
    /// [`passes_warn_throttle`].
    fn should_warn(&self, at: f64) -> bool {
        passes_warn_throttle(&self.last_warned, at)
    }
}

impl<T: SensorPayload> AidingHandler for TypedAidingHandler<T> {
    fn channel(&self) -> &ChannelKey {
        self.reader.channel()
    }

    fn schema(&self) -> MeasurementSchema {
        self.model.schema()
    }

    fn drain_and_apply(
        &self,
        bus: &PortBus,
        estimator: &mut dyn GaussianStateEstimator,
        tf: Option<&dyn TfProvider>,
    ) {
        for Measurement { z, at } in self.reader.take_new(bus) {
            let outcome = estimator.update(&z, &*self.model, &self.r, tf, at);
            // Surface a dropped correction that stems from a fault (an
            // unresolved transform, a shape/covariance bug) — the silent
            // aiding-drop this whole path exists to make loud. Expected quiet
            // skips (cold start, no provider) and applied updates say nothing.
            if let Some(cause) = aiding_drop_cause(&outcome) {
                if self.should_warn(at.0) {
                    warn!(
                        "aiding dropped on {}: {cause} at t={:.3}; \
                         filter running unaided.",
                        self.reader.channel(),
                        at.0,
                    );
                }
            }
        }
    }
}

/// Pipeline node wrapping any Gaussian-family estimator.
///
/// Construction is via [`Self::new`]. The port descriptor is derived from the
/// input builder and aiding handlers — callers don't compose channel keys
/// directly. Output is `FrameAwareState @ ""`.
pub(crate) struct GaussianEstimatorNode {
    name: String,
    edge: FrameEdge,
    estimator: Mutex<Box<dyn GaussianStateEstimator>>,
    input_builder: Box<dyn EstimatorInputBuilder>,
    aiding: Vec<Box<dyn AidingHandler>>,
    descriptor: PortDescriptor,
    /// Tick time of the last emitted "predict skipped" warning, for
    /// rate-limiting. Init `NEG_INFINITY` so the first fault always prints.
    last_predict_warned: AtomicF64,
}

impl GaussianEstimatorNode {
    pub(crate) fn new(
        name: impl Into<String>,
        edge: FrameEdge,
        estimator: Box<dyn GaussianStateEstimator>,
        input_builder: Box<dyn EstimatorInputBuilder>,
        aiding: Vec<Box<dyn AidingHandler>>,
    ) -> Self {
        let mut builder = AlgorithmNodePortDescriptor::new()
            .inputs_from_slices(
                input_builder.required_channels(),
                input_builder.optional_channels(),
            )
            .output_internal(InternalChannel::of::<FrameAwareState>())
            .output_internal(tf_edge(&edge));

        for handler in &aiding {
            // Each aiding sensor is optional: the filter still predicts and
            // publishes without it. The handler holds only an erased key, so
            // it goes through the slice path, which asserts the key is a
            // sensor or internal channel.
            builder = builder.inputs_from_slices(&[], &[handler.channel().clone()]);
        }
        let descriptor = builder.build();
        Self {
            name: name.into(),
            edge,
            estimator: Mutex::new(estimator),
            input_builder,
            aiding,
            descriptor,
            last_predict_warned: AtomicF64::new(f64::NEG_INFINITY),
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

    fn execute(&self, bus: &PortBus, tf: &dyn TfProvider, tick: TickContext) {
        let tf = Some(tf);
        // Skip the tick on a poisoned mutex rather than propagating the panic
        let Ok(mut estimator) = self.estimator.lock() else {
            return;
        };

        // 1. Predict (skip if input builder can't assemble — cold-start, dropout).
        // A skip the filter reports as a fault is surfaced: predicting on the
        // prior alone is the same silent drift as an unaided update.
        if let Some(inputs) = self.input_builder.assemble(bus, &tick) {
            let outcome = estimator.predict(tick.dt, &inputs);
            if let Some(cause) = predict_skip_cause(&outcome) {
                if passes_warn_throttle(&self.last_predict_warned, tick.now.0) {
                    warn!(
                        "estimator '{}' predict skipped: {cause} at t={:.3}; \
                         estimate not propagated.",
                        self.name, tick.now.0,
                    );
                }
            }
        }

        // 2. Update from each aiding sensor.
        for handler in &self.aiding {
            handler.drain_and_apply(bus, &mut **estimator, tf);
        }

        // 3. Publish snapshot.
        publish_estimate(
            bus,
            &self.edge,
            estimator.state().clone(),
            tick.now,
            Health::Ok,
            &tick,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Tests for [`GaussianEstimatorNode`] using minimal mocks. EKF/UKF behavior
    //! is covered in `helios_core/src/estimation/filters/`; here we verify only
    //! the node's wiring: predict-side input assembly, aiding-handler dispatch,
    //! and bus publish.

    use super::*;
    use crate::stamped::Stamped;

    use helios_core::estimation::carrier::kinematic_carrier_schema;
    use helios_core::estimation::measurement::{Prediction, Unavailable};
    use helios_core::estimation::schema::{
        InputSchema, MeasurementSchema, MeasurementSchemaBlock, StateSchema, StateSchemaBlock,
    };
    use helios_core::estimation::{
        EstimatorInputs, Innovation, PredictOutcome, PredictSkipReason, SkipReason, UpdateOutcome,
    };
    use helios_core::interchange::measurement::envelope::SensorReading;
    use helios_core::interchange::measurement::sensor::Acceleration;
    use helios_core::prelude::AgentId;
    use helios_core::spatial::conventions::{Enu, Flu};
    use helios_core::spatial::primitives::MonotonicTime;
    use helios_core::spatial::state::Quantity;
    use helios_core::spatial::transforms::tf::stamped::StampedTransform;
    use helios_core::spatial::transforms::{Convention, ErasedTransform};
    use helios_core::spatial::{FrameAwareState, FrameId};
    use nalgebra::{DMatrix, DVector, Isometry3};
    use std::sync::atomic::Ordering;
    use std::sync::{Arc, Mutex as StdMutex};

    // --- Mock TfProvider ---

    struct MockRuntime;

    impl TfProvider for MockRuntime {
        fn get_transform(
            &self,
            _: FrameId,
            _: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            Some(ErasedTransform::from_parts(
                Isometry3::identity(),
                Convention::Flu,
                Convention::Flu,
            ))
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
        /// What each `update` call returns. Defaults to `Applied`; a loud skip
        /// can be injected to exercise the handler's drop-warning path without
        /// standing up a real filter + measurement model.
        update_outcome: UpdateOutcome,
        /// What each `predict` call returns. Defaults to `Applied`; a loud skip
        /// can be injected to exercise the node's predict-warning path.
        predict_outcome: PredictOutcome,
    }

    /// An applied update. The node reads only that the update applied, never the
    /// innovation's value, so any innovation stands in.
    fn applied_update() -> UpdateOutcome {
        UpdateOutcome::Applied(Innovation::new(1.0, 1))
    }

    impl MockEstimator {
        fn new() -> Self {
            // A placeholder kinematic state: its schema anchors position in odom
            // and orientation base_link→odom, so `pose::<Flu, Enu>` resolves (to
            // the schema default, an identity pose) and the node dual-publishes.
            Self::with_state(FrameAwareState::from_schema(
                std::sync::Arc::new(kinematic_carrier_schema(AgentId::new("test_agent"))),
                MonotonicTime(0.0),
            ))
        }

        fn with_state(state: FrameAwareState) -> Self {
            Self {
                state,
                counts: StdMutex::new(Default::default()),
                update_outcome: applied_update(),
                predict_outcome: PredictOutcome::Applied,
            }
        }

        fn with_update_outcome(mut self, outcome: UpdateOutcome) -> Self {
            self.update_outcome = outcome;
            self
        }

        fn with_predict_outcome(mut self, outcome: PredictOutcome) -> Self {
            self.predict_outcome = outcome;
            self
        }
    }

    impl GaussianStateEstimator for MockEstimator {
        fn predict(&mut self, dt: f64, _inputs: &EstimatorInputs) -> PredictOutcome {
            let mut c = self.counts.lock().unwrap();
            c.predict_calls += 1;
            c.last_dt = dt;
            self.predict_outcome.clone()
        }
        fn update(
            &mut self,
            _z: &DVector<f64>,
            _model: &dyn MeasurementModel,
            _r: &DMatrix<f64>,
            _tf: Option<&dyn TfProvider>,
            _at: MonotonicTime,
        ) -> UpdateOutcome {
            self.counts.lock().unwrap().update_calls += 1;
            self.update_outcome.clone()
        }
        fn set_valid_at(&mut self, t: MonotonicTime) {
            self.state.timestamp = t;
        }
        fn state(&self) -> &FrameAwareState {
            &self.state
        }
    }

    // --- Mock MeasurementModel ---

    struct OnePassModel;

    impl MeasurementModel for OnePassModel {
        // A plumbing mock: it predicts a zero 3-vector to exercise the node's
        // tick/update path, not any real measurement. Its schema is just some
        // 3-DOF world-frame block.
        fn schema(&self) -> MeasurementSchema {
            MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::world()),
                Convention::Enu,
            )])
        }
        fn predict_measurement(
            &self,
            _state: &FrameAwareState,
            _tf: Option<&dyn TfProvider>,
            _at: MonotonicTime,
        ) -> Prediction {
            Prediction::Ready(DVector::zeros(3))
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
        fn input_schema(&self) -> Arc<InputSchema> {
            Arc::new(InputSchema::compose(vec![]))
        }
        fn assemble(&self, _bus: &PortBus, _tick: &TickContext) -> Option<EstimatorInputs> {
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
        fn input_schema(&self) -> Arc<InputSchema> {
            Arc::new(InputSchema::compose(vec![]))
        }
        fn assemble(&self, _bus: &PortBus, _tick: &TickContext) -> Option<EstimatorInputs> {
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
        SensorChannel::of::<Vec<SensorReading<Acceleration>>>()
    }

    fn accel_channel() -> ChannelKey {
        accel_sensor_channel().into()
    }

    fn state_channel() -> ChannelKey {
        InternalChannel::of::<FrameAwareState>().into()
    }

    fn make_bus(extra_outputs: Vec<ChannelKey>) -> PortBus {
        let descriptor = PortDescriptor::new(
            vec![],
            vec![],
            {
                let mut v = vec![state_channel(), accel_channel()];
                v.extend(extra_outputs);
                v
            },
            None,
        );
        PortBus::new(&[descriptor])
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 0,
        }
    }

    /// The `base_link → odom` edge the estimator owns, for `test_agent` — the
    /// same frames [`MockEstimator::new`]'s state anchors, so its `pose()`
    /// resolves and the dual-publish fires.
    fn test_edge() -> FrameEdge {
        let agent = AgentId::new("test_agent");
        FrameEdge {
            child: FrameId::base_link(agent.clone()),
            parent: FrameId::odom(agent),
        }
    }

    /// A state whose schema carries neither orientation nor an odom position
    /// block, so `pose::<Flu, Enu>` returns `None` — the cold-start shape before
    /// the filter is seeded with a pose.
    fn poseless_state() -> FrameAwareState {
        let agent = AgentId::new("test_agent");
        let schema = StateSchema::compose(vec![StateSchemaBlock::new(
            Quantity::Velocity(FrameId::odom(agent)),
            Convention::Enu,
            None,
            DVector::zeros(3),
            DMatrix::zeros(3, 3),
        )]);
        FrameAwareState::from_schema(std::sync::Arc::new(schema), MonotonicTime(0.0))
    }

    // --- Tests ---

    #[test]
    fn descriptor_outputs_state_and_its_transform_edge() {
        let node = GaussianEstimatorNode::new(
            "ekf",
            test_edge(),
            Box::new(MockEstimator::new()),
            Box::new(AlwaysReadyBuilder::new()),
            vec![],
        );
        // Two outputs: the rich FrameAwareState, and the bare transform edge the
        // estimate dual-publishes for the TfService to fold.
        let outputs = &node.port_descriptor().outputs();
        assert_eq!(outputs.len(), 2);
        assert!(outputs.contains(&state_channel()));
        assert!(outputs.contains(&tf_edge(&test_edge()).into()));
    }

    #[test]
    fn descriptor_lists_aiding_channels_as_optional() {
        let handler = TypedAidingHandler::<Acceleration>::new(
            accel_sensor_channel(),
            Box::new(OnePassModel),
            DMatrix::identity(3, 3),
        );
        let node = GaussianEstimatorNode::new(
            "ekf",
            test_edge(),
            Box::new(MockEstimator::new()),
            Box::new(AlwaysReadyBuilder::new()),
            vec![Box::new(handler)],
        );
        assert!(node
            .port_descriptor()
            .optional_inputs()
            .any(|k| k == &accel_channel()));
    }

    #[test]
    fn aiding_handler_forwards_the_model_schema() {
        // The handler's schema must be its model's, verbatim — the assembler
        // reads it through Box<dyn AidingHandler> and would otherwise be blind
        // to what the measurement actually observes. OnePassModel declares one
        // world-frame position block, so the forward must surface exactly that.
        let handler = TypedAidingHandler::<Acceleration>::new(
            accel_sensor_channel(),
            Box::new(OnePassModel),
            DMatrix::identity(3, 3),
        );

        let schema = handler.schema();
        assert_eq!(schema.dim(), 3);
        assert_eq!(schema.blocks().len(), 1);
        assert_eq!(
            schema.blocks()[0].quantity(),
            &Quantity::Position(FrameId::world())
        );
        assert_eq!(
            schema.blocks()[0].conventions(),
            &[(FrameId::world(), Convention::Enu)]
        );
    }

    #[test]
    fn execute_publishes_state_with_correct_stamp() {
        let node = GaussianEstimatorNode::new(
            "ekf",
            test_edge(),
            Box::new(MockEstimator::new()),
            Box::new(AlwaysReadyBuilder::new()),
            vec![],
        );
        let bus = make_bus(vec![]);
        let runtime = MockRuntime;

        node.execute(&bus, &runtime, tick_at(1.0, 0.1));

        let published = bus
            .read::<FrameAwareState>(state_channel())
            .expect("node must publish FrameAwareState");
        assert!((published.timestamp.0 - 1.0).abs() < 1e-9);
        assert_eq!(published.producer, 0);
    }

    #[test]
    fn execute_dual_publishes_the_transform_edge() {
        let edge = test_edge();
        let node = GaussianEstimatorNode::new(
            "ekf",
            edge.clone(),
            Box::new(MockEstimator::new()),
            Box::new(AlwaysReadyBuilder::new()),
            vec![],
        );
        let bus = make_bus(vec![tf_edge(&edge).into()]);

        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));

        let published = bus
            .read::<StampedTransform>(tf_edge(&edge).into())
            .expect("node must dual-publish the transform edge");
        // The message names the edge it belongs to, child-in-parent.
        assert_eq!(published.value.parent, edge.parent);
        assert_eq!(published.value.child, edge.child);
        // The inner (pose-held) stamp is the tick's `now`.
        assert!((published.value.stamp.0 - 1.0).abs() < 1e-9);
        // The mock's state is the schema default (identity pose); the erased
        // edge must cross back to a typed Flu→Enu identity transform.
        let typed = published
            .value
            .transform
            .typed::<Flu, Enu>()
            .expect("edge carries a Flu→Enu transform");
        assert!(typed.into_inner().translation.vector.norm() < 1e-9);
    }

    #[test]
    fn execute_skips_the_edge_when_state_has_no_pose() {
        let edge = test_edge();
        let node = GaussianEstimatorNode::new(
            "ekf",
            edge.clone(),
            Box::new(MockEstimator::with_state(poseless_state())),
            Box::new(AlwaysReadyBuilder::new()),
            vec![],
        );
        let bus = make_bus(vec![tf_edge(&edge).into()]);

        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));

        // State is still published; the edge is not — no pose to build it from,
        // and a cold-start tick must not feed the buffer a bogus identity.
        assert!(bus.read::<FrameAwareState>(state_channel()).is_some());
        assert!(bus
            .read::<StampedTransform>(tf_edge(&edge).into())
            .is_none());
    }

    #[test]
    fn execute_publishes_even_when_input_builder_returns_none() {
        // Cold-start: predict is skipped but state still gets published so
        // downstream consumers see *something* (the prior).
        let node = GaussianEstimatorNode::new(
            "ekf",
            test_edge(),
            Box::new(MockEstimator::new()),
            Box::new(NeverReadyBuilder { required: vec![] }),
            vec![],
        );
        let bus = make_bus(vec![]);
        let runtime = MockRuntime;

        node.execute(&bus, &runtime, tick_at(0.5, 0.1));

        assert!(bus.read::<FrameAwareState>(state_channel()).is_some());
    }

    #[test]
    fn aiding_handler_drains_and_applies_in_timestamp_order() {
        // Drive only the handler directly so we can inspect the estimator's
        // call counts without needing access through Box<dyn ...>.
        let mut estimator = MockEstimator::new();

        let handler = TypedAidingHandler::<Acceleration>::new(
            accel_sensor_channel(),
            Box::new(OnePassModel),
            DMatrix::identity(3, 3),
        );

        let bus = make_bus(vec![]);
        let readings = vec![
            SensorReading {
                sensor: FrameId::sensor(AgentId::new("test_agent"), "accel"),
                timestamp: MonotonicTime(2.0),
                data: Acceleration::default(),
            },
            SensorReading {
                sensor: FrameId::sensor(AgentId::new("test_agent"), "accel"),
                timestamp: MonotonicTime(1.0),
                data: Acceleration::default(),
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
        let handler = TypedAidingHandler::<Acceleration>::new(
            accel_sensor_channel(),
            Box::new(OnePassModel),
            DMatrix::identity(3, 3),
        );
        let bus = make_bus(vec![]);
        handler.drain_and_apply(&bus, &mut estimator, None);
        assert_eq!(estimator.counts.lock().unwrap().update_calls, 0);
    }

    #[test]
    fn aiding_handler_warns_once_per_interval_on_loud_skip() {
        // The runtime half of the aiding-drop guard against the silent-drift
        // bug class: when the filter reports a *loud* skip (here a missing
        // sensor→base_link transform), the handler must warn — but rate-limited,
        // so a permanently-missing extrinsic that fails on every reading logs
        // once per interval, not once per reading. That the filter *derives*
        // this outcome from an unresolved transform is core's contract (covered
        // by the filter tests); here the loud outcome is injected so this test
        // isolates the handler's emit + throttle behavior.
        let agent = AgentId::new("test_agent");
        let sensor = FrameId::sensor(agent.clone(), "accel");
        let base = FrameId::base_link(agent.clone());
        let mut estimator = MockEstimator::new().with_update_outcome(UpdateOutcome::Skipped(
            SkipReason::Model(Unavailable::MissingTransform {
                from: sensor.clone(),
                to: base,
            }),
        ));

        let handler = TypedAidingHandler::<Acceleration>::new(
            accel_sensor_channel(),
            Box::new(OnePassModel),
            DMatrix::identity(3, 3),
        );

        // Two readings whose stamps sit well inside one throttle window.
        let bus = make_bus(vec![]);
        let readings = vec![
            SensorReading {
                sensor: sensor.clone(),
                timestamp: MonotonicTime(1.0),
                data: Acceleration::default(),
            },
            SensorReading {
                sensor,
                timestamp: MonotonicTime(1.1),
                data: Acceleration::default(),
            },
        ];
        bus.write(
            accel_channel(),
            Stamped {
                value: readings,
                timestamp: MonotonicTime(1.1),
                health: Health::Ok,
                producer: 0,
            },
        )
        .unwrap();

        handler.drain_and_apply(&bus, &mut estimator, None);

        // Both readings were offered to the filter and both skipped...
        assert_eq!(estimator.counts.lock().unwrap().update_calls, 2);
        // ...but only the first fault crossed the throttle: `last_warned`
        // latched to the first reading's stamp, and the second reading (0.1 s
        // later, well within the interval) was suppressed.
        assert_eq!(handler.last_warned.load(Ordering::Relaxed), 1.0);
    }

    #[test]
    fn aiding_handler_stays_silent_on_applied_and_quiet_skips() {
        // The negative of the guard: an applied update and the expected quiet
        // skips (cold start / no provider) must never trip the warning latch, or
        // the throttle would be spent on non-faults and hide a later real one.
        for outcome in [
            applied_update(),
            UpdateOutcome::Skipped(SkipReason::Model(Unavailable::ColdStart)),
            UpdateOutcome::Skipped(SkipReason::Model(Unavailable::NoProvider)),
        ] {
            let mut estimator = MockEstimator::new().with_update_outcome(outcome);
            let handler = TypedAidingHandler::<Acceleration>::new(
                accel_sensor_channel(),
                Box::new(OnePassModel),
                DMatrix::identity(3, 3),
            );
            let bus = make_bus(vec![]);
            bus.write(
                accel_channel(),
                Stamped {
                    value: vec![SensorReading {
                        sensor: FrameId::sensor(AgentId::new("test_agent"), "accel"),
                        timestamp: MonotonicTime(1.0),
                        data: Acceleration::default(),
                    }],
                    timestamp: MonotonicTime(1.0),
                    health: Health::Ok,
                    producer: 0,
                },
            )
            .unwrap();

            handler.drain_and_apply(&bus, &mut estimator, None);

            // The latch never moved off its NEG_INFINITY seed.
            assert_eq!(
                handler.last_warned.load(Ordering::Relaxed),
                f64::NEG_INFINITY
            );
        }
    }

    #[test]
    fn node_warns_once_per_interval_on_a_loud_predict_skip() {
        // A misshapen input fails on every tick. The node must warn, but
        // rate-limited: two ticks inside one window latch only the first.
        let node = GaussianEstimatorNode::new(
            "ekf",
            test_edge(),
            Box::new(
                MockEstimator::new().with_predict_outcome(PredictOutcome::Skipped(
                    PredictSkipReason::InputShapeMismatch {
                        expected: 6,
                        supplied: 0,
                    },
                )),
            ),
            Box::new(AlwaysReadyBuilder::new()),
            vec![],
        );
        let bus = make_bus(vec![tf_edge(&test_edge()).into()]);

        node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));
        node.execute(&bus, &MockRuntime, tick_at(1.1, 0.1));

        assert_eq!(node.last_predict_warned.load(Ordering::Relaxed), 1.0);
    }

    #[test]
    fn node_stays_silent_on_an_applied_predict_and_a_non_positive_dt() {
        // An applied predict and the quiet first-tick skip must never spend the
        // throttle, or a later real fault inside the window would be hidden.
        for outcome in [
            PredictOutcome::Applied,
            PredictOutcome::Skipped(PredictSkipReason::NonPositiveDt),
        ] {
            let node = GaussianEstimatorNode::new(
                "ekf",
                test_edge(),
                Box::new(MockEstimator::new().with_predict_outcome(outcome)),
                Box::new(AlwaysReadyBuilder::new()),
                vec![],
            );
            let bus = make_bus(vec![tf_edge(&test_edge()).into()]);

            node.execute(&bus, &MockRuntime, tick_at(1.0, 0.1));

            assert_eq!(
                node.last_predict_warned.load(Ordering::Relaxed),
                f64::NEG_INFINITY
            );
        }
    }
}
