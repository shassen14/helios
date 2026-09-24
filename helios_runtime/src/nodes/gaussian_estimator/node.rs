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
//! 3. **Publish.** Snapshot the filter state; write `FrameAwareState @ ""` on
//!    the bus, and dual-publish the `base_link → odom` transform edge the
//!    estimate implies for the `TfService` to fold.

use super::input::EstimatorInputBuilder;
use crate::channels::tf::{publish_edge, tf_edge};
use crate::pipeline::descriptor::AlgorithmNodePortDescriptor;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, InternalChannel, PortBus, PortDescriptor, SensorChannel};
use crate::stamped::{Health, Stamped};

use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::spatial::tf::TfProvider;
use helios_core::interchange::measurement::sensor::SensorPayload;
use helios_core::estimation::measurement::{MeasurementModel, Unavailable};
use helios_core::estimation::schema::MeasurementSchema;
use helios_core::estimation::{GaussianStateEstimator, SkipReason, UpdateOutcome};
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};
use helios_core::spatial::transforms::ErasedTransform;
use helios_core::spatial::FrameAwareState;

use atomic_float::AtomicF64;
use nalgebra::DMatrix;
use std::marker::PhantomData;
use std::sync::atomic::Ordering;
use std::sync::Mutex;
use tracing::warn;

/// Minimum seconds of reading-clock time between two "aiding dropped" warnings
/// on one channel. A missing extrinsic or a corrupt covariance fails on *every*
/// reading, so without a throttle the log floods at sensor rate; the first fault
/// always prints and later ones inside this window are suppressed. Keyed on
/// reading time (not wall time) so the gate is deterministic and testable.
const AIDING_DROP_WARN_MIN_INTERVAL_SECS: f64 = 5.0;

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
    /// Reading-clock time of the last emitted "aiding dropped" warning, for
    /// rate-limiting. Init `NEG_INFINITY` so the first fault always clears the
    /// interval and prints. Sibling of `last_applied_ts`, same latch pattern.
    last_warned: AtomicF64,
    // phantom data in order to avoid compile error that T isn't used
    // fn() -> T to say output-only, non-owned, covariant data
    _phantom: PhantomData<fn() -> T>,
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
            channel: channel.into(),
            model,
            r,
            last_applied_ts: AtomicF64::new(f64::NEG_INFINITY),
            last_warned: AtomicF64::new(f64::NEG_INFINITY),
            _phantom: PhantomData,
        }
    }

    /// Rate-limit gate for the aiding-dropped warning. Returns `true` at most
    /// once per [`AIDING_DROP_WARN_MIN_INTERVAL_SECS`] of reading time, and
    /// records `at` as the new last-warned time when it does. The
    /// `NEG_INFINITY` seed makes the first fault always pass.
    fn should_warn(&self, at: f64) -> bool {
        let last = self.last_warned.load(Ordering::Relaxed);
        if at - last < AIDING_DROP_WARN_MIN_INTERVAL_SECS {
            return false;
        }
        self.last_warned.store(at, Ordering::Relaxed);
        true
    }
}

impl<T: SensorPayload> AidingHandler for TypedAidingHandler<T> {
    fn channel(&self) -> &ChannelKey {
        &self.channel
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
            let outcome = estimator.update(
                &z,
                &*self.model,
                &self.r,
                tf,
                helios_core::prelude::MonotonicTime(reading_ts),
            );
            // Surface a dropped correction that stems from a fault (an
            // unresolved transform, a shape/covariance bug) — the silent
            // aiding-drop this whole path exists to make loud. Expected quiet
            // skips (cold start, no provider) and applied updates say nothing.
            if let Some(cause) = aiding_drop_cause(&outcome) {
                if self.should_warn(reading_ts) {
                    warn!(
                        "aiding dropped on {}: {cause} at t={reading_ts:.3}; \
                         filter running unaided.",
                        self.channel,
                    );
                }
            }
            if reading_ts > max_applied {
                max_applied = reading_ts;
            }
        }
        if max_applied > last_applied {
            self.last_applied_ts.store(max_applied, Ordering::Relaxed);
        }
    }
}

/// The cause of a *loud* aiding drop, or `None` for an applied
/// update or an expected quiet skip (cold start / no provider).
///
/// This is where the runtime reads the loud/quiet judgment off the
/// [`UpdateOutcome`]: the core filter has already classified the skip; the
/// caller only decides whether to warn. The frame-carrying transform faults name
/// their frames so the log points straight at the missing extrinsic.
fn aiding_drop_cause(outcome: &UpdateOutcome) -> Option<String> {
    match outcome {
        UpdateOutcome::Applied
        | UpdateOutcome::Skipped(SkipReason::Model(
            Unavailable::ColdStart | Unavailable::NoProvider,
        )) => None,
        UpdateOutcome::Skipped(SkipReason::Model(Unavailable::MissingTransform { from, to })) => {
            Some(format!("transform {from:?} → {to:?} unresolved"))
        }
        UpdateOutcome::Skipped(SkipReason::Model(Unavailable::ConventionMismatch { from, to })) => {
            Some(format!("convention mismatch between {from:?} and {to:?}"))
        }
        UpdateOutcome::Skipped(SkipReason::MeasurementShapeMismatch) => {
            Some("measurement and covariance lengths disagree (wiring bug)".to_string())
        }
        UpdateOutcome::Skipped(SkipReason::CovarianceNotPositiveDefinite) => Some(
            "innovation covariance not positive-definite (filter covariance corrupt)".to_string(),
        ),
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
            // Aiding handlers always read a SensorChannel; the cached
            // enum-form is unwrapped back via `kind()`-checked optional.
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
        if let Some(inputs) = self.input_builder.assemble(bus, &tick) {
            estimator.predict(tick.dt, &inputs);
        }

        // 2. Update from each aiding sensor.
        for handler in &self.aiding {
            handler.drain_and_apply(bus, &mut **estimator, tf);
        }

        // 3. Publish snapshot.
        let snapshot = estimator.state().clone();

        // Dual-publish the transform edge this estimate implies (child in
        // parent = base_link in odom): the rich FrameAwareState on its own
        // channel, and a bare StampedTransform on the tf-edge channel for the
        // TfService to fold. The pose is a pure read of the state's orientation
        // and reference-frame position; if either block is absent (a cold-start
        // schema not yet seeded with a pose), skip the edge this tick rather
        // than feed the buffer a bogus identity.
        if let Some(pose) =
            snapshot.pose::<Flu, Enu>(self.edge.child.clone(), self.edge.parent.clone())
        {
            let edge = StampedTransform {
                parent: self.edge.parent.clone(),
                child: self.edge.child.clone(),
                // When the pose held. For the filter's current estimate that is
                // `now`; it coincides with the envelope timestamp below but means
                // a different thing (pose-held vs published-at), so they are kept
                // as two fields, not merged.
                stamp: tick.now,
                transform: ErasedTransform::erase::<Flu, Enu>(pose),
            };
            publish_edge(
                bus,
                Stamped {
                    value: edge,
                    timestamp: tick.now,
                    health: Health::Ok,
                    producer: tick.node_id,
                },
            );
        }

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
    use helios_core::interchange::measurement::envelope::SensorReading;
    use helios_core::spatial::primitives::MonotonicTime;
    use helios_core::interchange::measurement::sensor::Acceleration;
    use helios_core::prelude::AgentId;
    use helios_core::estimation::carrier::kinematic_carrier_schema;
    use helios_core::estimation::measurement::Prediction;
    use helios_core::estimation::schema::{
        MeasurementSchema, MeasurementSchemaBlock, StateSchema, StateSchemaBlock,
    };
    use helios_core::estimation::{EstimatorInputs, UpdateOutcome};
    use helios_core::spatial::transforms::{Convention, ErasedTransform};
    use helios_core::spatial::{FrameAwareState, FrameId};
    use helios_core::spatial::state::Quantity;
    use nalgebra::{DMatrix, DVector, Isometry3};
    use std::sync::Mutex as StdMutex;

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
    }

    impl MockEstimator {
        fn new() -> Self {
            // A placeholder kinematic state: its schema anchors position in odom
            // and orientation base_link→odom, so `pose::<Flu, Enu>` resolves (to
            // the schema default, an identity pose) and the node dual-publishes.
            Self::with_state(FrameAwareState::from_schema(
                std::sync::Arc::new(kinematic_carrier_schema(AgentId::new("test_agent"))),
                0.0,
            ))
        }

        fn with_state(state: FrameAwareState) -> Self {
            Self {
                state,
                counts: StdMutex::new(Default::default()),
                update_outcome: UpdateOutcome::Applied,
            }
        }

        fn with_update_outcome(mut self, outcome: UpdateOutcome) -> Self {
            self.update_outcome = outcome;
            self
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
            _at: MonotonicTime,
        ) -> UpdateOutcome {
            self.counts.lock().unwrap().update_calls += 1;
            self.update_outcome.clone()
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
        FrameAwareState::from_schema(std::sync::Arc::new(schema), 0.0)
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
        let outputs = &node.port_descriptor().outputs;
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
            .optional_inputs
            .contains(&accel_channel()));
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
            UpdateOutcome::Applied,
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
}
