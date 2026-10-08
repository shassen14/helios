//! [`RecursiveEstimatorNode`]: the predict → update → publish loop around a
//! recursive filter, whichever filter, dynamics and measurement models it was
//! built from.
//!
//! ## Each tick
//!
//! 1. **Predict.** The input builder assembles the control vector from the
//!    bus; if it can't yet (cold start, dropout), predict is skipped and the
//!    tick goes on to the updates. A predict the filter itself skips for a
//!    fault (a misshapen input) is warned, rate-limited.
//! 2. **Update.** Each aiding source hands over the readings it has not handed
//!    over before, oldest first, and each is applied to the filter. A dropped
//!    correction that stems from a fault is warned, rate-limited per source.
//! 3. **Publish.** The filter's state goes out as `FrameAwareState @ ""`, and
//!    the `base_link → odom` edge it implies goes to the TF service.
//!
//! Predict uses the tick's `dt` and everything is stamped with the tick's
//! time: the pipeline clock, not the state's own.

use crate::channels::tf::{publish_edge, tf_edge};
use crate::nodes::estimation::{EstimatorInputBuilder, Measurement, MeasurementSource};
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{
    AlgorithmNodePortDescriptor, ChannelError, ChannelKey, InternalChannel, PortBus, PortDescriptor,
};
use crate::stamped::{Health, Stamped};

use helios_core::estimation::measurement::Unavailable;
use helios_core::estimation::{
    GaussianStateEstimator, PredictOutcome, PredictSkipReason, SkipReason, UpdateOutcome,
};
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::transforms::tf::stamped::{FrameEdge, StampedTransform};
use helios_core::spatial::transforms::ErasedTransform;
use helios_core::spatial::FrameAwareState;

use atomic_float::AtomicF64;
use std::sync::atomic::Ordering;
use std::sync::Mutex;
use tracing::warn;

/// Minimum seconds of pipeline-clock time between two skip warnings from one
/// source: an aiding source's "aiding dropped", or the node's "predict
/// skipped". A missing extrinsic, a corrupt covariance or a misshapen input
/// fails on *every* reading or tick, so without a throttle the log floods at
/// sensor rate; the first fault always prints and later ones inside this window
/// are suppressed. Keyed on reading or tick time (not wall time) so the gate is
/// deterministic and testable.
pub(crate) const SKIP_WARN_MIN_INTERVAL_SECS: f64 = 5.0;

/// A recursive filter run as a pipeline node.
///
/// The port descriptor is derived from the input builder and the aiding
/// sources: the builder's channels as it declares them, each aiding channel
/// optional, since the filter still predicts and publishes without it.
pub(crate) struct RecursiveEstimatorNode {
    name: String,
    edge: FrameEdge,
    filter: Mutex<Box<dyn GaussianStateEstimator>>,
    input: Box<dyn EstimatorInputBuilder>,
    aiding: Vec<Aiding>,
    descriptor: PortDescriptor,
    /// Tick time of the last "predict skipped" warning; `NEG_INFINITY` so the
    /// first fault always prints.
    last_predict_warned: AtomicF64,
}

impl RecursiveEstimatorNode {
    /// A node named `name` that runs `filter`, predicting from what `input`
    /// assembles and correcting from each of `aiding`, in order. `edge` is the
    /// transform edge the estimate implies.
    pub(crate) fn new(
        name: impl Into<String>,
        edge: FrameEdge,
        filter: Box<dyn GaussianStateEstimator>,
        input: Box<dyn EstimatorInputBuilder>,
        aiding: Vec<Box<dyn MeasurementSource>>,
    ) -> Self {
        let mut builder = AlgorithmNodePortDescriptor::new()
            .inputs_from_slices(input.required_channels(), input.optional_channels())
            .output_internal(InternalChannel::of::<FrameAwareState>())
            .output_internal(tf_edge(&edge));
        for source in &aiding {
            // The source holds only an erased key, so it goes through the
            // slice path, which asserts the key is a sensor or internal
            // channel.
            builder = builder.inputs_from_slices(&[], &[source.channel().clone()]);
        }

        Self {
            name: name.into(),
            edge,
            filter: Mutex::new(filter),
            input,
            aiding: aiding.into_iter().map(Aiding::new).collect(),
            descriptor: builder.build(),
            last_predict_warned: AtomicF64::new(f64::NEG_INFINITY),
        }
    }
}

impl PipelineNode for RecursiveEstimatorNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, tf: &dyn TfProvider, tick: TickContext) {
        // Skip the tick on a poisoned mutex rather than propagating the panic.
        let Ok(mut filter) = self.filter.lock() else {
            return;
        };

        // 1. Predict. A skip the filter reports as a fault is surfaced:
        // predicting on the prior alone is the same silent drift as an
        // unaided update.
        if let Some(inputs) = self.input.assemble(bus, &tick) {
            let outcome = filter.predict(tick.dt, &inputs);
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

        // 2. Update from each aiding source.
        for aiding in &self.aiding {
            aiding.apply_new(bus, &mut **filter, Some(tf));
        }

        // 3. Publish.
        publish_estimate(bus, &self.edge, filter.state().clone(), &tick);
    }
}

/// One aiding source and the throttle on its drop warnings.
struct Aiding {
    source: Box<dyn MeasurementSource>,
    /// Reading time of the last "aiding dropped" warning; `NEG_INFINITY` so
    /// the first fault always prints.
    last_warned: AtomicF64,
}

impl Aiding {
    fn new(source: Box<dyn MeasurementSource>) -> Self {
        Self {
            source,
            last_warned: AtomicF64::new(f64::NEG_INFINITY),
        }
    }

    /// Applies every reading the source has not handed over before, oldest
    /// first, warning on a correction dropped for a fault.
    fn apply_new(
        &self,
        bus: &PortBus,
        filter: &mut dyn GaussianStateEstimator,
        tf: Option<&dyn TfProvider>,
    ) {
        for Measurement { z, at } in self.source.take_new(bus) {
            let outcome = filter.update(&z, self.source.model(), self.source.noise(), tf, at);
            // Expected quiet skips (cold start, no provider) and applied
            // updates say nothing.
            if let Some(cause) = aiding_drop_cause(&outcome) {
                if passes_warn_throttle(&self.last_warned, at.0) {
                    warn!(
                        "aiding dropped on {}: {cause} at t={:.3}; \
                         filter running unaided.",
                        self.source.channel(),
                        at.0,
                    );
                }
            }
        }
    }
}

/// Writes `state` as `FrameAwareState @ ""` and the `edge` transform it
/// implies, both stamped with the tick's time.
///
/// The edge is a pure read of the state's orientation and reference-frame
/// position. If either block is absent (a schema not yet seeded with a pose),
/// the edge is skipped this tick rather than feeding the TF buffer a bogus
/// identity.
pub(crate) fn publish_estimate(
    bus: &PortBus,
    edge: &FrameEdge,
    state: FrameAwareState,
    tick: &TickContext,
) {
    if let Some(pose) = state.pose::<Flu, Enu>(edge.child.clone(), edge.parent.clone()) {
        let transform = StampedTransform {
            parent: edge.parent.clone(),
            child: edge.child.clone(),
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
                value: transform,
                timestamp: tick.now,
                health: Health::Ok,
                producer: tick.node_id,
            },
        );
    }

    let stamped = Stamped {
        value: state,
        timestamp: tick.now,
        health: Health::Ok,
        producer: tick.node_id,
    };
    let state_channel: ChannelKey = InternalChannel::of::<FrameAwareState>().into();
    if let Err(ChannelError::UnknownChannel) = bus.write(state_channel.clone(), stamped) {
        warn!(
            channel = %state_channel,
            "estimator state output channel is not wired into the DAG"
        );
    }
}

/// The cause of a *loud* aiding drop, or `None` for an applied update or an
/// expected quiet skip (cold start / no provider).
///
/// The core filter has already classified the skip; the caller only decides
/// whether to warn. The frame-carrying transform faults name their frames so
/// the log points straight at the missing extrinsic.
pub(crate) fn aiding_drop_cause(outcome: &UpdateOutcome) -> Option<String> {
    match outcome {
        UpdateOutcome::Applied(_)
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
        // A reason added to core after this match was written: surface it
        // rather than guess it is harmless.
        UpdateOutcome::Skipped(reason) => Some(format!("{reason:?}")),
    }
}

/// The cause of a *loud* predict skip, or `None` for an applied predict or the
/// expected quiet skip (a non-positive step, as on the first tick).
///
/// The predict-side twin of [`aiding_drop_cause`].
pub(crate) fn predict_skip_cause(outcome: &PredictOutcome) -> Option<String> {
    match outcome {
        PredictOutcome::Applied | PredictOutcome::Skipped(PredictSkipReason::NonPositiveDt) => None,
        PredictOutcome::Skipped(PredictSkipReason::InputShapeMismatch { expected, supplied }) => {
            Some(format!(
                "input has {supplied} rows but the dynamics' input schema has {expected} \
                 (wiring bug)"
            ))
        }
        PredictOutcome::Skipped(PredictSkipReason::CovarianceNotPositiveDefinite) => {
            Some("state covariance not positive-definite (filter covariance corrupt)".to_string())
        }
        // A reason added to core after this match was written: surface it
        // rather than guess it is harmless.
        PredictOutcome::Skipped(reason) => Some(format!("{reason:?}")),
    }
}

/// Rate-limit gate shared by the skip warnings. Returns `true` at most once per
/// [`SKIP_WARN_MIN_INTERVAL_SECS`] of `at`, and records `at` in `last_warned`
/// when it does. A latch seeded with `NEG_INFINITY` lets the first fault pass.
pub(crate) fn passes_warn_throttle(last_warned: &AtomicF64, at: f64) -> bool {
    let last = last_warned.load(Ordering::Relaxed);
    if at - last < SKIP_WARN_MIN_INTERVAL_SECS {
        return false;
    }
    last_warned.store(at, Ordering::Relaxed);
    true
}

#[cfg(test)]
mod tests {
    //! The loop's wiring only: what it reads, the order it applies readings
    //! in, and what it publishes. Filter math is tested in `helios_core`.

    use super::*;

    use crate::nodes::estimation::PayloadReader;
    use crate::port::{InputPort, SensorChannel};

    use helios_core::estimation::carrier::kinematic_carrier_schema;
    use helios_core::estimation::measurement::{MeasurementModel, Prediction};
    use helios_core::estimation::schema::{InputSchema, MeasurementSchema, MeasurementSchemaBlock};
    use helios_core::estimation::{EstimatorInputs, Innovation};
    use helios_core::interchange::measurement::envelope::SensorReading;
    use helios_core::interchange::measurement::sensor::Acceleration;
    use helios_core::prelude::AgentId;
    use helios_core::spatial::primitives::MonotonicTime;
    use helios_core::spatial::state::Quantity;
    use helios_core::spatial::transforms::Convention;
    use helios_core::spatial::FrameId;

    use nalgebra::{DMatrix, DVector};
    use std::sync::{Arc, Mutex as StdMutex};

    const DT: f64 = 0.1;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn accel_channel() -> ChannelKey {
        SensorChannel::named::<Vec<SensorReading<Acceleration>>>("accel").into()
    }

    fn state_channel() -> ChannelKey {
        InternalChannel::of::<FrameAwareState>().into()
    }

    /// What the filter was asked to do, readable after the filter is boxed.
    #[derive(Default)]
    struct Calls {
        predict_dts: Vec<f64>,
        update_times: Vec<f64>,
    }

    /// A filter that applies everything and records each call.
    struct RecordingFilter {
        state: FrameAwareState,
        calls: Arc<StdMutex<Calls>>,
    }

    impl GaussianStateEstimator for RecordingFilter {
        fn predict(&mut self, dt: f64, _: &EstimatorInputs) -> PredictOutcome {
            self.calls.lock().expect("test lock").predict_dts.push(dt);
            PredictOutcome::Applied
        }

        fn update(
            &mut self,
            _: &DVector<f64>,
            _: &dyn MeasurementModel,
            _: &DMatrix<f64>,
            _: Option<&dyn TfProvider>,
            at: MonotonicTime,
        ) -> UpdateOutcome {
            self.calls
                .lock()
                .expect("test lock")
                .update_times
                .push(at.0);
            UpdateOutcome::Applied(Innovation::new(1.0, 1))
        }

        fn state(&self) -> &FrameAwareState {
            &self.state
        }
    }

    /// A 3-vector measurement model that predicts zero.
    struct ZeroModel;

    impl MeasurementModel for ZeroModel {
        fn schema(&self) -> MeasurementSchema {
            MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::world()),
                Convention::Enu,
            )])
        }

        fn predict_measurement(
            &self,
            _: &FrameAwareState,
            _: Option<&dyn TfProvider>,
            _: MonotonicTime,
        ) -> Prediction {
            Prediction::Ready(DVector::zeros(3))
        }
    }

    /// Accelerometer readings on [`accel_channel`], predicted by [`ZeroModel`].
    struct AccelSource {
        reader: PayloadReader<Acceleration>,
        model: ZeroModel,
        noise: DMatrix<f64>,
    }

    impl MeasurementSource for AccelSource {
        fn channel(&self) -> &ChannelKey {
            self.reader.channel()
        }

        fn model(&self) -> &dyn MeasurementModel {
            &self.model
        }

        fn noise(&self) -> &DMatrix<f64> {
            &self.noise
        }

        fn take_new(&self, bus: &PortBus) -> Vec<Measurement> {
            self.reader.take_new(bus)
        }
    }

    /// An input builder with nothing to read, always ready.
    struct ReadyInput;

    impl EstimatorInputBuilder for ReadyInput {
        fn input_schema(&self) -> Arc<InputSchema> {
            Arc::new(InputSchema::compose(vec![]))
        }

        fn assemble(&self, _: &PortBus, _: &TickContext) -> Option<EstimatorInputs> {
            Some(EstimatorInputs {
                control: DVector::zeros(0),
            })
        }

        fn required_channels(&self) -> &[ChannelKey] {
            &[]
        }

        fn optional_channels(&self) -> &[ChannelKey] {
            &[]
        }
    }

    /// A node over a [`RecordingFilter`] with one accelerometer source, and the
    /// record of what the filter was asked to do.
    fn node() -> (RecursiveEstimatorNode, Arc<StdMutex<Calls>>) {
        let calls = Arc::new(StdMutex::new(Calls::default()));
        let filter = RecordingFilter {
            // Anchors base_link in odom, so the state holds a pose and the
            // edge is published.
            state: FrameAwareState::from_schema(
                Arc::new(kinematic_carrier_schema(agent())),
                MonotonicTime(0.0),
            ),
            calls: Arc::clone(&calls),
        };
        let source = AccelSource {
            reader: PayloadReader::new(SensorChannel::named::<Vec<SensorReading<Acceleration>>>(
                "accel",
            )),
            model: ZeroModel,
            noise: DMatrix::identity(3, 3),
        };
        let edge = FrameEdge {
            child: FrameId::base_link(agent()),
            parent: FrameId::odom(agent()),
        };
        let node = RecursiveEstimatorNode::new(
            "primary",
            edge,
            Box::new(filter),
            Box::new(ReadyInput),
            vec![Box::new(source)],
        );
        (node, calls)
    }

    /// A bus carrying everything `node` reads and writes.
    fn bus_for(node: &RecursiveEstimatorNode) -> PortBus {
        let descriptor = node.port_descriptor();
        let mut channels: Vec<ChannelKey> = descriptor
            .inputs()
            .map(InputPort::channel)
            .cloned()
            .collect();
        channels.extend(descriptor.outputs().iter().cloned());
        PortBus::new(&[PortDescriptor::new(vec![], vec![], channels, None)])
    }

    fn write_accel(bus: &PortBus, times: &[f64]) {
        let readings = times
            .iter()
            .map(|&t| SensorReading {
                sensor: FrameId::sensor(agent(), "accel"),
                timestamp: MonotonicTime(t),
                data: Acceleration::default(),
            })
            .collect::<Vec<_>>();
        bus.write(
            accel_channel(),
            Stamped {
                value: readings,
                timestamp: MonotonicTime(times.iter().copied().fold(0.0, f64::max)),
                health: Health::Ok,
                producer: 99,
            },
        )
        .expect("the bus carries the accelerometer channel");
    }

    fn tick(now: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt: DT,
            node_id: 0,
        }
    }

    /// The filter still predicts and publishes without an aiding sensor, so
    /// each aiding channel is an optional input.
    #[test]
    fn aiding_channels_are_optional_inputs() {
        let (node, _) = node();
        let descriptor = node.port_descriptor();
        assert!(descriptor
            .optional_inputs()
            .any(|key| key == &accel_channel()));
        assert!(!descriptor
            .required_inputs()
            .any(|key| key == &accel_channel()));
    }

    /// Readings are applied oldest first, and a batch still on the channel the
    /// next tick is not applied again.
    #[test]
    fn readings_are_applied_oldest_first_and_once() {
        let (node, calls) = node();
        let bus = bus_for(&node);
        write_accel(&bus, &[2.0, 1.0]);

        node.execute(&bus, &NoTransforms, tick(2.0));
        node.execute(&bus, &NoTransforms, tick(2.1));

        assert_eq!(calls.lock().expect("test lock").update_times, [1.0, 2.0]);
    }

    /// Predict steps by the tick's `dt`, and the estimate and its edge are
    /// stamped with the tick's time.
    #[test]
    fn predict_and_publish_follow_the_tick() {
        let (node, calls) = node();
        let bus = bus_for(&node);

        node.execute(&bus, &NoTransforms, tick(3.0));

        assert_eq!(calls.lock().expect("test lock").predict_dts, [DT]);
        let state = bus
            .read::<FrameAwareState>(state_channel())
            .expect("the estimate is published");
        assert_eq!(state.timestamp, MonotonicTime(3.0));
        let edge = bus
            .read::<StampedTransform>(tf_edge(&node.edge).into())
            .expect("the edge is published");
        assert_eq!(edge.value.stamp, MonotonicTime(3.0));
    }

    /// The throttle lets the first fault through, holds the next ones for
    /// the interval, then lets one through again.
    #[test]
    fn the_warn_throttle_passes_once_per_interval() {
        let last = AtomicF64::new(f64::NEG_INFINITY);
        assert!(passes_warn_throttle(&last, 1.0));
        assert!(!passes_warn_throttle(
            &last,
            1.0 + SKIP_WARN_MIN_INTERVAL_SECS / 2.0
        ));
        assert!(passes_warn_throttle(
            &last,
            1.0 + SKIP_WARN_MIN_INTERVAL_SECS
        ));
    }

    /// Applied steps and expected skips are quiet; faults are loud.
    #[test]
    fn only_faults_are_loud() {
        assert_eq!(
            aiding_drop_cause(&UpdateOutcome::Skipped(SkipReason::Model(
                Unavailable::ColdStart
            ))),
            None
        );
        assert!(aiding_drop_cause(&UpdateOutcome::Skipped(
            SkipReason::MeasurementShapeMismatch
        ))
        .is_some());
        assert_eq!(
            predict_skip_cause(&PredictOutcome::Skipped(PredictSkipReason::NonPositiveDt)),
            None
        );
        assert!(predict_skip_cause(&PredictOutcome::Skipped(
            PredictSkipReason::InputShapeMismatch {
                expected: 6,
                supplied: 3
            }
        ))
        .is_some());
    }

    /// A TF provider that resolves nothing.
    struct NoTransforms;

    impl TfProvider for NoTransforms {
        fn get_transform(
            &self,
            _: FrameId,
            _: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            None
        }
    }
}
