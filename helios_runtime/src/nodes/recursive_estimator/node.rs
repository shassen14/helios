//! [`RecursiveEstimatorNode`]: the predict → update → publish loop around a
//! recursive filter, whichever filter, dynamics and measurement models it was
//! built from.
//!
//! ## Each tick
//!
//! 0. **Start (first tick only).** The prior was built before the node could
//!    read the pipeline clock, so it is stamped valid at this tick's time and
//!    nothing is predicted: no interval has elapsed since it began to hold.
//! 1. **Predict.** The input builder assembles the control vector from the
//!    bus, and the filter steps from the state's valid-at time to now,
//!    holding that input over the whole interval. If the builder can't
//!    assemble yet (cold start, dropout), predict is skipped and the tick
//!    goes on to the updates; the interval is not lost, since the next predict
//!    starts from the same valid-at time. A predict the filter itself skips
//!    for a fault (a misshapen input) is warned, rate-limited, and likewise
//!    caught up later.
//! 2. **Update.** Each aiding source hands over the readings it has not handed
//!    over before, oldest first, and each is applied to the filter. A dropped
//!    correction that stems from a fault is warned, rate-limited per source.
//! 3. **Publish.** The filter's state goes out on a channel named after the
//!    node, stamped with the state's valid-at time. After a tick with no
//!    predict that is earlier than the tick's: the estimate says when it
//!    holds. The node publishes no TF edge; if the stack's estimate seam names
//!    this node, its relay publishes the `base_link → odom` edge from this
//!    state.
//!
//! ## Health
//!
//! The state carries `Health::Degraded` while either holds:
//! - the last predict the filter attempted was skipped for a fault, so the
//!   estimate is no longer being propagated (cleared by the next applied
//!   predict);
//! - an aiding sensor with a NIS window has a full window whose mean
//!   NIS ÷ dof lies outside its band.
//!
//! Otherwise they are `Health::Ok`. Health says how far to trust the
//! estimate; it never changes what the filter computes.
//!
//! The state's valid-at time is the node's only clock memory. The tick's
//! `dt` is never read: a tick that does not predict would lose it.

use super::nis_health::NisWindow;

use crate::channels::estimate::estimator_output;
use crate::nodes::estimation::{EstimatorInputBuilder, Measurement, MeasurementSource};
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{AlgorithmNodePortDescriptor, ChannelError, ChannelKey, PortBus, PortDescriptor};
use crate::stamped::{Health, Stamped};

use helios_core::estimation::measurement::Unavailable;
use helios_core::estimation::{
    GaussianStateEstimator, PredictOutcome, PredictSkipReason, SkipReason, UpdateOutcome,
};
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::FrameAwareState;

use atomic_float::AtomicF64;
use std::sync::atomic::{AtomicBool, Ordering};
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
    output: ChannelKey,
    filter: Mutex<Running>,
    input: Box<dyn EstimatorInputBuilder>,
    aiding: Vec<Aiding>,
    descriptor: PortDescriptor,
    /// Whether the first tick has stamped the prior with the pipeline clock.
    started: AtomicBool,
    /// Tick time of the last "predict skipped" warning; `NEG_INFINITY` so the
    /// first fault always prints.
    last_predict_warned: AtomicF64,
}

impl RecursiveEstimatorNode {
    /// A node named `name` that runs `filter`, predicting from what `input`
    /// assembles and correcting from each of `aiding`, in order. It writes its
    /// state on the channel named after it.
    pub(crate) fn new(
        name: impl Into<String>,
        filter: Box<dyn GaussianStateEstimator>,
        input: Box<dyn EstimatorInputBuilder>,
        aiding: Vec<Aiding>,
    ) -> Self {
        let name = name.into();
        let output = estimator_output(&name);
        let mut builder = AlgorithmNodePortDescriptor::new()
            .inputs_from_slices(input.required_channels(), input.optional_channels())
            .output_internal(output.clone());
        for aiding in &aiding {
            // The source holds only an erased key, so it goes through the
            // slice path, which asserts the key is a sensor or internal
            // channel.
            builder = builder.inputs_from_slices(&[], &[aiding.source.channel().clone()]);
        }

        Self {
            name,
            output: output.into(),
            filter: Mutex::new(Running {
                filter,
                predict_fault: None,
            }),
            input,
            aiding,
            descriptor: builder.build(),
            started: AtomicBool::new(false),
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
        let Ok(mut running) = self.filter.lock() else {
            return;
        };
        let Running {
            filter,
            predict_fault,
        } = &mut *running;

        // 0. Start. Read and set under the filter lock, so no two ticks
        // can both see it unset.
        let starting = !self.started.swap(true, Ordering::Relaxed);
        if starting {
            filter.set_valid_at(tick.now);
        }

        // 1. Predict, from when the state last held to now. A skip the filter
        // reports as a fault is surfaced: predicting on the prior alone is
        // the same silent drift as an unaided update.
        let inputs = if starting {
            None
        } else {
            self.input.assemble(bus, &tick)
        };
        if let Some(inputs) = inputs {
            let dt = tick.now.0 - filter.state().timestamp.0;
            let outcome = filter.predict(dt, &inputs);
            if outcome == PredictOutcome::Applied {
                *predict_fault = None;
            }
            if let Some(cause) = predict_skip_cause(&outcome) {
                if passes_warn_throttle(&self.last_predict_warned, tick.now.0) {
                    warn!(
                        "estimator '{}' predict skipped: {cause} at t={:.3}; \
                         estimate not propagated.",
                        self.name, tick.now.0,
                    );
                }
                *predict_fault = Some(cause);
            }
        }

        // 2. Update from each aiding source.
        for aiding in &self.aiding {
            aiding.apply_new(bus, &mut **filter, Some(tf));
        }

        // 3. Publish, at the time the state holds, with the health it has
        // earned.
        let health = self
            .aiding
            .iter()
            .map(Aiding::health)
            .fold(predict_health(predict_fault.as_deref()), Health::worse_of);
        let state = filter.state().clone();
        let valid_at = state.timestamp;
        publish_estimate(bus, &self.output, state, valid_at, health, &tick);
    }
}

/// What the node holds under its lock: the filter, and why its last
/// attempted predict was skipped, if for a fault.
struct Running {
    filter: Box<dyn GaussianStateEstimator>,
    predict_fault: Option<String>,
}

/// The health a predict fault leaves the estimate in.
fn predict_health(fault: Option<&str>) -> Health {
    match fault {
        None => Health::Ok,
        Some(cause) => Health::Degraded {
            reason: format!("predict skipped: {cause}").into(),
        },
    }
}

/// One aiding source, the throttle on its drop warnings, and its NIS window
/// if it has one.
pub(crate) struct Aiding {
    source: Box<dyn MeasurementSource>,
    /// Reading time of the last "aiding dropped" warning; `NEG_INFINITY` so
    /// the first fault always prints.
    last_warned: AtomicF64,
    /// Locked only inside a tick, under the node's filter lock, so never
    /// contended.
    nis: Option<Mutex<NisWindow>>,
}

impl Aiding {
    /// `source`, with no NIS window.
    pub(crate) fn new(source: Box<dyn MeasurementSource>) -> Self {
        Self {
            source,
            last_warned: AtomicF64::new(f64::NEG_INFINITY),
            nis: None,
        }
    }

    /// The same source, judged by `window`.
    pub(crate) fn with_nis_window(mut self, window: NisWindow) -> Self {
        self.nis = Some(Mutex::new(window));
        self
    }

    /// `Degraded` while the NIS window is full and its mean is out of band;
    /// `Ok` otherwise, and always without a window.
    fn health(&self) -> Health {
        let Some(Ok(nis)) = self.nis.as_ref().map(Mutex::lock) else {
            return Health::Ok;
        };
        match nis.out_of_band_mean() {
            None => Health::Ok,
            Some(mean) => {
                let [low, high] = nis.band();
                Health::Degraded {
                    reason: format!(
                        "{}: mean NIS/dof {mean:.2} over the last {} readings is outside \
                         [{low}, {high}]",
                        self.source.channel(),
                        nis.capacity(),
                    )
                    .into(),
                }
            }
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
            if let (UpdateOutcome::Applied(innovation), Some(Ok(mut nis))) =
                (&outcome, self.nis.as_ref().map(Mutex::lock))
            {
                nis.record(*innovation);
            }
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

/// Writes `state` on `output`, stamped `at` and carrying `health`.
pub(crate) fn publish_estimate(
    bus: &PortBus,
    output: &ChannelKey,
    state: FrameAwareState,
    at: MonotonicTime,
    health: Health,
    tick: &TickContext,
) {
    let stamped = Stamped {
        value: state,
        timestamp: at,
        health,
        producer: tick.node_id,
    };
    if let Err(ChannelError::UnknownChannel) = bus.write(output.clone(), stamped) {
        warn!(
            channel = %output,
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
        UpdateOutcome::Skipped(SkipReason::NonFiniteInput) => {
            Some("measurement, R or estimate holds NaN or ∞".to_string())
        }
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
        PredictOutcome::Skipped(PredictSkipReason::NonFiniteInput) => {
            Some("step, input or estimate holds NaN or ∞".to_string())
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
    use helios_core::prelude::{AgentId, MonotonicDuration};
    use helios_core::spatial::state::Quantity;
    use helios_core::spatial::transforms::{Convention, ErasedTransform};
    use helios_core::spatial::FrameId;

    use nalgebra::{DMatrix, DVector};
    use std::sync::{Arc, Mutex as StdMutex};

    /// The tick's own step, which the node must never predict by.
    const DT: f64 = 0.1;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn accel_channel() -> ChannelKey {
        SensorChannel::named::<Vec<SensorReading<Acceleration>>>("accel").into()
    }

    fn state_channel() -> ChannelKey {
        estimator_output("primary").into()
    }

    /// What the filter was asked to do, readable after the filter is boxed,
    /// and how it answers.
    #[derive(Default)]
    struct Calls {
        predict_dts: Vec<f64>,
        update_times: Vec<f64>,
        /// Skip every predict for a fault while set.
        fail_predict: bool,
        /// The NIS every update reports, over one degree of freedom.
        nis: f64,
    }

    /// A filter that applies everything, advancing its valid-at time as a
    /// real filter does, and records each call.
    struct RecordingFilter {
        state: FrameAwareState,
        calls: Arc<StdMutex<Calls>>,
    }

    impl GaussianStateEstimator for RecordingFilter {
        fn predict(&mut self, dt: f64, _: &EstimatorInputs) -> PredictOutcome {
            let mut calls = self.calls.lock().expect("test lock");
            calls.predict_dts.push(dt);
            if calls.fail_predict {
                return PredictOutcome::Skipped(PredictSkipReason::NonFiniteInput);
            }
            self.state.timestamp += MonotonicDuration(dt);
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
            let mut calls = self.calls.lock().expect("test lock");
            calls.update_times.push(at.0);
            UpdateOutcome::Applied(Innovation::new(calls.nis, 1))
        }

        fn set_valid_at(&mut self, t: MonotonicTime) {
            self.state.timestamp = t;
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

    /// An input builder with nothing to read, ready while its flag is set
    /// (clear it to stand for an input dropout).
    struct ToggledInput(Arc<AtomicBool>);

    impl EstimatorInputBuilder for ToggledInput {
        fn input_schema(&self) -> Arc<InputSchema> {
            Arc::new(InputSchema::compose(vec![]))
        }

        fn assemble(&self, _: &PortBus, _: &TickContext) -> Option<EstimatorInputs> {
            self.0.load(Ordering::Relaxed).then(|| EstimatorInputs {
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

    /// What a test reads and steers around a [`node`].
    struct Probe {
        calls: Arc<StdMutex<Calls>>,
        input_ready: Arc<AtomicBool>,
    }

    impl Probe {
        fn predict_dts(&self) -> Vec<f64> {
            self.calls.lock().expect("test lock").predict_dts.clone()
        }

        fn fail_predict(&self, fail: bool) {
            self.calls.lock().expect("test lock").fail_predict = fail;
        }

        fn report_nis(&self, nis: f64) {
            self.calls.lock().expect("test lock").nis = nis;
        }
    }

    /// A node over a [`RecordingFilter`] with one accelerometer source, its
    /// input ready, and the probe on it. The prior is valid at zero, as the
    /// factory builds it.
    fn node() -> (RecursiveEstimatorNode, Probe) {
        node_with(None)
    }

    /// [`node`], its accelerometer source judged by `nis` if given.
    fn node_with(nis: Option<NisWindow>) -> (RecursiveEstimatorNode, Probe) {
        let calls = Arc::new(StdMutex::new(Calls::default()));
        let filter = RecordingFilter {
            // Anchors base_link in odom, so the state holds a pose.
            state: FrameAwareState::from_schema(
                Arc::new(kinematic_carrier_schema(agent())),
                MonotonicTime(0.0),
            ),
            calls: Arc::clone(&calls),
        };
        let input_ready = Arc::new(AtomicBool::new(true));
        let source = AccelSource {
            reader: PayloadReader::new(SensorChannel::named::<Vec<SensorReading<Acceleration>>>(
                "accel",
            )),
            model: ZeroModel,
            noise: DMatrix::identity(3, 3),
        };
        let node = RecursiveEstimatorNode::new(
            "primary",
            Box::new(filter),
            Box::new(ToggledInput(Arc::clone(&input_ready))),
            vec![match nis {
                Some(window) => Aiding::new(Box::new(source)).with_nis_window(window),
                None => Aiding::new(Box::new(source)),
            }],
        );
        (node, Probe { calls, input_ready })
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
        let (node, probe) = node();
        let bus = bus_for(&node);
        write_accel(&bus, &[2.0, 1.0]);

        node.execute(&bus, &NoTransforms, tick(2.0));
        node.execute(&bus, &NoTransforms, tick(2.1));

        assert_eq!(
            probe.calls.lock().expect("test lock").update_times,
            [1.0, 2.0]
        );
    }

    /// The published estimate's envelope stamp and valid-at time.
    fn published_stamps(bus: &PortBus) -> (f64, f64) {
        let state = bus
            .read::<FrameAwareState>(state_channel())
            .expect("the estimate is published");
        (state.timestamp.0, state.value.timestamp.0)
    }

    /// The first tick stamps the prior with the clock's time and predicts
    /// nothing, wherever the clock starts.
    #[test]
    fn the_first_tick_starts_the_prior_at_its_time() {
        let (node, probe) = node();
        let bus = bus_for(&node);

        node.execute(&bus, &NoTransforms, tick(1000.0));

        assert!(probe.predict_dts().is_empty());
        assert_eq!(published_stamps(&bus), (1000.0, 1000.0));
    }

    /// Predict steps from the state's valid-at time to now, never by the
    /// tick's `dt`.
    #[test]
    fn predict_steps_from_the_valid_at_time_to_now() {
        let (node, probe) = node();
        let bus = bus_for(&node);

        node.execute(&bus, &NoTransforms, tick(1.0));
        node.execute(&bus, &NoTransforms, tick(1.25));

        assert_eq!(probe.predict_dts(), [0.25]);
        assert_eq!(published_stamps(&bus), (1.25, 1.25));
    }

    /// A tick whose input can't be assembled loses no time: the next predict
    /// covers it, and meanwhile the estimate is published at the time it
    /// still holds, not the tick's.
    #[test]
    fn a_tick_without_input_loses_no_time() {
        let (node, probe) = node();
        let bus = bus_for(&node);

        node.execute(&bus, &NoTransforms, tick(1.0));
        probe.input_ready.store(false, Ordering::Relaxed);
        node.execute(&bus, &NoTransforms, tick(1.5));
        assert_eq!(published_stamps(&bus), (1.0, 1.0));

        probe.input_ready.store(true, Ordering::Relaxed);
        node.execute(&bus, &NoTransforms, tick(2.0));

        assert_eq!(probe.predict_dts(), [1.0]);
        assert_eq!(published_stamps(&bus), (2.0, 2.0));
    }

    /// The published estimate's health.
    fn published_health(bus: &PortBus) -> Health {
        bus.read::<FrameAwareState>(state_channel())
            .expect("the estimate is published")
            .health
            .clone()
    }

    /// A predict skipped for a fault degrades the estimate, which stays
    /// degraded through ticks with no predict and recovers at the next
    /// applied one.
    #[test]
    fn a_predict_fault_degrades_until_a_predict_applies() {
        let (node, probe) = node();
        let bus = bus_for(&node);
        node.execute(&bus, &NoTransforms, tick(1.0));
        assert!(matches!(published_health(&bus), Health::Ok));

        probe.fail_predict(true);
        node.execute(&bus, &NoTransforms, tick(1.1));
        let Health::Degraded { reason } = published_health(&bus) else {
            panic!("a predict fault degrades the estimate");
        };
        assert!(reason.starts_with("predict skipped:"), "{reason}");

        probe.fail_predict(false);
        probe.input_ready.store(false, Ordering::Relaxed);
        node.execute(&bus, &NoTransforms, tick(1.2));
        assert!(matches!(published_health(&bus), Health::Degraded { .. }));

        probe.input_ready.store(true, Ordering::Relaxed);
        node.execute(&bus, &NoTransforms, tick(1.3));
        assert!(matches!(published_health(&bus), Health::Ok));
    }

    /// An aiding sensor whose windowed mean NIS leaves the band degrades the
    /// estimate, naming the sensor; back inside, the estimate is healthy.
    #[test]
    fn an_out_of_band_nis_window_degrades_the_estimate() {
        let window = NisWindow::new(2, [0.5, 2.0]).expect("a valid window");
        let (node, probe) = node_with(Some(window));
        let bus = bus_for(&node);
        probe.report_nis(9.0);

        write_accel(&bus, &[1.0]);
        node.execute(&bus, &NoTransforms, tick(1.0));
        assert!(
            matches!(published_health(&bus), Health::Ok),
            "one reading does not fill the window"
        );

        write_accel(&bus, &[2.0]);
        node.execute(&bus, &NoTransforms, tick(2.0));
        let Health::Degraded { reason } = published_health(&bus) else {
            panic!("a full out-of-band window degrades the estimate");
        };
        assert!(reason.contains("accel"), "{reason}");

        probe.report_nis(1.0);
        write_accel(&bus, &[3.0, 4.0]);
        node.execute(&bus, &NoTransforms, tick(4.0));
        assert!(matches!(published_health(&bus), Health::Ok));
    }

    /// Without a NIS window, no innovation degrades the estimate.
    #[test]
    fn without_a_nis_window_innovations_leave_health_alone() {
        let (node, probe) = node();
        let bus = bus_for(&node);
        probe.report_nis(1e6);
        write_accel(&bus, &[1.0, 2.0, 3.0]);
        node.execute(&bus, &NoTransforms, tick(3.0));
        assert!(matches!(published_health(&bus), Health::Ok));
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
