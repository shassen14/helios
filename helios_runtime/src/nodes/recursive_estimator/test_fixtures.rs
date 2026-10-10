//! The recursive estimator's test fixtures: a recording filter, one
//! accelerometer source, and the bus, ticks and watching buffer to run a node
//! over them. Shared by the node's and the aiding sources' tests.

use super::aiding::Aiding;
use super::nis_health::NisWindow;
use super::node::RecursiveEstimatorNode;

use crate::channels::estimate::estimator_output;
use crate::nodes::estimation::{
    EstimatorInputBuilder, Measurement, MeasurementSource, PayloadReader,
};
use crate::observe::buffer::NodeObservations;
use crate::observe::observation::{Observation, ObservedValue};
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, InputPort, PortBus, PortDescriptor, SensorChannel};
use crate::stamped::{Health, Stamped};

use helios_core::estimation::carrier::kinematic_carrier_schema;
use helios_core::estimation::measurement::{MeasurementModel, Prediction};
use helios_core::estimation::schema::{InputSchema, MeasurementSchema, MeasurementSchemaBlock};
use helios_core::estimation::{
    EstimatorInputs, GaussianStateEstimator, Innovation, PredictOutcome, PredictSkipReason,
    SkipReason, UpdateOutcome,
};
use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::interchange::measurement::sensor::Acceleration;
use helios_core::prelude::{AgentId, MonotonicDuration};
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::state::Quantity;
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::transforms::{Convention, ErasedTransform};
use helios_core::spatial::{FrameAwareState, FrameId};

use nalgebra::{DMatrix, DVector};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex as StdMutex};

/// The tick's own step, which the node must never predict by.
const DT: f64 = 0.1;
/// The aiding entry the accelerometer source is read for.
const ACCEL_ENTRY: &str = "accel";

fn agent() -> AgentId {
    AgentId::new("car")
}

pub(super) fn accel_channel() -> ChannelKey {
    SensorChannel::named::<Vec<SensorReading<Acceleration>>>("accel").into()
}

pub(super) fn state_channel() -> ChannelKey {
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
    /// Skip every update for this reason while set.
    skip_update: Option<SkipReason>,
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
        match &calls.skip_update {
            Some(reason) => UpdateOutcome::Skipped(reason.clone()),
            None => UpdateOutcome::Applied(Innovation::new(calls.nis, 1)),
        }
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
pub(super) struct Probe {
    calls: Arc<StdMutex<Calls>>,
    pub(super) input_ready: Arc<AtomicBool>,
}

impl Probe {
    pub(super) fn predict_dts(&self) -> Vec<f64> {
        self.calls.lock().expect("test lock").predict_dts.clone()
    }

    pub(super) fn update_times(&self) -> Vec<f64> {
        self.calls.lock().expect("test lock").update_times.clone()
    }

    pub(super) fn fail_predict(&self, fail: bool) {
        self.calls.lock().expect("test lock").fail_predict = fail;
    }

    pub(super) fn report_nis(&self, nis: f64) {
        self.calls.lock().expect("test lock").nis = nis;
    }

    pub(super) fn skip_updates(&self, reason: SkipReason) {
        self.calls.lock().expect("test lock").skip_update = Some(reason);
    }
}

/// A node over a [`RecordingFilter`] with one accelerometer source, its
/// input ready, and the probe on it. The prior is valid at zero, as the
/// factory builds it.
pub(super) fn node() -> (RecursiveEstimatorNode, Probe) {
    node_with(None)
}

/// [`node`], its accelerometer source judged by `nis` if given.
pub(super) fn node_with(nis: Option<NisWindow>) -> (RecursiveEstimatorNode, Probe) {
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
            Some(window) => Aiding::new(ACCEL_ENTRY, Box::new(source)).with_nis_window(window),
            None => Aiding::new(ACCEL_ENTRY, Box::new(source)),
        }],
    );
    (node, Probe { calls, input_ready })
}

/// A bus carrying everything `node` reads and writes.
pub(super) fn bus_for(node: &RecursiveEstimatorNode) -> PortBus {
    let descriptor = node.port_descriptor();
    let mut channels: Vec<ChannelKey> = descriptor
        .inputs()
        .map(InputPort::channel)
        .cloned()
        .collect();
    channels.extend(descriptor.outputs().iter().cloned());
    PortBus::new(&[PortDescriptor::new(vec![], vec![], channels, None)])
}

pub(super) fn write_accel(bus: &PortBus, times: &[f64]) {
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

pub(super) fn tick(now: f64) -> TickContext<'static> {
    TickContext::detached(MonotonicTime(now), DT, 0)
}

/// A buffer holding every leaf `node` declares, all watched, as a pipeline
/// builds one.
pub(super) fn watching(node: &RecursiveEstimatorNode) -> NodeObservations {
    let leaves = node
        .port_descriptor()
        .observables()
        .iter()
        .map(|observable| observable.leaf_name().clone());
    let mut buffer = NodeObservations::new(node.name(), leaves);
    buffer.set_all_watched(true);
    buffer
}

/// A tick at `now` whose emits go to `buffer`.
pub(super) fn watched_tick(now: f64, buffer: &NodeObservations) -> TickContext<'_> {
    TickContext::new(MonotonicTime(now), DT, 0, buffer)
}

pub(super) fn drain(buffer: &NodeObservations) -> Vec<Observation> {
    let mut out = Vec::new();
    buffer.drain_into(&mut out);
    out
}

/// What the `primary` node reports on `leaf` for a reading at `t`.
pub(super) fn reported(leaf: &str, t: f64, value: f64) -> Observation {
    Observation {
        node: "primary".into(),
        leaf: leaf.into(),
        timestamp: MonotonicTime(t),
        value: ObservedValue::Scalar(value),
    }
}

/// The published estimate's health.
pub(super) fn published_health(bus: &PortBus) -> Health {
    bus.read::<FrameAwareState>(state_channel())
        .expect("the estimate is published")
        .health
        .clone()
}

/// A TF provider that resolves nothing.
pub(super) struct NoTransforms;

impl TfProvider for NoTransforms {
    fn get_transform(&self, _: FrameId, _: FrameId, _: MonotonicTime) -> Option<ErasedTransform> {
        None
    }
}
