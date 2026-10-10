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
//!    Each applied correction reports its NIS ÷ dof, and each dropped one a
//!    count, under the source's `aiding.<entry>` leaves, for watchers.
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

use super::aiding::Aiding;
use super::throttle::passes_warn_throttle;

use crate::channels::estimate::estimator_output;
use crate::nodes::estimation::EstimatorInputBuilder;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{AlgorithmNodePortDescriptor, ChannelError, ChannelKey, PortBus, PortDescriptor};
use crate::stamped::{Health, Stamped};

use helios_core::estimation::{GaussianStateEstimator, PredictOutcome, PredictSkipReason};
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::FrameAwareState;

use atomic_float::AtomicF64;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use tracing::warn;

/// A recursive filter run as a pipeline node.
///
/// The port descriptor is derived from the input builder and the aiding
/// sources: the builder's channels as it declares them, each aiding channel
/// optional, since the filter still predicts and publishes without it, and
/// each aiding source's two leaves, its NIS and its drop count.
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
            builder = aiding.declare(builder);
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
            aiding.apply_new(bus, &tick, &mut **filter, Some(tf));
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

/// The cause of a *loud* predict skip, or `None` for an applied predict or the
/// expected quiet skip (a non-positive step, as on the first tick).
///
/// The predict-side twin of the aiding sources' drop cause.
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

#[cfg(test)]
mod tests {
    //! The loop's wiring only: what it reads, the order it applies readings
    //! in, and what it publishes. Filter math is tested in `helios_core`.

    use super::*;
    use crate::nodes::recursive_estimator::test_fixtures::*;

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

    /// Applied predicts and expected skips are quiet; faults are loud.
    #[test]
    fn only_predict_faults_are_loud() {
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
}
