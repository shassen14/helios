//! [`Aiding`]: one aiding source of a recursive estimator. It applies the
//! source's new readings to the filter, judges them by its NIS window, reports
//! them to watchers, and warns on a correction dropped for a fault.

use super::leaves::{AidingLeaf, AidingLeaves, ONE_DROP};
use super::nis_health::{nis_per_dof, NisWindow};
use super::throttle::passes_warn_throttle;

use crate::nodes::estimation::{Measurement, MeasurementSource};
use crate::pipeline::node::TickContext;
use crate::port::{AlgorithmNodePortDescriptor, PortBus};
use crate::stamped::Health;

use helios_core::estimation::measurement::Unavailable;
use helios_core::estimation::{GaussianStateEstimator, SkipReason, UpdateOutcome};
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::tf::TfProvider;

use atomic_float::AtomicF64;
use std::sync::Mutex;
use tracing::warn;

/// One aiding source, the throttle on its drop warnings, its NIS window if it
/// has one, and the leaves it reports under.
pub(crate) struct Aiding {
    source: Box<dyn MeasurementSource>,
    /// Reading time of the last "aiding dropped" warning; `NEG_INFINITY` so
    /// the first fault always prints.
    last_warned: AtomicF64,
    /// Locked only inside a tick, under the node's filter lock, so never
    /// contended.
    nis: Option<Mutex<NisWindow>>,
    /// The paths this source reports under.
    leaves: AidingLeaves,
}

impl Aiding {
    /// `source`, read for the aiding entry named `entry`, with no NIS window.
    /// The entry's name names its leaves.
    pub(crate) fn new(entry: &str, source: Box<dyn MeasurementSource>) -> Self {
        Self {
            source,
            last_warned: AtomicF64::new(f64::NEG_INFINITY),
            nis: None,
            leaves: AidingLeaves::new(entry),
        }
    }

    /// The same source, judged by `window`.
    pub(crate) fn with_nis_window(mut self, window: NisWindow) -> Self {
        self.nis = Some(Mutex::new(window));
        self
    }

    /// `builder` with this source's channel as an optional input, since the
    /// filter still predicts and publishes without it, and its leaves.
    pub(super) fn declare(
        &self,
        mut builder: AlgorithmNodePortDescriptor,
    ) -> AlgorithmNodePortDescriptor {
        // The source holds only an erased key, so it goes through the slice
        // path, which asserts the key is a sensor or internal channel.
        builder = builder.inputs_from_slices(&[], &[self.source.channel().clone()]);
        for (leaf, path) in self.leaves.iter() {
            builder = builder.observable(path.clone(), leaf.determinism());
        }
        builder
    }

    /// `Degraded` while the NIS window is full and its mean is out of band;
    /// `Ok` otherwise, and always without a window.
    pub(super) fn health(&self) -> Health {
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
    ///
    /// Reports each applied correction's NIS ÷ dof and each correction
    /// dropped for a fault on the source's leaves, stamped with the reading's
    /// time. Every drop is reported, though its warning is rate-limited.
    pub(super) fn apply_new(
        &self,
        bus: &PortBus,
        tick: &TickContext,
        filter: &mut dyn GaussianStateEstimator,
        tf: Option<&dyn TfProvider>,
    ) {
        for Measurement { z, at } in self.source.take_new(bus) {
            let outcome = filter.update(&z, self.source.model(), self.source.noise(), tf, at);

            if let UpdateOutcome::Applied(innovation) = &outcome {
                if let Some(ratio) = nis_per_dof(*innovation) {
                    self.report(tick, AidingLeaf::Nis, at, ratio);
                }
                if let Some(Ok(mut nis)) = self.nis.as_ref().map(Mutex::lock) {
                    nis.record(*innovation);
                }
            }

            // Expected quiet skips (cold start, no provider) and applied
            // updates are not drops: no count, no warning.
            if let Some(cause) = aiding_drop_cause(&outcome) {
                self.report(tick, AidingLeaf::Dropped, at, ONE_DROP);
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

    /// Reports `value` on `leaf`, stamped `at`, if a watcher asked for it.
    fn report(&self, tick: &TickContext, leaf: AidingLeaf, at: MonotonicTime, value: f64) {
        let path = self.leaves.path(leaf);
        debug_assert!(
            path.is_some(),
            "aiding leaf {leaf:?} is missing from AidingLeaf::ALL"
        );
        if let Some(path) = path {
            tick.emit(path, at, value);
        }
    }
}

/// The cause of a *loud* aiding drop, or `None` for an applied update or an
/// expected quiet skip (cold start / no provider).
///
/// The core filter has already classified the skip; the caller only decides
/// whether to warn. The frame-carrying transform faults name their frames so
/// the log points straight at the missing extrinsic.
fn aiding_drop_cause(outcome: &UpdateOutcome) -> Option<String> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nodes::recursive_estimator::test_fixtures::*;

    use crate::pipeline::node::PipelineNode;
    use crate::port::Determinism;

    /// Readings are applied oldest first, and a batch still on the channel the
    /// next tick is not applied again.
    #[test]
    fn readings_are_applied_oldest_first_and_once() {
        let (node, probe) = node();
        let bus = bus_for(&node);
        write_accel(&bus, &[2.0, 1.0]);

        node.execute(&bus, &NoTransforms, tick(2.0));
        node.execute(&bus, &NoTransforms, tick(2.1));

        assert_eq!(probe.update_times(), [1.0, 2.0]);
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

    /// Each aiding entry declares every leaf it reports, as replayable values.
    #[test]
    fn each_aiding_entry_declares_its_leaves() {
        let (node, _) = node();

        let declared: Vec<(&str, Determinism)> = node
            .port_descriptor()
            .observables()
            .iter()
            .map(|observable| (observable.leaf_name().as_ref(), observable.determinism()))
            .collect();

        assert_eq!(
            declared,
            [
                ("aiding.accel.nis", Determinism::Reproducible),
                ("aiding.accel.dropped", Determinism::Reproducible),
            ]
        );
    }

    /// Each applied update reports its NIS ÷ dof, stamped with the reading's
    /// time, not the tick's. The fixture's innovations have one degree of
    /// freedom, so the value is the NIS itself.
    #[test]
    fn each_applied_update_reports_its_nis_at_the_reading_time() {
        let (node, probe) = node();
        let bus = bus_for(&node);
        let buffer = watching(&node);
        probe.report_nis(2.5);
        write_accel(&bus, &[1.0, 2.0]);

        node.execute(&bus, &NoTransforms, watched_tick(3.0, &buffer));

        assert_eq!(
            drain(&buffer),
            [
                reported("aiding.accel.nis", 1.0, 2.5),
                reported("aiding.accel.nis", 2.0, 2.5),
            ]
        );
    }

    /// Every correction dropped for a fault is counted, though all but the
    /// first warning inside the throttle interval are suppressed.
    #[test]
    fn every_loud_drop_is_counted_while_its_warning_is_throttled() {
        let (node, probe) = node();
        let bus = bus_for(&node);
        let buffer = watching(&node);
        probe.skip_updates(SkipReason::MeasurementShapeMismatch);
        write_accel(&bus, &[1.0, 2.0, 3.0]);

        node.execute(&bus, &NoTransforms, watched_tick(3.0, &buffer));

        assert_eq!(
            drain(&buffer),
            [
                reported("aiding.accel.dropped", 1.0, ONE_DROP),
                reported("aiding.accel.dropped", 2.0, ONE_DROP),
                reported("aiding.accel.dropped", 3.0, ONE_DROP),
            ]
        );
    }

    /// An expected skip, such as a model not ready yet, is neither a drop nor
    /// an applied update, so it reports nothing.
    #[test]
    fn a_quiet_skip_reports_nothing() {
        let (node, probe) = node();
        let bus = bus_for(&node);
        let buffer = watching(&node);
        probe.skip_updates(SkipReason::Model(Unavailable::ColdStart));
        write_accel(&bus, &[1.0]);

        node.execute(&bus, &NoTransforms, watched_tick(1.0, &buffer));

        assert!(drain(&buffer).is_empty());
    }

    /// Applied updates and expected skips are quiet; faults are loud.
    #[test]
    fn only_aiding_faults_are_loud() {
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
    }
}
