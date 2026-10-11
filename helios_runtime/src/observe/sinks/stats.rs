//! A sink that watches everything and logs, per agent, how many observations
//! arrive per second.
//!
//! Its request is the widest there is, so a run with it installed is a run
//! with every observable watched: the run a host compares against one with
//! nothing watched to show that watching changes nothing.

use crate::observe::{
    observation::AgentObservation,
    sink::{ObservationSink, WatchRequest},
};

use helios_core::prelude::{AgentId, MonotonicDuration, MonotonicTime};

use tracing::info;

/// Counts each agent's observations in fixed buckets of time and logs their
/// rates as each bucket finishes.
///
/// Bucket `k` holds the timestamps in `[k * period, (k + 1) * period)`, so
/// every bucket is exactly one period long and reports line up across runs.
/// Time is read from the observations' own timestamps, never the wall clock,
/// so the sink needs nothing from its host and reports the same rates on
/// every replay of a run.
#[derive(Debug)]
pub struct StatsSink {
    period: MonotonicDuration,
    /// The bucket being counted; `None` until the first observation.
    bucket: Option<i64>,
    /// Observations per agent in the current bucket, in the order the agents
    /// first appeared. An agent stays listed once seen, so one that goes
    /// silent is reported at zero rather than dropped.
    counts: Vec<(AgentId, usize)>,
}

impl StatsSink {
    pub fn new(period: MonotonicDuration) -> Self {
        Self {
            period,
            bucket: None,
            counts: Vec::new(),
        }
    }

    /// Counts one observation. If it opens a later bucket, first returns each
    /// agent's rate over the bucket it finishes.
    ///
    /// A value is stamped with the time it describes, so a measurement update
    /// can arrive behind a later observation. One from a bucket already
    /// finished counts toward the current bucket. Buckets nothing landed in
    /// are skipped, not reported.
    fn add(&mut self, observation: &AgentObservation) -> Option<Vec<(AgentId, f64)>> {
        let bucket = self.bucket_of(observation.observation.timestamp);

        let report = match self.bucket {
            Some(current) if bucket > current => {
                self.bucket = Some(bucket);
                Some(self.take_rates())
            }
            Some(_) => None,
            None => {
                self.bucket = Some(bucket);
                None
            }
        };

        // A linear search: `AgentId` has no order to key a map by, and a host
        // runs few agents.
        match self
            .counts
            .iter_mut()
            .find(|(agent, _)| *agent == observation.agent)
        {
            Some((_, count)) => *count += 1,
            None => self.counts.push((observation.agent.clone(), 1)),
        }

        report
    }

    fn bucket_of(&self, timestamp: MonotonicTime) -> i64 {
        (timestamp.0 / self.period.0).floor() as i64
    }

    /// Each agent's rate over the finished bucket, zeroing its count.
    fn take_rates(&mut self) -> Vec<(AgentId, f64)> {
        self.counts
            .iter_mut()
            .map(|(agent, count)| {
                let rate = *count as f64 / self.period.0;
                *count = 0;
                (agent.clone(), rate)
            })
            .collect()
    }
}

impl ObservationSink for StatsSink {
    fn request(&self) -> WatchRequest {
        WatchRequest::Everything
    }

    fn receive(&mut self, batch: &[AgentObservation]) {
        for observation in batch {
            // Printing is this sink's whole output, so it logs; a node never
            // does, it emits.
            if let Some(report) = self.add(observation) {
                for (agent, rate) in report {
                    info!("Agent '{}' reported {:.1} observations/s", agent, rate);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::observe::observation::{Observation, ObservedValue};

    const CAR: &str = "car";
    const TRUCK: &str = "truck";
    const PERIOD: MonotonicDuration = MonotonicDuration(1.0);

    fn observed(agent: &str, at: f64) -> AgentObservation {
        AgentObservation {
            agent: AgentId::new(agent),
            observation: Observation {
                node: "estimator".into(),
                leaf: "aiding.gps.nis".into(),
                timestamp: MonotonicTime(at),
                value: ObservedValue::Scalar(0.0),
            },
        }
    }

    /// Adds each observation in turn and keeps every report that comes out.
    fn add_all(
        sink: &mut StatsSink,
        observations: &[AgentObservation],
    ) -> Vec<Vec<(AgentId, f64)>> {
        observations
            .iter()
            .filter_map(|observation| sink.add(observation))
            .collect()
    }

    fn rate_of(report: &[(AgentId, f64)], agent: &str) -> f64 {
        report
            .iter()
            .find(|(id, _)| id.as_str() == agent)
            .map(|(_, rate)| *rate)
            .expect("agent is in the report")
    }

    #[test]
    fn it_requests_everything() {
        assert_eq!(StatsSink::new(PERIOD).request(), WatchRequest::Everything);
    }

    #[test]
    fn nothing_is_reported_within_one_bucket() {
        let mut sink = StatsSink::new(PERIOD);

        let reports = add_all(
            &mut sink,
            &[observed(CAR, 0.0), observed(CAR, 0.5), observed(CAR, 0.99)],
        );

        assert!(reports.is_empty());
    }

    #[test]
    fn opening_a_bucket_reports_the_finished_one_per_agent() {
        let mut sink = StatsSink::new(PERIOD);

        let reports = add_all(
            &mut sink,
            &[
                observed(CAR, 0.0),
                observed(TRUCK, 0.2),
                observed(CAR, 0.5),
                observed(CAR, 1.0),
            ],
        );

        // The observation at 1.0 opens the next bucket, so it isn't counted
        // in the one it finishes.
        assert_eq!(reports.len(), 1);
        assert_eq!(rate_of(&reports[0], CAR), 2.0);
        assert_eq!(rate_of(&reports[0], TRUCK), 1.0);
    }

    #[test]
    fn the_opening_observation_counts_in_its_own_bucket_and_silent_agents_report_zero() {
        let mut sink = StatsSink::new(PERIOD);

        let reports = add_all(
            &mut sink,
            &[
                observed(CAR, 0.0),
                observed(TRUCK, 0.5),
                observed(CAR, 1.0),
                observed(CAR, 2.0),
            ],
        );

        assert_eq!(reports.len(), 2);
        assert_eq!(rate_of(&reports[1], CAR), 1.0);
        assert_eq!(rate_of(&reports[1], TRUCK), 0.0);
    }

    #[test]
    fn a_late_observation_counts_toward_the_current_bucket() {
        let mut sink = StatsSink::new(PERIOD);

        let reports = add_all(
            &mut sink,
            &[
                observed(CAR, 0.0),
                observed(CAR, 1.1),
                observed(CAR, 0.9),
                observed(CAR, 2.0),
            ],
        );

        assert_eq!(rate_of(&reports[1], CAR), 2.0);
    }

    #[test]
    fn a_rate_is_per_second_whatever_the_period() {
        let mut sink = StatsSink::new(MonotonicDuration(0.5));

        let reports = add_all(
            &mut sink,
            &[observed(CAR, 0.0), observed(CAR, 0.25), observed(CAR, 0.5)],
        );

        assert_eq!(rate_of(&reports[0], CAR), 4.0);
    }
}
