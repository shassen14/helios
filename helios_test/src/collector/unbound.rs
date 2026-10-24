//! Values a bound selector covers that arrived under no bound key: the
//! collector counts them, so a run that dropped them fails rather than
//! judging series that silently missed values.

use super::{instance_of, ObservationCollector};
use crate::assertion::AssertionTarget;

use helios_runtime::observe::key::{RecordedKey, Subject};

use std::fmt::{self, Display};

impl ObservationCollector {
    /// Each key a value arrived under that a bound selector covers but no
    /// series is bound to, with how many arrived, in path order. Empty when
    /// every covered value landed in a series.
    pub fn unbound_arrivals(&self) -> Vec<UnboundArrivals> {
        let mut unbound: Vec<UnboundArrivals> = self
            .unbound
            .iter()
            .map(|(key, count)| UnboundArrivals {
                key: key.clone(),
                count: *count,
            })
            .collect();
        unbound.sort_by_cached_key(|arrivals| (arrivals.key.path(), arrivals.key.subject.clone()));
        unbound
    }

    /// Covers each key `selector` names from now on. A key found uncovered
    /// before is looked at again, since it may be covered now.
    pub(super) fn cover(&mut self, selector: AssertionTarget) {
        self.covered.insert(selector);
        self.unrequested.clear();
    }

    /// Counts a value that arrived under `key`, bound to no series, when a
    /// bound selector covers it.
    pub(super) fn count_unbound(&mut self, key: &RecordedKey) {
        if let Some(count) = self.unbound.get_mut(key) {
            *count += 1;
            return;
        }
        if self.unrequested.contains(key) {
            return;
        }
        let path = key.path();
        let instance = instance_of(&key.subject);
        let covered = self
            .covered
            .iter()
            .any(|target| target.path() == path && target.covers(instance));
        if covered {
            self.unbound.insert(key.clone(), 1);
        } else {
            self.unrequested.insert(key.clone());
        }
    }
}

/// _Post-step._ Values that arrived under one key a bound selector covers
/// but to which no series is bound.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnboundArrivals {
    pub key: RecordedKey,
    pub count: u64,
}

/// The path, then the whole key where the path alone doesn't name it: a
/// channel's kind and payload type, and its instance.
impl Display for UnboundArrivals {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.key.path())?;
        if let Subject::Channel { channel, instance } = &self.key.subject {
            write!(f, " ({channel}")?;
            if let Some(instance) = instance {
                write!(f, ", instance \"{instance}\"")?;
            }
            write!(f, ")")?;
        }
        write!(f, ": {} value(s) arrived under no bound series", self.count)
    }
}

#[cfg(test)]
mod tests {
    use super::{super::fixtures::*, *};
    use crate::assertion::InstanceSelector;

    use helios_runtime::observe::{observation::ObservedValue, sink::ObservationSink};

    #[test]
    fn a_value_under_a_bound_selector_but_no_bound_key_is_counted() {
        // `wheels` is bound for the front wheel alone: the rear's values are
        // counted, each arrival once, the front's are not.
        let front = key(CAR, wheel("front"));
        let rear = key(CAR, wheel("rear"));
        let mut collector = collector();
        collector.bind(&channel_target(CAR, &wheels()), front.clone());

        collector.receive(&[
            arrival(&front, 1.0, ObservedValue::Scalar(1.0)),
            arrival(&rear, 1.0, ObservedValue::Scalar(2.0)),
        ]);
        collector.receive(&[arrival(&rear, 2.0, ObservedValue::Scalar(2.0))]);

        assert_eq!(
            collector.unbound_arrivals(),
            vec![UnboundArrivals {
                key: rear,
                count: 2
            }]
        );
    }

    #[test]
    fn a_selector_naming_one_instance_does_not_count_its_siblings() {
        // Front is selected and bound; rear is a sibling no selector covers.
        // Another selector with no instance would cover it.
        let front = key(CAR, wheel("front"));
        let rear = key(CAR, wheel("rear"));
        let front_only =
            channel_target(CAR, &wheels()).with_instance(InstanceSelector::One("front".into()));
        let mut selected = collector();
        selected.bind(&front_only, front.clone());
        selected.receive(&[arrival(&rear, 1.0, ObservedValue::Scalar(2.0))]);
        assert_eq!(selected.unbound_arrivals(), vec![]);

        let mut with_whole = collector();
        with_whole.bind(&front_only, front.clone());
        with_whole.bind(&channel_target(CAR, &wheels()), front);
        with_whole.receive(&[arrival(&rear, 1.0, ObservedValue::Scalar(2.0))]);
        assert_eq!(
            with_whole.unbound_arrivals(),
            vec![UnboundArrivals {
                key: rear,
                count: 1
            }]
        );
    }

    #[test]
    fn a_value_under_a_path_no_selector_covers_is_not_counted() {
        // Another sink's request: every sink is handed the whole batch.
        let mut collector = collector();
        collector.bind(&target(CAR, NIS), emission_key(CAR, NIS));

        collector.receive(&[
            arrival(&emission_key(CAR, SPEED), 1.0, ObservedValue::Scalar(1.0)),
            arrival(&emission_key(CAR, SPEED), 2.0, ObservedValue::Scalar(1.0)),
        ]);

        assert_eq!(collector.unbound_arrivals(), vec![]);
    }

    #[test]
    fn a_key_found_uncovered_is_counted_once_a_selector_covers_it() {
        // The rear arrives before anything covers it, then `wheels` is bound
        // for the front: the rear's next value is one the run missed.
        let front = key(CAR, wheel("front"));
        let rear = key(CAR, wheel("rear"));
        let mut collector = collector();

        collector.receive(&[arrival(&rear, 1.0, ObservedValue::Scalar(2.0))]);
        collector.bind(&channel_target(CAR, &wheels()), front);
        collector.receive(&[arrival(&rear, 2.0, ObservedValue::Scalar(2.0))]);

        assert_eq!(
            collector.unbound_arrivals(),
            vec![UnboundArrivals {
                key: rear,
                count: 1
            }]
        );
    }

    #[test]
    fn unbound_arrivals_name_the_whole_key_where_the_path_does_not() {
        let rear = UnboundArrivals {
            key: key(CAR, wheel("rear")),
            count: 2,
        };
        let nis = UnboundArrivals {
            key: emission_key(CAR, NIS),
            count: 1,
        };

        assert_eq!(
            rear.to_string(),
            format!(
                "agent.car.channel.wheels ({}, instance \"rear\"): 2 value(s) arrived \
                 under no bound series",
                wheels()
            )
        );
        assert_eq!(
            nis.to_string(),
            "agent.car.node.estimator.aiding.gps.nis: 1 value(s) arrived under no bound series"
        );
    }
}
