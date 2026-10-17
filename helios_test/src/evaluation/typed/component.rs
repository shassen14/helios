//! [`Component`]: one metric an evaluator offers, read off its typed error
//! at each sample, or over the whole run.

use crate::evaluation::{
    derived::Refused,
    kind::{InputRejection, Metric},
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::MonotonicTime;

/// _Pre-step._ One metric of an evaluator whose error is `E`: its name and
/// unit, and how its values are read. A run file names it as
/// `eval.inline.<name>.<metric>`.
pub struct Component<E> {
    name: &'static str,
    si_unit: SiUnit,
    reading: Reading<E>,
}

impl<E> Component<E> {
    /// A metric with one value per sample, read off its error, or refused
    /// there in the kind's own words: a heading of a frame that faces no way.
    pub fn each(
        name: &'static str,
        si_unit: SiUnit,
        value: impl Fn(&E) -> Result<f64, InputRejection> + Send + Sync + 'static,
    ) -> Self {
        Self {
            name,
            si_unit,
            reading: Reading::Each(Box::new(value)),
        }
    }

    /// A metric with one value for the whole run, from every sample's error
    /// in stamp order, stamped at the run's last tick: a path's length, a
    /// ratio of sums. `None` when the run gives it no value.
    pub fn whole_run(
        name: &'static str,
        si_unit: SiUnit,
        value: impl Fn(&[(MonotonicTime, E)]) -> Option<f64> + Send + Sync + 'static,
    ) -> Self {
        Self {
            name,
            si_unit,
            reading: Reading::WholeRun(Box::new(value)),
        }
    }

    /// Its name and unit.
    pub fn metric(&self) -> Metric {
        Metric {
            name: self.name,
            si_unit: self.si_unit,
        }
    }

    /// Its values over `errors`, every computed sample in stamp order, a
    /// whole-run value stamped `until`. A sample it can't give is counted
    /// in `refused`.
    pub(crate) fn values(
        &self,
        errors: &[(MonotonicTime, E)],
        until: MonotonicTime,
        refused: &mut Refused,
    ) -> Vec<(MonotonicTime, f64)> {
        match &self.reading {
            Reading::Each(value) => errors
                .iter()
                .filter_map(|(at, error)| match value(error) {
                    Ok(value) => Some((*at, value)),
                    Err(rejection) => {
                        refused.add(rejection);
                        None
                    }
                })
                .collect(),
            Reading::WholeRun(value) => value(errors)
                .map(|value| (until, value))
                .into_iter()
                .collect(),
        }
    }
}

/// How a [`Component`]'s values are read.
enum Reading<E> {
    Each(PerSample<E>),
    WholeRun(OverRun<E>),
}

/// A value read off one sample's error, or refused there.
type PerSample<E> = Box<dyn Fn(&E) -> Result<f64, InputRejection> + Send + Sync>;

/// A value read off every sample's error in the run, if it gives one.
type OverRun<E> = Box<dyn Fn(&[(MonotonicTime, E)]) -> Option<f64> + Send + Sync>;
