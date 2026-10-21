//! [`Component`]: one metric an evaluator offers, read off its evaluator's
//! output at each sample.

use crate::evaluation::{
    derived::Refused,
    kind::{InputRejection, Metric},
};

use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::MonotonicTime;

/// _Pre-step._ One metric of an evaluator whose output is `E`: its name and
/// unit, and how its values are read. A run file names it as
/// `eval.inline.<name>.<metric>`.
pub struct Component<E> {
    name: &'static str,
    si_unit: SiUnit,
    value: PerSample<E>,
}

impl<E> Component<E> {
    /// A metric with one value per sample, read off its output, or refused
    /// there in the kind's own words: a heading of a frame that faces no way.
    pub fn each(
        name: &'static str,
        si_unit: SiUnit,
        value: impl Fn(&E) -> Result<f64, InputRejection> + Send + Sync + 'static,
    ) -> Self {
        Self {
            name,
            si_unit,
            value: Box::new(value),
        }
    }

    /// Its name and unit.
    pub fn metric(&self) -> Metric {
        Metric {
            name: self.name,
            si_unit: self.si_unit,
        }
    }

    /// Its values over `outputs`, every computed sample in stamp order. A
    /// sample it can't give is counted in `refused`.
    pub(crate) fn values(
        &self,
        outputs: &[(MonotonicTime, E)],
        refused: &mut Refused,
    ) -> Vec<(MonotonicTime, f64)> {
        outputs
            .iter()
            .filter_map(|(at, output)| match (self.value)(output) {
                Ok(value) => Some((*at, value)),
                Err(rejection) => {
                    refused.add(rejection);
                    None
                }
            })
            .collect()
    }
}

/// A value read off one sample's output, or refused there.
type PerSample<E> = Box<dyn Fn(&E) -> Result<f64, InputRejection> + Send + Sync>;
