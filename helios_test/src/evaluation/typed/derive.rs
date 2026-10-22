//! [`derive`]: what every evaluation shape derives at finalize from its
//! lined-up rows, fitted, computed and read into components.

use super::{buffer::LinedUp, component::Component};
use crate::evaluation::{
    derived::{Derived, Refused},
    kind::InputRejection,
};

use helios_core::prelude::MonotonicTime;

/// What every shape derives from its lined-up `rows`: every covered sample
/// is fitted over once, then each is computed, refusing what can't be, as
/// the base values between two values the kind couldn't blend are;
/// each wanted component is read off what was computed. `inputs` is each
/// role's refusals, in role order.
pub(super) fn derive<R: Copy, F, O>(
    rows: LinedUp<R>,
    fit: impl FnOnce(&[R]) -> Result<F, InputRejection>,
    mut compute: impl FnMut(R, &F) -> Result<O, InputRejection>,
    components: &[Component<O>],
    inputs: &[Refused],
    wanted: &[usize],
) -> Derived {
    let run: Vec<R> = rows.rows.iter().map(|row| row.values).collect();

    let mut not_computed = rows.unblended;
    let mut values_refused = vec![Refused::default(); components.len()];
    let metrics = match fit(&run) {
        Ok(fit) => {
            let outputs: Vec<(MonotonicTime, O)> = rows
                .rows
                .iter()
                .filter_map(|row| match compute(row.values, &fit) {
                    Ok(output) => Some((row.at, output)),
                    Err(rejection) => {
                        not_computed.add(rejection);
                        None
                    }
                })
                .collect();
            components
                .iter()
                .zip(&mut values_refused)
                .enumerate()
                .map(|(index, (component, refused))| {
                    wanted
                        .contains(&index)
                        .then(|| component.values(&outputs, refused))
                })
                .collect()
        }
        // A run refused whole: every wanted metric has no values, and
        // still carries the refusal to the entries reading it.
        Err(rejection) => {
            run.iter().for_each(|_| not_computed.add(rejection));
            (0..components.len())
                .map(|index| wanted.contains(&index).then(Vec::new))
                .collect()
        }
    };
    Derived::new(
        rows.base_samples,
        rows.uncovered,
        rows.coverage,
        inputs,
        not_computed,
        metrics,
        values_refused,
    )
}
