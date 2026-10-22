//! How every shape's registry adapter states a kind's menu, and checks that
//! a bound evaluation offers exactly the menu its kind fixed.

use super::component::Component;
use crate::evaluation::kind::Metric;

/// The metrics `components` offer, in order.
pub(super) fn metrics_of<O>(components: &[Component<O>]) -> Vec<Metric> {
    components.iter().map(Component::metric).collect()
}

/// Whether an evaluation of the kind `kind`, bound, offers `offered` as its
/// `menu` fixed it, or why not. A kind with no fixed menu offers whatever it
/// binds to. A fixed menu is the kind's components written a second time,
/// one read before binding and one after, so the types can't make them one
/// list; load checks a run file's names against the first and setup against
/// the second.
pub(super) fn offers_its_menu(
    kind: &str,
    menu: Option<Vec<Metric>>,
    offered: &[Metric],
) -> Result<(), Vec<String>> {
    match menu {
        Some(menu) if menu != offered => Err(vec![format!(
            "{kind} offers {} once bound, but its menu is {}: a kind with a fixed menu \
             offers exactly its menu",
            listed(offered),
            listed(&menu)
        )]),
        _ => Ok(()),
    }
}

/// `metrics` as a run file names them, each with its unit.
fn listed(metrics: &[Metric]) -> String {
    let each: Vec<String> = metrics
        .iter()
        .map(|metric| format!("{} ({})", metric.name, metric.si_unit))
        .collect();
    format!("[{}]", each.join(", "))
}
