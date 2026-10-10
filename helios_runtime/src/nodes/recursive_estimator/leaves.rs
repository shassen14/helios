//! What a recursive estimator reports to watchers: for each aiding source,
//! the leaves in [`AidingLeaf`], under `aiding.<entry>.<leaf>`.

use crate::observe::path::PATH_SEPARATOR;
use crate::port::Determinism;

use std::sync::Arc;

/// The group every aiding source's leaves sit under, followed by the source's
/// entry name.
pub(crate) const AIDING_LEAF_GROUP: &str = "aiding";

/// The value reported on [`AidingLeaf::Dropped`] for one dropped correction,
/// so a sink sums the samples to count drops.
pub(crate) const ONE_DROP: f64 = 1.0;

/// One value an aiding source reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AidingLeaf {
    /// An applied correction's NIS ÷ dof: dimensionless, one sample per
    /// applied update, stamped with the reading's time.
    Nis,
    /// A correction dropped for a fault: [`ONE_DROP`] per drop, stamped with
    /// the reading's time. Expected quiet skips are not counted.
    Dropped,
}

impl AidingLeaf {
    /// Every leaf. Rust can't list an enum's variants, so a new variant must
    /// be added here as well; one left out is never declared, and reporting it
    /// trips a debug assertion.
    pub(crate) const ALL: [Self; 2] = [Self::Nis, Self::Dropped];

    /// The leaf's last path segment.
    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Nis => "nis",
            Self::Dropped => "dropped",
        }
    }

    /// Whether the leaf's values repeat when the same inputs are replayed.
    pub(crate) fn determinism(self) -> Determinism {
        match self {
            Self::Nis | Self::Dropped => Determinism::Reproducible,
        }
    }
}

/// The paths one aiding source reports under, built once from its entry name
/// so the declarations and the reports share the same strings.
pub(crate) struct AidingLeaves {
    paths: [(AidingLeaf, Arc<str>); AidingLeaf::ALL.len()],
}

impl AidingLeaves {
    /// Every leaf of the aiding entry named `entry`.
    pub(crate) fn new(entry: &str) -> Self {
        Self {
            paths: AidingLeaf::ALL.map(|leaf| {
                let path = [AIDING_LEAF_GROUP, entry, leaf.name()].join(PATH_SEPARATOR);
                (leaf, Arc::from(path))
            }),
        }
    }

    /// The path `leaf` reports under, or `None` if it is missing from
    /// [`AidingLeaf::ALL`].
    pub(crate) fn path(&self, leaf: AidingLeaf) -> Option<&Arc<str>> {
        self.paths
            .iter()
            .find(|(each, _)| *each == leaf)
            .map(|(_, path)| path)
    }

    /// Every leaf with its path, in [`AidingLeaf::ALL`] order.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (AidingLeaf, &Arc<str>)> {
        self.paths.iter().map(|(leaf, path)| (*leaf, path))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_leaf_sits_under_the_group_and_the_entry() {
        let leaves = AidingLeaves::new("gps");

        let paths: Vec<&str> = leaves.iter().map(|(_, path)| path.as_ref()).collect();
        assert_eq!(paths, ["aiding.gps.nis", "aiding.gps.dropped"]);
    }
}
