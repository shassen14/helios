//! [`RecursiveEstimatorConfig`]: the `[nodes.<name>]` section of a
//! `RecursiveEstimator` entry.
//!
//! The `filter`, `dynamics` and each aiding `model` are component sub-tables:
//! each names its own `kind`, and the component table for that slot parses
//! the rest. They are held raw here, so this config never lists a kind's
//! keys.

use serde::{Deserialize, Serialize};

use std::collections::BTreeMap;

/// The section of a `RecursiveEstimator` node.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct RecursiveEstimatorConfig {
    /// The recursive filter (`kind = "Ekf"`, …) and its own parameters.
    pub(crate) filter: toml::Table,
    /// The process model, the input that drives its predict and their
    /// parameters (`kind = "IntegratedImu"`, …).
    pub(crate) dynamics: toml::Table,
    /// The sensors that correct the estimate, keyed by a name of the
    /// profile's choosing. Applied in key order each tick.
    #[serde(default)]
    pub(crate) aiding: BTreeMap<String, AidingEntry>,
    /// Nuisance parameters estimated alongside the state, appended to the
    /// dynamics' state in list order.
    #[serde(default)]
    pub(crate) augmentation: Vec<AugmentationEntry>,
    /// The pose the filter starts from. Its uncertainty is the dynamics'.
    #[serde(default)]
    pub(crate) initial_pose: InitialPoseConfig,
}

/// One aiding sensor: where its readings arrive, how noisy they are, and the
/// model that predicts them.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct AidingEntry {
    /// The host sensor channel the readings arrive on. Also the leaf of the
    /// sensor's frame.
    pub(crate) input: String,
    /// Diagonal of the measurement noise covariance R; its length must equal
    /// the model's measurement length.
    pub(crate) r_diag: Vec<f64>,
    /// The measurement model (`kind = "gps_position"`, …). The kind fixes the
    /// payload type read from `input`.
    pub(crate) model: toml::Table,
}

/// One nuisance block appended to the state, calibrated through an aiding
/// sensor.
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct AugmentationEntry {
    /// Augmentation kind, one of the reserved kinds in
    /// `helios_core::estimation::augmentation` (e.g. the magnetometer bias).
    pub(crate) kind: String,
    /// The `input` of the aiding entry whose sensor this block calibrates.
    /// That sensor's measurements are the only thing that observes the block.
    pub(crate) sensor: String,
    /// Prior std dev on each axis, in the block's units; squared onto P₀.
    pub(crate) init_uncertainty: f64,
    /// Per-axis random-walk std dev; forms the block's Q.
    pub(crate) random_walk: f64,
}

/// The mean of the starting pose of `base_link` in `odom`.
///
/// All zero by default: a cold start whose pose is unknown until the aiding
/// sensors pin it, which the dynamics' prior uncertainty must allow for.
#[derive(Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct InitialPoseConfig {
    /// East (m).
    #[serde(default)]
    pub(crate) x: f64,
    /// North (m).
    #[serde(default)]
    pub(crate) y: f64,
    /// Up (m).
    #[serde(default)]
    pub(crate) z: f64,
    /// Yaw from east toward north. Degrees in config; converted to radians on
    /// load.
    #[serde(default)]
    pub(crate) heading_deg: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    const MINIMAL: &str = r#"
        filter = { kind = "Ekf" }
        dynamics = { kind = "IntegratedImu" }
    "#;

    #[test]
    fn a_section_with_only_filter_and_dynamics_parses() {
        let config: RecursiveEstimatorConfig = toml::from_str(MINIMAL).expect("a valid section");
        assert!(config.aiding.is_empty());
        assert!(config.augmentation.is_empty());
        assert_eq!(config.initial_pose.heading_deg, 0.0);
    }

    // The registry relies on `deny_unknown_fields` to turn a misspelled key
    // into an error instead of a silently ignored one, at every level the
    // node owns.
    #[test]
    fn a_misspelled_key_is_rejected_at_every_level() {
        for extra in [
            "agiding = {}",
            "[initial_pose]\nheadng_deg = 90.0",
            "[aiding.gps]\ninput = \"gps\"\nr_diag = []\nmodel = {}\nr_diagg = []",
        ] {
            let section = format!("{MINIMAL}\n{extra}");
            assert!(
                toml::from_str::<RecursiveEstimatorConfig>(&section).is_err(),
                "accepted: {extra}"
            );
        }
    }

    // Aiding entries come back in key order, which is the order they are
    // applied in.
    #[test]
    fn aiding_entries_are_ordered_by_name() {
        let section = format!(
            "{MINIMAL}
            [aiding.mag]
            input = \"mag\"
            r_diag = [1.0, 1.0, 1.0]
            model = {{ kind = \"magnetometer\" }}

            [aiding.gps]
            input = \"gps\"
            r_diag = [1.0, 1.0, 1.0]
            model = {{ kind = \"gps_position\" }}"
        );
        let config: RecursiveEstimatorConfig = toml::from_str(&section).expect("a valid section");
        let names: Vec<&str> = config.aiding.keys().map(String::as_str).collect();
        assert_eq!(names, ["gps", "mag"]);
    }
}
