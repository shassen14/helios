// helios_sim/src/simulation/profile.rs
//
// SimulationProfile: selects which subsystems the simulation runs.
// Used by ProfiledSimulationPlugin to add only the required plugins.

use bevy::prelude::Resource;

/// Flat set of boolean capabilities derived from a `SimulationProfile`.
/// Inserted as a Bevy resource by `ProfiledSimulationPlugin` so any system
/// can read which subsystems are active without re-deriving from the profile.
///
/// Always construct via [`SimulationProfile::capabilities()`] — never build
/// this struct directly, as some field combinations are invalid at runtime
/// (e.g. `real_estimation` and `mock_estimator` are mutually exclusive).
#[derive(Resource, Debug, Clone, Default)]
pub struct CapabilitySet {
    sensors: bool,
    real_estimation: bool,
    mock_estimator: bool,
    mapping: bool,
    planning: bool,
    control: bool,
    mock_path: bool,
    mock_map: bool,
    metrics: bool,
}

impl CapabilitySet {
    pub fn sensors(&self) -> bool {
        self.sensors
    }
    pub fn real_estimation(&self) -> bool {
        self.real_estimation
    }
    pub fn mock_estimator(&self) -> bool {
        self.mock_estimator
    }
    pub fn mapping(&self) -> bool {
        self.mapping
    }
    pub fn planning(&self) -> bool {
        self.planning
    }
    pub fn control(&self) -> bool {
        self.control
    }
    pub fn mock_path(&self) -> bool {
        self.mock_path
    }
    pub fn mock_map(&self) -> bool {
        self.mock_map
    }
    pub fn metrics(&self) -> bool {
        self.metrics
    }

    /// True if any estimator (real or mock ground-truth) is active.
    pub fn estimation(&self) -> bool {
        self.real_estimation || self.mock_estimator
    }
}

/// Selects which subsystems the simulation runs.
///
/// Passed either via the `--profile` CLI flag or the `[simulation] profile` TOML field.
/// CLI flag takes precedence over TOML.
#[derive(Resource, Debug, Clone, PartialEq, Eq, Default)]
pub enum SimulationProfile {
    /// Full autonomy pipeline: sensors + estimation + mapping + planning + control.
    #[default]
    FullPipeline,
    /// Sensors + estimation only.  No mapping, planning, or control.
    EstimationOnly,
    /// Sensors + ground-truth estimator + mapping.  No planning or control.
    MappingOnly,
    /// Static map + ground-truth estimator + planning.  No sensors or control.
    PlanningOnly,
    /// Static path + ground-truth estimator + control + metrics.  No sensors or mapping.
    ControlOnly,
    /// Sensors + real estimation + control + static path + metrics.
    PathTracking,
}

impl SimulationProfile {
    /// Parse from the string stored in `[simulation] profile` TOML.
    pub fn from_str_opt(s: &str) -> Option<Self> {
        match s.to_lowercase().replace('-', "_").as_str() {
            "full_pipeline" | "full" => Some(Self::FullPipeline),
            "estimation_only" | "estimation" => Some(Self::EstimationOnly),
            "mapping_only" | "mapping" => Some(Self::MappingOnly),
            "planning_only" | "planning" => Some(Self::PlanningOnly),
            "control_only" | "control" => Some(Self::ControlOnly),
            "path_tracking" | "tracking" => Some(Self::PathTracking),
            _ => None,
        }
    }

    /// Derive the flat capability set for this profile.
    pub fn capabilities(&self) -> CapabilitySet {
        match self {
            Self::FullPipeline => CapabilitySet {
                sensors: true,
                real_estimation: true,
                mapping: true,
                planning: true,
                control: true,
                ..Default::default()
            },
            Self::EstimationOnly => CapabilitySet {
                sensors: true,
                real_estimation: true,
                ..Default::default()
            },
            Self::MappingOnly => CapabilitySet {
                sensors: true,
                mock_estimator: true,
                mapping: true,
                ..Default::default()
            },
            Self::PlanningOnly => CapabilitySet {
                mock_estimator: true,
                planning: true,
                mock_map: true,
                ..Default::default()
            },
            Self::ControlOnly => CapabilitySet {
                mock_estimator: true,
                control: true,
                mock_path: true,
                metrics: true,
                ..Default::default()
            },
            Self::PathTracking => CapabilitySet {
                sensors: true,
                real_estimation: true,
                control: true,
                mock_path: true,
                metrics: true,
                ..Default::default()
            },
        }
    }
}
