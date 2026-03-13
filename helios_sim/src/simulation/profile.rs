// helios_sim/src/simulation/profile.rs
//
// SimulationProfile: selects which subsystems the simulation runs.
// Used by ProfiledSimulationPlugin to add only the required plugins.

use bevy::prelude::Resource;

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

    // -----------------------------------------------------------------------
    // Capability predicates — used by ProfiledSimulationPlugin
    // -----------------------------------------------------------------------

    /// Whether hardware sensor plugins (IMU, GPS, LiDAR, …) should be added.
    pub fn needs_sensors(&self) -> bool {
        matches!(
            self,
            Self::FullPipeline | Self::EstimationOnly | Self::MappingOnly | Self::PathTracking
        )
    }

    /// Whether the real EKF/UKF estimator should be active.
    pub fn needs_real_estimation(&self) -> bool {
        matches!(
            self,
            Self::FullPipeline | Self::EstimationOnly | Self::PathTracking
        )
    }

    /// Whether a ground-truth passthrough mock estimator should replace the real one.
    pub fn needs_mock_estimator(&self) -> bool {
        matches!(
            self,
            Self::MappingOnly | Self::PlanningOnly | Self::ControlOnly
        )
    }

    /// Whether the mapping subsystem should be active.
    pub fn needs_mapping(&self) -> bool {
        matches!(self, Self::FullPipeline | Self::MappingOnly)
    }

    /// Whether the planning subsystem should be active.
    pub fn needs_planning(&self) -> bool {
        matches!(self, Self::FullPipeline | Self::PlanningOnly)
    }

    /// Whether the control subsystem should be active.
    pub fn needs_control(&self) -> bool {
        matches!(
            self,
            Self::FullPipeline | Self::ControlOnly | Self::PathTracking
        )
    }

    /// Whether a baked path fixture should be injected (bypasses the planner).
    pub fn needs_mock_path(&self) -> bool {
        matches!(self, Self::ControlOnly | Self::PathTracking)
    }

    /// Whether a baked map fixture should be injected (bypasses the mapper).
    pub fn needs_mock_map(&self) -> bool {
        matches!(self, Self::PlanningOnly)
    }

    /// Whether the control-quality metrics plugin should be active.
    pub fn needs_metrics(&self) -> bool {
        matches!(self, Self::ControlOnly | Self::PathTracking)
    }
}
