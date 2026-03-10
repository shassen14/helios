use std::collections::HashSet;

use crate::config::{AutonomyStack, ControllerConfig, WorldModelConfig};

/// Snapshot of algorithm keys registered in each category.
pub struct CapabilitySet {
    pub estimators: HashSet<String>,
    pub dynamics: HashSet<String>,
    pub mappers: HashSet<String>,
    pub slam: HashSet<String>,
    pub controllers: HashSet<String>,
    pub planners: HashSet<String>,
}

/// Structured validation failure.
#[derive(Debug)]
pub enum ValidationError {
    UnknownEstimator { kind: String },
    UnknownDynamics { kind: String },
    UnknownController { kind: String },
    UnknownControllerDynamics { controller_kind: String, dynamics_key: String },
    UnknownMapper { kind: String },
    UnknownPlanner { kind: String },
    UnknownSlam { kind: String },
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ValidationError::UnknownEstimator { kind } =>
                write!(f, "Unknown estimator kind '{kind}'"),
            ValidationError::UnknownDynamics { kind } =>
                write!(f, "Unknown dynamics kind '{kind}'"),
            ValidationError::UnknownController { kind } =>
                write!(f, "Unknown controller kind '{kind}'"),
            ValidationError::UnknownControllerDynamics { controller_kind, dynamics_key } =>
                write!(f, "Controller '{controller_kind}' references unknown dynamics_key '{dynamics_key}'"),
            ValidationError::UnknownMapper { kind } =>
                write!(f, "Unknown mapper kind '{kind}'"),
            ValidationError::UnknownPlanner { kind } =>
                write!(f, "Unknown planner kind '{kind}'"),
            ValidationError::UnknownSlam { kind } =>
                write!(f, "Unknown SLAM kind '{kind}'"),
        }
    }
}

/// Validates `config` against `capabilities`, collecting all errors.
/// Returns an empty `Vec` when the config is fully valid.
pub fn validate_autonomy_config(
    config: &AutonomyStack,
    capabilities: &CapabilitySet,
) -> Vec<ValidationError> {
    let mut errors = Vec::new();

    // World model validation
    match &config.world_model {
        Some(WorldModelConfig::Separate { estimator, mapper }) => {
            if let Some(est_cfg) = estimator {
                let kind = est_cfg.get_kind_str();
                if !capabilities.estimators.contains(kind) {
                    errors.push(ValidationError::UnknownEstimator { kind: kind.to_string() });
                }
                // Validate dynamics referenced by the estimator
                if let crate::config::EstimatorConfig::Ekf(ekf) = est_cfg {
                    let dyn_kind = ekf.dynamics.get_kind_str();
                    if !capabilities.dynamics.contains(dyn_kind) {
                        errors.push(ValidationError::UnknownDynamics { kind: dyn_kind.to_string() });
                    }
                }
            }
            if let Some(map_cfg) = mapper {
                let kind = map_cfg.get_kind_str();
                if kind != "None" && !capabilities.mappers.contains(kind) {
                    errors.push(ValidationError::UnknownMapper { kind: kind.to_string() });
                }
            }
        }
        Some(WorldModelConfig::CombinedSlam { slam }) => {
            let kind = slam.get_kind_str();
            if !capabilities.slam.contains(kind) {
                errors.push(ValidationError::UnknownSlam { kind: kind.to_string() });
            }
        }
        None => {}
    }

    // Controller validation
    for ctrl_cfg in config.controllers.values() {
        let kind = ctrl_cfg.get_kind_str();
        if !capabilities.controllers.contains(kind) {
            errors.push(ValidationError::UnknownController { kind: kind.to_string() });
        }
        // FeedforwardPid references a dynamics_key that must also be registered
        if let ControllerConfig::FeedforwardPid { dynamics_key, .. } = ctrl_cfg {
            if !capabilities.dynamics.contains(dynamics_key.as_str()) {
                errors.push(ValidationError::UnknownControllerDynamics {
                    controller_kind: kind.to_string(),
                    dynamics_key: dynamics_key.clone(),
                });
            }
        }
    }

    // Planner validation
    for plan_cfg in config.planners.values() {
        let kind = plan_cfg.get_kind_str();
        if !capabilities.planners.contains(kind) {
            errors.push(ValidationError::UnknownPlanner { kind: kind.to_string() });
        }
    }

    errors
}
