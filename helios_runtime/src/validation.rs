use std::collections::HashSet;

use crate::config::{AutonomyStack, ControllerConfig, EstimatorConfig};

/// Snapshot of algorithm keys registered in each family.
///
/// Family-granular so the validator distinguishes "no Gaussian estimator
/// named X" from "no particle estimator named X" — important once both
/// families have implementations.
pub struct CapabilitySet {
    pub gaussian_estimators: HashSet<String>,
    pub dynamics: HashSet<String>,
    pub measurement_models: HashSet<String>,
    pub mappers: HashSet<String>,
    pub controllers: HashSet<String>,
    pub planners: HashSet<String>,
}

/// Structured validation failure.
#[derive(Debug)]
pub enum ConfigValidationError {
    UnknownGaussianEstimator {
        instance: String,
        kind: String,
    },
    UnknownDynamics {
        kind: String,
    },
    UnknownController {
        kind: String,
    },
    UnknownControllerDynamics {
        controller_kind: String,
        dynamics_key: String,
    },
    UnknownMapper {
        kind: String,
    },
    UnknownPlanner {
        kind: String,
    },
    UnknownMeasurementModel {
        estimator_instance: String,
        model_kind: String,
    },
    UnknownSensorPayload {
        estimator_instance: String,
        payload_kind: String,
    },
}

impl std::fmt::Display for ConfigValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigValidationError::UnknownGaussianEstimator { instance, kind } => {
                write!(
                    f,
                    "Unknown Gaussian estimator kind '{kind}' in estimator '{instance}'"
                )
            }
            ConfigValidationError::UnknownDynamics { kind } => {
                write!(f, "Unknown dynamics kind '{kind}'")
            }
            ConfigValidationError::UnknownController { kind } => {
                write!(f, "Unknown controller kind '{kind}'")
            }
            ConfigValidationError::UnknownControllerDynamics {
                controller_kind,
                dynamics_key,
            } => write!(
                f,
                "Controller '{controller_kind}' references unknown dynamics_key '{dynamics_key}'"
            ),
            ConfigValidationError::UnknownMapper { kind } => {
                write!(f, "Unknown mapper kind '{kind}'")
            }
            ConfigValidationError::UnknownPlanner { kind } => {
                write!(f, "Unknown planner kind '{kind}'")
            }
            ConfigValidationError::UnknownMeasurementModel {
                estimator_instance,
                model_kind,
            } => {
                write!(
                    f,
                    "Estimator '{estimator_instance}' references unknown measurement model kind '{model_kind}'"
                )
            }
            ConfigValidationError::UnknownSensorPayload {
                estimator_instance,
                payload_kind,
            } => {
                write!(
                    f,
                    "Estimator '{estimator_instance}' references unknown sensor payload '{payload_kind}'"
                )
            }
        }
    }
}

/// Known `SensorPayload` implementor names. These must stay in sync with the
/// types that implement `SensorPayload` in `helios_core::data::sensor`.
///
/// When a new sensor payload type is added to `helios_core`, add its name here.
/// A future registry-based approach would make this dynamic, but an inline
/// list is sufficient while the set is small.
const KNOWN_SENSOR_PAYLOADS: &[&str] = &[
    "GpsPosition",
    "GpsVelocity",
    "LinearAcceleration3D",
    "AngularVelocity3D",
    "MagneticField3D",
];

/// Validates `config` against `capabilities`, collecting all errors.
/// Returns an empty `Vec` when the config is fully valid.
pub fn validate_autonomy_config(
    config: &AutonomyStack,
    capabilities: &CapabilitySet,
) -> Vec<ConfigValidationError> {
    let mut errors = Vec::new();

    // Estimator validation (all named instances).
    for (instance, est_cfg) in &config.estimators {
        let kind = est_cfg.get_kind_str();
        if !capabilities.gaussian_estimators.contains(kind) {
            errors.push(ConfigValidationError::UnknownGaussianEstimator {
                instance: instance.clone(),
                kind: kind.to_string(),
            });
        }

        // Validate dynamics and aiding for EKF configs.
        if let EstimatorConfig::Ekf(ekf) = est_cfg {
            let dyn_kind = ekf.dynamics.get_kind_str();
            if !capabilities.dynamics.contains(dyn_kind) {
                errors.push(ConfigValidationError::UnknownDynamics {
                    kind: dyn_kind.to_string(),
                });
            }

            for aiding in &ekf.aiding {
                if !capabilities.measurement_models.contains(&aiding.model.kind) {
                    errors.push(ConfigValidationError::UnknownMeasurementModel {
                        estimator_instance: instance.clone(),
                        model_kind: aiding.model.kind.clone(),
                    });
                }

                if !KNOWN_SENSOR_PAYLOADS.contains(&aiding.sensor_payload.as_str()) {
                    errors.push(ConfigValidationError::UnknownSensorPayload {
                        estimator_instance: instance.clone(),
                        payload_kind: aiding.sensor_payload.clone(),
                    });
                }
            }
        }
    }

    // Map layer validation.
    for map_cfg in config.map_layers.values() {
        let kind = map_cfg.get_kind_str();
        if kind != "None" && !capabilities.mappers.contains(kind) {
            errors.push(ConfigValidationError::UnknownMapper {
                kind: kind.to_string(),
            });
        }
    }

    // Controller validation.
    for ctrl_cfg in config.controllers.values() {
        let kind = ctrl_cfg.get_kind_str();
        if !capabilities.controllers.contains(kind) {
            errors.push(ConfigValidationError::UnknownController {
                kind: kind.to_string(),
            });
        }
        if let ControllerConfig::FeedforwardPid { dynamics_key, .. } = ctrl_cfg {
            if !capabilities.dynamics.contains(dynamics_key.as_str()) {
                errors.push(ConfigValidationError::UnknownControllerDynamics {
                    controller_kind: kind.to_string(),
                    dynamics_key: dynamics_key.clone(),
                });
            }
        }
    }

    // Planner validation.
    for plan_cfg in config.search_planners.values() {
        let kind = plan_cfg.get_kind_str();
        if !capabilities.planners.contains(kind) {
            errors.push(ConfigValidationError::UnknownPlanner {
                kind: kind.to_string(),
            });
        }
    }

    errors
}
