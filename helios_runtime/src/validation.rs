use crate::config::{AutonomyStack, EstimatorConfig};

use std::collections::{BTreeMap, HashSet};

/// Snapshot of algorithm keys registered in each family.
///
/// Family-granular so the validator distinguishes "no Gaussian estimator
/// named X" from "no particle estimator named X" — important once both
/// families have implementations.
pub struct CapabilitySet {
    pub gaussian_estimators: HashSet<String>,
    /// Mock estimators (e.g. `MockOracle`): a separate registry family, so a
    /// stack that selects one is checked here, not against the Gaussian set.
    pub mock_estimators: HashSet<String>,
    pub measurement_models: HashSet<String>,
    pub allocators: HashSet<String>,
}

/// Structured validation failure.
#[derive(Debug)]
pub enum ConfigValidationError {
    UnknownGaussianEstimator {
        instance: String,
        kind: String,
    },
    UnknownMockEstimator {
        instance: String,
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
    /// An augmentation declares a `sensor` that no aiding entry feeds. The
    /// appended nuisance block would ride through predict untouched — never
    /// observed, a silent no-op that only inflates the state. A block is
    /// observable only through an aiding source on its own sensor channel.
    AugmentationHasNoAidingSource {
        estimator_instance: String,
        kind: String,
        sensor: String,
    },
    UnknownAllocator {
        kind: String,
    },

    /// Two or more allocators name the same actuator. Decoupled control merges
    /// several allocators' outputs into one terminal command by unioning their
    /// disjoint actuator sets; a shared actuator breaks that disjointness, so the
    /// merge would keep one allocator's setpoint and silently drop the other's.
    /// Each allocator config declares the actuators it drives, so the collision
    /// is caught here at load rather than as a dropped setpoint at runtime.
    AllocatorActuatorConflict {
        actuator: String,
        allocators: Vec<String>,
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
            ConfigValidationError::UnknownMockEstimator { instance, kind } => {
                write!(
                    f,
                    "Unknown mock estimator kind '{kind}' in estimator '{instance}'"
                )
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
            ConfigValidationError::AugmentationHasNoAidingSource {
                estimator_instance,
                kind,
                sensor,
            } => {
                write!(
                    f,
                    "Estimator '{estimator_instance}' augmentation '{kind}' names sensor '{sensor}', but no aiding entry feeds that channel; the block would never be observed"
                )
            }
            ConfigValidationError::UnknownAllocator { kind } => {
                write!(f, "Unknown allocator kind '{kind}'")
            }
            ConfigValidationError::AllocatorActuatorConflict {
                actuator,
                allocators,
            } => {
                let allocators = allocators.join(", ");
                write!(
                    f,
                    "actuator '{actuator}' is claimed by more than one allocator ({allocators}); each actuator must be owned by exactly one"
                )
            }
        }
    }
}

/// Known `SensorPayload` implementor names. These must stay in sync with the
/// types that implement `SensorPayload` in `helios_core::interchange::measurement::sensor`.
///
/// When a new sensor payload type is added to `helios_core`, add its name here.
/// A future registry-based approach would make this dynamic, but an inline
/// list is sufficient while the set is small.
const KNOWN_SENSOR_PAYLOADS: &[&str] = &[
    "GpsPosition",
    "GpsVelocity",
    "Acceleration",
    "AngularRate",
    "MagneticField",
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
        // Each estimator variant is built by its own registry family, so its
        // kind is checked against that family's factories.
        let kind = est_cfg.get_kind_str();
        match est_cfg {
            EstimatorConfig::MockOracle(_) => {
                if !capabilities.mock_estimators.contains(kind) {
                    errors.push(ConfigValidationError::UnknownMockEstimator {
                        instance: instance.clone(),
                        kind: kind.to_string(),
                    });
                }
            }
            EstimatorConfig::Ekf(_) | EstimatorConfig::Ukf(_) => {
                if !capabilities.gaussian_estimators.contains(kind) {
                    errors.push(ConfigValidationError::UnknownGaussianEstimator {
                        instance: instance.clone(),
                        kind: kind.to_string(),
                    });
                }
            }
        }

        // Validate dynamics and aiding for EKF configs.
        if let EstimatorConfig::Ekf(ekf) = est_cfg {
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

            // Each augmentation is observed only through an aiding source on the
            // same sensor channel; without one the appended block has no
            // measurement touching its columns and rides inertly. Catch it here
            // rather than let it be a silent runtime no-op.
            for aug in &ekf.augmentation {
                let has_aiding_source = ekf
                    .aiding
                    .iter()
                    .any(|aiding| aiding.input_channel == aug.sensor);
                if !has_aiding_source {
                    errors.push(ConfigValidationError::AugmentationHasNoAidingSource {
                        estimator_instance: instance.clone(),
                        kind: aug.kind.clone(),
                        sensor: aug.sensor.clone(),
                    });
                }
            }
        }
    }

    // Allocator validation.
    for alloc_cfg in config.allocators.values() {
        let kind = alloc_cfg.get_kind_str();
        if !capabilities.allocators.contains(kind) {
            errors.push(ConfigValidationError::UnknownAllocator {
                kind: kind.to_string(),
            });
        }
    }

    // Allocator cross-field check. The per-kind check above rejects unknown
    // allocators; this one catches a well-formed allocator that fights another
    // for the same actuator, which would otherwise surface late as two writers
    // racing one terminal slot.
    //
    // Decoupled control lets several allocators coexist, each owning a disjoint
    // set of actuators that a downstream merge unions into the one terminal
    // command. That union is only well-defined if no two allocators claim the
    // same actuator: a double-claimed actuator would take its setpoint from
    // whichever allocator the merge saw first, silently dropping the other. Each
    // allocator config names the actuators it drives, so this half of the
    // partition — disjointness — is checkable here. The other half, totality
    // (every physical actuator is claimed by some allocator), needs the body's
    // actuation model and so is the host's to check at spawn.
    //
    // A BTreeMap and the per-conflict sort keep the emitted errors ordered by
    // actuator, then by allocator name, so the report is stable across runs
    // regardless of the source HashMap's iteration order.
    let mut actuator_to_allocators: BTreeMap<&str, Vec<String>> = BTreeMap::new();
    for (allocator, cfg) in &config.allocators {
        for actuator in cfg.actuator_ids() {
            actuator_to_allocators
                .entry(actuator)
                .or_default()
                .push(allocator.to_string());
        }
    }

    for (actuator, allocators) in &actuator_to_allocators {
        if allocators.len() > 1 {
            let mut a = allocators.clone();
            a.sort();
            errors.push(ConfigValidationError::AllocatorActuatorConflict {
                actuator: actuator.to_string(),
                allocators: a,
            });
        }
    }

    errors
}
