use std::collections::HashSet;

use crate::config::{AutonomyStack, CommandSource, EstimatorConfig, MapLayerConfig};

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
    pub allocators: HashSet<String>,
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
    /// A planner's `level` names no active map layer, so nothing produces the
    /// `MapData` it reads. The layer is either absent or declared `None`.
    PlannerReferencesUnknownMapLayer {
        planner: String,
        level: String,
    },
    UnknownMeasurementModel {
        estimator_instance: String,
        model_kind: String,
    },
    UnknownSensorPayload {
        estimator_instance: String,
        payload_kind: String,
    },
    UnknownAllocator {
        kind: String,
    },

    /// `command_arbitration` lists `autonomy` as a source, but no controller is
    /// configured to produce the autonomy command.
    AutonomySourceWithoutController,
    /// A controller is configured, but `autonomy` is not among the explicitly
    /// listed command sources, so the controller's output is never routed to
    /// `command`.
    ControllerConfiguredButNotACommandSource,
    /// The same command source appears more than once in
    /// `command_arbitration.sources`, making priority order ambiguous.
    DuplicateCommandSource {
        source: String,
    },

    /// An allocator is configured, but nothing produces the `command` it
    /// consumes — no controller and no teleop source — so the actuator terminal
    /// has no input. The terminal-side twin of `AutonomySourceWithoutController`.
    AllocatorWithoutCommandSource,
    /// More than one allocator is configured. The actuator terminal is a single
    /// canonical channel; every allocator writes the same key, so two collide.
    /// Exactly one allocator owns the terminal (its `ActuatorCommand` must be
    /// total over the body's actuators).
    MultipleAllocators {
        count: usize,
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
            ConfigValidationError::PlannerReferencesUnknownMapLayer { planner, level } => {
                write!(
                    f,
                    "Planner '{planner}' reads map layer '{level}', but no active map layer of that name is declared"
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
            ConfigValidationError::UnknownAllocator { kind } => {
                write!(f, "Unknown allocator kind '{kind}'")
            }
            ConfigValidationError::AutonomySourceWithoutController => {
                write!(
                    f,
                    "command_arbitration lists 'autonomy' as a source but no controller is configured to produce it"
                )
            }
            ConfigValidationError::ControllerConfiguredButNotACommandSource => {
                write!(
                    f,
                    "a controller is configured but 'autonomy' is not among command_arbitration.sources; its output is never routed to command"
                )
            }
            ConfigValidationError::DuplicateCommandSource { source } => {
                write!(
                    f,
                    "command_arbitration.sources lists '{source}' more than once"
                )
            }
            ConfigValidationError::AllocatorWithoutCommandSource => {
                write!(
                    f,
                    "an allocator is configured but nothing produces the 'command' it consumes (no controller and no teleop source)"
                )
            }
            ConfigValidationError::MultipleAllocators { count } => {
                write!(
                    f,
                    "{count} allocators are configured, but the actuator terminal is a single channel that exactly one allocator may own"
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

    // Planner validation.
    for (instance, plan_cfg) in &config.search_planners {
        let kind = plan_cfg.get_kind_str();
        if !capabilities.planners.contains(kind) {
            errors.push(ConfigValidationError::UnknownPlanner {
                kind: kind.to_string(),
            });
        }

        // The planner reads its `MapData` from the channel named by `level`; the
        // mapper of that same config-map key produces it. A `level` naming no
        // active layer would only surface as an `UnsatisfiedInput` at DAG build.
        let level = plan_cfg.get_level_str();
        let layer_is_active = config
            .map_layers
            .get(level)
            .is_some_and(|layer| !matches!(layer, MapLayerConfig::None));
        if !layer_is_active {
            errors.push(ConfigValidationError::PlannerReferencesUnknownMapLayer {
                planner: instance.clone(),
                level: level.to_string(),
            });
        }
    }

    // Command arbitration validation.
    let sources = &config.command_arbitration.sources;
    let has_controller = !config.controllers.is_empty();
    let lists_autonomy = sources.contains(&CommandSource::Autonomy);

    // An explicit autonomy source with nothing to produce it.
    if lists_autonomy && !has_controller {
        errors.push(ConfigValidationError::AutonomySourceWithoutController);
    }

    // A controller whose output is never routed to `command`. An empty sources
    // list infers `[Autonomy]`, so this only fires when the list is explicit
    // and omits autonomy.
    if has_controller && !sources.is_empty() && !lists_autonomy {
        errors.push(ConfigValidationError::ControllerConfiguredButNotACommandSource);
    }

    // A source listed more than once makes priority order ambiguous.
    let mut seen = HashSet::new();
    for source in sources {
        if !seen.insert(*source) {
            errors.push(ConfigValidationError::DuplicateCommandSource {
                source: source.as_str().to_string(),
            });
        }
    }

    // Allocator cross-field checks. The per-kind check above rejects unknown
    // allocators; these two catch a well-formed allocator wired into a graph
    // that can't feed or hold it, which would otherwise surface late as an
    // UnsatisfiedInput / duplicate-producer failure at DAG build.

    // The allocator consumes `command`; `command` is produced only by a
    // controller or a host-published teleop source. With neither, its input is
    // unsatisfiable. Mirrors the assembler's `CommandTopology::None` case.
    let command_is_produced = has_controller || sources.contains(&CommandSource::Teleop);
    if !config.allocators.is_empty() && !command_is_produced {
        errors.push(ConfigValidationError::AllocatorWithoutCommandSource);
    }

    // The actuator terminal is one canonical channel keyed only by type, so two
    // allocators write the same slot. Exactly one may own it.
    if config.allocators.len() > 1 {
        errors.push(ConfigValidationError::MultipleAllocators {
            count: config.allocators.len(),
        });
    }

    errors
}
