use std::collections::{BTreeMap, HashSet};

use crate::config::{AutonomyStack, CommandSource, CommandSpace, EstimatorConfig, MapLayerConfig};

/// Snapshot of algorithm keys registered in each family.
///
/// Family-granular so the validator distinguishes "no Gaussian estimator
/// named X" from "no particle estimator named X" — important once both
/// families have implementations.
pub struct CapabilitySet {
    pub gaussian_estimators: HashSet<String>,
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
    UnknownController {
        kind: String,
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

    /// A controller emits a different command space than the allocator consumes.
    /// The allocator defines the command seam, and the assembler instantiates the
    /// `command` channel as one concrete `command::<T>()`; a controller speaking a
    /// different space writes a slot the allocator never reads. The DAG erases the
    /// type at the channel boundary, so this would otherwise surface late as an
    /// `UnsatisfiedInput` (the allocator's input unfilled, the controller's
    /// contribution an orphan). Caught here at load time instead.
    ControllerCommandSpaceMismatch {
        controller: String,
        controller_space: CommandSpace,
        allocator_space: CommandSpace,
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
            ConfigValidationError::UnknownController { kind } => {
                write!(f, "Unknown controller kind '{kind}'")
            }
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
            ConfigValidationError::ControllerCommandSpaceMismatch {
                controller,
                controller_space,
                allocator_space,
            } => {
                write!(
                    f,
                    "controller '{controller}' emits {controller_space:?} but the allocator's command space is {allocator_space:?}; every controller feeding the allocator must speak its command space"
                )
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
    // allocators; these catch a well-formed allocator wired into a graph that
    // can't feed it or that fights another allocator for the same actuator,
    // each of which would otherwise surface late and cryptically at DAG build
    // (an UnsatisfiedInput, or two writers racing one terminal slot).

    // The allocator consumes `command`; `command` is produced only by a
    // controller or a host-published teleop source. With neither, its input is
    // unsatisfiable. Mirrors the assembler's `CommandTopology::None` case.
    let command_is_produced = has_controller || sources.contains(&CommandSource::Teleop);
    if !config.allocators.is_empty() && !command_is_produced {
        errors.push(ConfigValidationError::AllocatorWithoutCommandSource);
    }

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

    // Command-space agreement. Every controller writes its contribution into the
    // fold that feeds the allocator's `command` input; the allocator defines the
    // seam, so each controller must speak its space or its output lands in a slot
    // nothing reads. This binds only the single-allocator case — one seam, one
    // space to match against. A multi-allocator stack has several seams at once,
    // and matching each controller against the set of allocator spaces is a
    // wider check not yet performed here; an empty allocator set has no seam.
    if let [allocator] = config.allocators.values().collect::<Vec<_>>().as_slice() {
        let allocator_space = allocator.command_space();
        for (name, ctrl_cfg) in &config.controllers {
            let controller_space = ctrl_cfg.command_space();
            if allocator_space != controller_space {
                errors.push(ConfigValidationError::ControllerCommandSpaceMismatch {
                    controller: name.clone(),
                    controller_space,
                    allocator_space,
                });
            }
        }
    }

    errors
}
