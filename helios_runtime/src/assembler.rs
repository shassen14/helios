//! Config-driven pipeline assembler.
//!
//! [`build_pipeline`] is the single entry point: given a fully-resolved
//! [`AutonomyStack`] and an [`AutonomyRegistry`], it constructs every
//! [`PipelineNode`], declares sensor signal channels, validates the graph, and
//! returns a ready-to-tick [`AutonomyPipeline`].
//!
//! ## What the host provides
//!
//! Two things cannot come from config — they are host-specific runtime tokens:
//!
//! - `agent_handle` — the agent's [`FrameHandle`], assigned by the host's
//!   entity system (Bevy `Entity` bits in sim, static calibration ID on hw).
//! - `sensor_frame_handles` — maps each aiding `input_channel` string to the
//!   [`FrameHandle`] of the physical sensor that publishes on it. Used to build
//!   the `MeasurementModelBuildContext` for each aiding handler.
//!
//! Everything else — algorithm kinds, noise params, physical constants,
//! channel names — comes from `stack`.
//!
//! ## Sensor payload dispatch
//!
//! Aiding handler construction requires a concrete `T: SensorPayload` at
//! compile time. The assembler matches the `sensor_payload` string from
//! [`AidingConfig`] to one of the known implementors via an inline `match`.
//! This list must stay in sync with `KNOWN_SENSOR_PAYLOADS` in `validation.rs`
//! and with the `SensorPayload` impls in `helios_core::data::sensor`.
//!
//! If third-party sensor payload types become a real requirement, this can be
//! promoted to a registry family (`register_aiding_handler_factory`). For the
//! current set of five built-in types, the inline match is sufficient.

use std::collections::HashMap;

use helios_core::data::primitives::FrameHandle;
use helios_core::data::sensor::{
    AngularVelocity3D, GpsPosition, GpsVelocity, LinearAcceleration3D, MagneticField3D,
    SensorReading,
};
use helios_core::mapping::MapData;
use helios_core::planning::types::Path;
use nalgebra::DMatrix;

use crate::config::AutonomyStack;
use crate::config::{AidingConfig, EkfDynamicsConfig, EstimatorConfig, MapLayerConfig};
use crate::pipeline::autonomy_pipeline::PipelineBuilder;
use crate::pipeline::build_error::PipelineBuildError;
use crate::pipeline::nodes::gaussian_estimator::{AidingHandler, TypedAidingHandler};
use crate::pipeline::AutonomyPipeline;
use crate::port::ChannelKey;
use crate::registry::contexts::{
    ControllerBuildContext, GaussianEstimatorBuildContext, MapperBuildContext,
    MeasurementModelBuildContext, PathFollowerBuildContext, SearchPlannerBuildContext,
};
use crate::registry::AutonomyRegistry;

/// Errors that can occur while assembling a pipeline from config.
#[derive(Debug)]
pub enum PipelineAssemblyError {
    /// The config references an algorithm or model not in the registry.
    FactoryFailure { node_kind: String, reason: String },
    /// The assembled node graph failed topological validation.
    PipelineBuild(Vec<PipelineBuildError>),
    /// An aiding entry names a sensor channel with no corresponding
    /// [`FrameHandle`] in `sensor_frame_handles`.
    UnknownSensorChannel {
        estimator_instance: String,
        input_channel: String,
    },
    /// An aiding entry names a `sensor_payload` the assembler does not
    /// recognize. (Should be caught first by `validate_autonomy_config`.)
    UnknownSensorPayload {
        estimator_instance: String,
        payload_kind: String,
    },
    /// `path_following` is present but no planner was configured to produce a
    /// path, and no explicit `path_source` was given.
    NoPathSourceForFollower,
    /// `path_following` names a `path_source` planner key that does not exist
    /// in `search_planners`.
    UnknownPathSource { path_source: String },
}

impl std::fmt::Display for PipelineAssemblyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PipelineAssemblyError::FactoryFailure { node_kind, reason } => {
                write!(f, "factory '{node_kind}' failed: {reason}")
            }
            PipelineAssemblyError::PipelineBuild(errs) => {
                write!(f, "pipeline graph errors: ")?;
                for (i, e) in errs.iter().enumerate() {
                    if i > 0 {
                        write!(f, "; ")?;
                    }
                    write!(f, "{e}")?;
                }
                Ok(())
            }
            PipelineAssemblyError::UnknownSensorChannel {
                estimator_instance,
                input_channel,
            } => {
                write!(
                    f,
                    "estimator '{estimator_instance}' aiding channel '{input_channel}' has no FrameHandle in sensor_frame_handles"
                )
            }
            PipelineAssemblyError::UnknownSensorPayload {
                estimator_instance,
                payload_kind,
            } => {
                write!(
                    f,
                    "estimator '{estimator_instance}' aiding entry has unknown sensor_payload '{payload_kind}'"
                )
            }
            PipelineAssemblyError::NoPathSourceForFollower => {
                write!(f, "path_following configured but no planner produces a path and no path_source specified")
            }
            PipelineAssemblyError::UnknownPathSource { path_source } => {
                write!(f, "path_following.path_source '{path_source}' does not match any key in search_planners")
            }
        }
    }
}

/// Builds a fully-validated [`AutonomyPipeline`] from a resolved [`AutonomyStack`].
///
/// Call [`crate::validation::validate_autonomy_config`] before this to catch
/// config errors with better messages. `build_pipeline` is not a substitute
/// for validation — it fails fast on the first factory error it hits.
///
/// # Parameters
///
/// - `stack` — fully-resolved autonomy config (no unresolved `from` refs).
/// - `registry` — factory registry, typically `AutonomyRegistry::default()`.
/// - `agent_handle` — host-assigned identity token for this agent.
/// - `sensor_frame_handles` — maps each aiding `input_channel` to the
///   [`FrameHandle`] of the physical sensor publishing on that channel.
///   Channels not used by any aiding entry may be absent.
pub fn build_pipeline(
    stack: &AutonomyStack,
    registry: &AutonomyRegistry,
    agent_handle: FrameHandle,
    sensor_frame_handles: &HashMap<String, FrameHandle>,
) -> Result<AutonomyPipeline, Vec<PipelineAssemblyError>> {
    let mut errors: Vec<PipelineAssemblyError> = vec![];
    let mut builder = PipelineBuilder::new();
    // Channels supplied from outside the graph (sensor publishers, mission
    // layer, operator UI). Used to seed the topological sort so consumers
    // don't trip UnsatisfiedInput.
    let mut external_channels: Vec<ChannelKey> = vec![];

    // --- Estimators ---
    for (instance_name, est_cfg) in &stack.estimators {
        match build_estimator_node(
            instance_name,
            est_cfg,
            agent_handle,
            sensor_frame_handles,
            registry,
            &mut external_channels,
        ) {
            Ok(node) => {
                builder = builder.add_node(node);
            }
            Err(e) => errors.push(e),
        }
    }

    // --- Map layers ---
    for (_, map_cfg) in &stack.map_layers {
        if matches!(map_cfg, MapLayerConfig::None) {
            continue;
        }

        match registry.build_mapper(
            map_cfg.get_kind_str(),
            MapperBuildContext {
                agent_handle,
                config: map_cfg.clone(),
            },
        ) {
            Ok(node) => {
                // FrameAwareState is produced by the estimator upstream;
                // every other required input of a mapper is an external
                // sensor channel that must seed the topological sort.
                for key in &node.port_descriptor().required_inputs {
                    if *key != ChannelKey::of::<helios_core::frames::FrameAwareState>() {
                        external_channels.push(key.clone());
                    }
                }
                builder = builder.add_node(node);
            }
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: map_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    // --- Planners ---
    for (planner_name, plan_cfg) in &stack.search_planners {
        let level = plan_cfg.get_level_str();
        let map_channel = ChannelKey::named::<MapData>(level);
        let path_channel = ChannelKey::named::<Path>(planner_name.as_str());

        match registry.build_search_planner(
            plan_cfg.get_kind_str(),
            SearchPlannerBuildContext {
                agent_handle,
                config: plan_cfg.clone(),
                map_channel,
                path_channel,
            },
        ) {
            Ok(node) => {
                builder = builder.add_node(node);
            }
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: plan_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    // --- Path follower ---
    if let Some(pf_cfg) = &stack.path_following {
        match resolve_path_channel(stack) {
            Ok(path_channel) => {
                match registry.build_path_follower(
                    pf_cfg.get_kind_str(),
                    PathFollowerBuildContext {
                        agent_handle,
                        config: pf_cfg.clone(),
                        path_channel,
                    },
                ) {
                    Ok(node) => {
                        builder = builder.add_node(node);
                    }
                    Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                        node_kind: pf_cfg.get_kind_str().to_string(),
                        reason,
                    }),
                }
            }
            Err(e) => errors.push(e),
        }
    }

    // --- Controllers ---
    for (_, ctrl_cfg) in &stack.controllers {
        match registry.build_controller(
            ctrl_cfg.get_kind_str(),
            ControllerBuildContext {
                agent_handle,
                config: ctrl_cfg.clone(),
            },
        ) {
            Ok(node) => {
                builder = builder.add_node(node);
            }
            Err(reason) => errors.push(PipelineAssemblyError::FactoryFailure {
                node_kind: ctrl_cfg.get_kind_str().to_string(),
                reason,
            }),
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    // Deduplicate external channels before handing to the builder.
    external_channels.sort_by_key(|k| format!("{k:?}"));
    external_channels.dedup();

    builder
        .with_external_channels(external_channels)
        .build()
        .map_err(|build_errors| vec![PipelineAssemblyError::PipelineBuild(build_errors)])
}

// --- Internals ---

fn build_estimator_node(
    instance_name: &str,
    est_cfg: &EstimatorConfig,
    agent_handle: FrameHandle,
    sensor_frame_handles: &HashMap<String, FrameHandle>,
    registry: &AutonomyRegistry,
    external_channels: &mut Vec<ChannelKey>,
) -> Result<Box<dyn crate::pipeline::node::PipelineNode>, PipelineAssemblyError> {
    let EstimatorConfig::Ekf(ekf_cfg) = est_cfg else {
        return Err(PipelineAssemblyError::FactoryFailure {
            node_kind: est_cfg.get_kind_str().to_string(),
            reason: "only EKF is currently implemented".to_string(),
        });
    };

    // Build aiding handlers from the aiding list in EkfConfig.
    let mut aiding: Vec<Box<dyn AidingHandler>> = vec![];
    for aid in &ekf_cfg.aiding {
        let handler = build_aiding_handler(
            instance_name,
            aid,
            agent_handle,
            sensor_frame_handles,
            registry,
        )?;
        external_channels.push(handler.channel().clone());
        aiding.push(handler);
    }

    // If dynamics is IntegratedImu, declare the IMU predict-side channels
    // as external too (accel + gyro Vec<SensorReading<_>>).
    if matches!(ekf_cfg.dynamics, EkfDynamicsConfig::IntegratedImu(_)) {
        use crate::pipeline::builders::estimator::{
            EstimatorInputBuilder, IntegratedImuInputBuilder,
        };
        let builder = IntegratedImuInputBuilder::new();
        external_channels.extend_from_slice(builder.required_channels());
    }

    registry
        .build_gaussian_estimator(
            est_cfg.get_kind_str(),
            est_cfg.clone(),
            GaussianEstimatorBuildContext {
                agent_handle,
                aiding,
            },
        )
        .map_err(|reason| PipelineAssemblyError::FactoryFailure {
            node_kind: est_cfg.get_kind_str().to_string(),
            reason,
        })
}

fn build_aiding_handler(
    instance_name: &str,
    aid: &AidingConfig,
    agent_handle: FrameHandle,
    sensor_frame_handles: &HashMap<String, FrameHandle>,
    registry: &AutonomyRegistry,
) -> Result<Box<dyn AidingHandler>, PipelineAssemblyError> {
    let sensor_handle = sensor_frame_handles
        .get(&aid.input_channel)
        .copied()
        .ok_or_else(|| PipelineAssemblyError::UnknownSensorChannel {
            estimator_instance: instance_name.to_string(),
            input_channel: aid.input_channel.clone(),
        })?;

    let model = registry
        .build_measurement_model(
            &aid.model.kind,
            MeasurementModelBuildContext {
                agent_handle,
                sensor_handle,
                model_config: aid.model.clone(),
            },
        )
        .map_err(|reason| PipelineAssemblyError::FactoryFailure {
            node_kind: aid.model.kind.clone(),
            reason,
        })?;

    let r = DMatrix::from_diagonal(&nalgebra::DVector::from_vec(aid.r_diag.clone()));

    // Dispatch on sensor_payload to construct the correctly-typed handler.
    // This list mirrors KNOWN_SENSOR_PAYLOADS in validation.rs and the
    // SensorPayload impls in helios_core::data::sensor.
    //
    // If third-party SensorPayload types become a real need, promote this to a
    // registry family: `register_aiding_handler_factory("MyType", factory)`.
    let channel = build_aiding_channel(aid)?;

    let handler: Box<dyn AidingHandler> = match aid.sensor_payload.as_str() {
        "GpsPosition" => Box::new(TypedAidingHandler::<GpsPosition>::new(channel, model, r)),
        "GpsVelocity" => Box::new(TypedAidingHandler::<GpsVelocity>::new(channel, model, r)),
        "LinearAcceleration3D" => Box::new(TypedAidingHandler::<LinearAcceleration3D>::new(
            channel, model, r,
        )),
        "AngularVelocity3D" => Box::new(TypedAidingHandler::<AngularVelocity3D>::new(
            channel, model, r,
        )),
        "MagneticField3D" => Box::new(TypedAidingHandler::<MagneticField3D>::new(
            channel, model, r,
        )),
        other => {
            return Err(PipelineAssemblyError::UnknownSensorPayload {
                estimator_instance: instance_name.to_string(),
                payload_kind: other.to_string(),
            });
        }
    };

    Ok(handler)
}

/// Constructs the bus [`ChannelKey`] for a sensor reading channel, typed by
/// `sensor_payload`. The channel key encodes both the Rust type and the
/// instance qualifier from `input_channel`.
fn build_aiding_channel(aid: &AidingConfig) -> Result<ChannelKey, PipelineAssemblyError> {
    let q = aid.input_channel.as_str();
    let key = match aid.sensor_payload.as_str() {
        "GpsPosition" => ChannelKey::named::<Vec<SensorReading<GpsPosition>>>(q),
        "GpsVelocity" => ChannelKey::named::<Vec<SensorReading<GpsVelocity>>>(q),
        "LinearAcceleration3D" => ChannelKey::named::<Vec<SensorReading<LinearAcceleration3D>>>(q),
        "AngularVelocity3D" => ChannelKey::named::<Vec<SensorReading<AngularVelocity3D>>>(q),
        "MagneticField3D" => ChannelKey::named::<Vec<SensorReading<MagneticField3D>>>(q),
        _ => unreachable!("caller already validated sensor_payload"),
    };
    Ok(key)
}

/// Determines the path channel the `PathFollowerNode` should read from.
///
/// Uses `PathFollowingConfig::path_source` if set; otherwise, falls back to
/// the single planner's key. Errors if multiple planners exist and no
/// explicit source is given.
fn resolve_path_channel(stack: &AutonomyStack) -> Result<ChannelKey, PipelineAssemblyError> {
    // Use an explicit path_source if configured.
    // TODO: add `path_source: Option<String>` to PathFollowingConfig to
    // support multi-planner stacks without ambiguity.
    // For now: auto-select the single planner, error if there are multiple.
    match stack.search_planners.len() {
        0 => Err(PipelineAssemblyError::NoPathSourceForFollower),
        1 => {
            let planner_name = stack.search_planners.keys().next().unwrap();
            Ok(ChannelKey::named::<Path>(planner_name.as_str()))
        }
        _ => {
            // Multiple planners: cannot auto-select. Caller should add
            // `path_source` to PathFollowingConfig (tracked as follow-up).
            Err(PipelineAssemblyError::NoPathSourceForFollower)
        }
    }
}
