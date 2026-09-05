//! Config-to-node assembly for the Gaussian-estimator family: builds the
//! aiding handlers and predict-side input channels from an [`EkfConfig`], then
//! hands them to the registry factory.

use super::{AidingHandler, EstimatorInputBuilder, IntegratedImuInputBuilder, TypedAidingHandler};
use crate::config::{AidingConfig, EkfConfig, EkfDynamicsConfig, EstimatorConfig};
use crate::pipeline::node::PipelineNode;
use crate::port::{ChannelKey, SensorChannel};
use crate::registry::contexts::{GaussianEstimatorBuildContext, MeasurementModelBuildContext};
use crate::registry::AutonomyRegistry;
use crate::PipelineAssemblyError;

use helios_core::data::envelope::SensorReading;
use helios_core::data::primitives::FrameHandle;
use helios_core::data::sensor::{
    AngularVelocity3D, GpsPosition, GpsVelocity, LinearAcceleration3D, MagneticField3D,
};
use helios_core::estimation::augmentation::augmentation_block;
use helios_core::estimation::schema::{MeasurementAgreementError, StateSchemaBlock};
use helios_core::frames::FrameId;

use nalgebra::DMatrix;
use std::collections::HashMap;

/// Assembles a Gaussian-estimator node from its config: constructs the aiding
/// handlers, declares the predict-side input channels as external, and invokes
/// the registry factory. `external_channels` accumulates every host-published
/// channel the estimator reads so the topological sort can be seeded.
pub(crate) fn assemble(
    instance_name: &str,
    est_cfg: &EstimatorConfig,
    ekf_cfg: &EkfConfig,
    agent_handle: FrameHandle,
    sensor_frame_handles: &HashMap<String, FrameHandle>,
    registry: &AutonomyRegistry,
    external_channels: &mut Vec<ChannelKey>,
) -> Result<Box<dyn PipelineNode>, PipelineAssemblyError> {
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
    if let EkfDynamicsConfig::IntegratedImu(imu_cfg) = &ekf_cfg.dynamics {
        let builder = IntegratedImuInputBuilder::new(
            imu_cfg.accel_channel.as_str(),
            imu_cfg.gyro_channel.as_str(),
        );
        external_channels.extend_from_slice(builder.required_channels());
    }

    // Turn each declared augmentation into a schema block, tied to the same
    // sensor FrameHandle its aiding source resolves to. Reusing the aiding
    // channel resolution (below) is what guarantees the appended bias slots
    // carry the FrameId the measurement model reads back — a mismatch would
    // leave the block inert, never observed. The `sensor`-has-an-aiding-source
    // requirement itself is enforced earlier by `validate_autonomy_config`.
    let augmentation_blocks =
        build_augmentation_blocks(instance_name, ekf_cfg, sensor_frame_handles)?;

    registry
        .build_gaussian_estimator(
            est_cfg.get_kind_str(),
            est_cfg.clone(),
            GaussianEstimatorBuildContext {
                agent_handle,
                instance_name: instance_name.to_string(),
                aiding,
                augmentation_blocks,
            },
        )
        .map_err(|reason| PipelineAssemblyError::FactoryFailure {
            node_kind: est_cfg.get_kind_str().to_string(),
            reason,
        })
}

/// Resolves every `[[augmentation]]` entry into a [`StateSchemaBlock`], each tagged
/// with the `FrameId::Sensor` its `sensor` channel resolves to.
///
/// The sensor string is looked up in the same `sensor_frame_handles` map the
/// aiding handlers use, so the block and the aiding sensor share one handle. An
/// unresolvable `sensor` is an [`UnknownSensorChannel`], and a bad `kind` or
/// noise is an [`AugmentationFailure`] — both build-time faults, never a panic.
///
/// [`UnknownSensorChannel`]: PipelineAssemblyError::UnknownSensorChannel
/// [`AugmentationFailure`]: PipelineAssemblyError::AugmentationFailure
fn build_augmentation_blocks(
    instance_name: &str,
    ekf_cfg: &EkfConfig,
    sensor_frame_handles: &HashMap<String, FrameHandle>,
) -> Result<Vec<StateSchemaBlock>, PipelineAssemblyError> {
    let mut blocks = Vec::with_capacity(ekf_cfg.augmentation.len());
    for aug in &ekf_cfg.augmentation {
        let sensor_handle = sensor_frame_handles
            .get(&aug.sensor)
            .copied()
            .ok_or_else(|| PipelineAssemblyError::UnknownSensorChannel {
                estimator_instance: instance_name.to_string(),
                input_channel: aug.sensor.clone(),
            })?;

        let block = augmentation_block(
            &aug.kind,
            FrameId::Sensor(sensor_handle),
            aug.init_uncertainty,
            aug.random_walk,
        )
        .map_err(|reason| PipelineAssemblyError::AugmentationFailure {
            estimator_instance: instance_name.to_string(),
            reason: reason.to_string(),
        })?;

        blocks.push(block);
    }
    Ok(blocks)
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

    // The measurement's innovation length (its schema dim) and the noise matrix
    // R must match; otherwise the filter silently skips every update from this
    // sensor. Catch a mis-sized R here, at build time, rather than as a runtime
    // no-op that looks like a dead sensor.
    let schema_dim = model.schema().dim();
    if schema_dim != r.nrows() {
        return Err(PipelineAssemblyError::FactoryFailure {
            node_kind: aid.model.kind.clone(),
            reason: MeasurementAgreementError::DimensionMismatch {
                schema_dim,
                expected: r.nrows(),
            }
            .to_string(),
        });
    }

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

/// Constructs the bus [`SensorChannel`] for a sensor reading channel, typed
/// by `sensor_payload`. The channel encodes both the Rust type and the
/// instance qualifier from `input_channel`.
fn build_aiding_channel(aid: &AidingConfig) -> Result<SensorChannel, PipelineAssemblyError> {
    let q = aid.input_channel.as_str();
    let key = match aid.sensor_payload.as_str() {
        "GpsPosition" => SensorChannel::named::<Vec<SensorReading<GpsPosition>>>(q),
        "GpsVelocity" => SensorChannel::named::<Vec<SensorReading<GpsVelocity>>>(q),
        "LinearAcceleration3D" => {
            SensorChannel::named::<Vec<SensorReading<LinearAcceleration3D>>>(q)
        }
        "AngularVelocity3D" => SensorChannel::named::<Vec<SensorReading<AngularVelocity3D>>>(q),
        "MagneticField3D" => SensorChannel::named::<Vec<SensorReading<MagneticField3D>>>(q),
        _ => unreachable!("caller already validated sensor_payload"),
    };
    Ok(key)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::config::SensorModelConfig;

    // A GPS-position aiding config (a 3-DOF measurement) whose R diagonal has the
    // given length, reading from a sensor channel named "gps".
    fn gps_aiding(r_len: usize) -> AidingConfig {
        AidingConfig {
            sensor_payload: "GpsPosition".to_string(),
            model: SensorModelConfig {
                kind: "gps_position".to_string(),
                gravity_enu: [0.0, 0.0, -9.81],
                magnetic_field_enu: None,
            },
            input_channel: "gps".to_string(),
            r_diag: vec![1.0; r_len],
        }
    }

    fn sensor_handles() -> HashMap<String, FrameHandle> {
        HashMap::from([("gps".to_string(), FrameHandle(1))])
    }

    // R sized to the model's measurement dimension builds a handler cleanly.
    #[test]
    fn aiding_handler_builds_when_r_matches_model_dim() {
        let registry = AutonomyRegistry::default();
        let handler = build_aiding_handler(
            "est",
            &gps_aiding(3),
            FrameHandle(0),
            &sensor_handles(),
            &registry,
        );
        assert!(handler.is_ok());
    }

    // A mis-sized R would make every update from this sensor a silent runtime
    // no-op; the build must reject it instead of producing a dead handler.
    #[test]
    fn aiding_handler_rejects_r_of_wrong_dim() {
        let registry = AutonomyRegistry::default();
        let result = build_aiding_handler(
            "est",
            &gps_aiding(2),
            FrameHandle(0),
            &sensor_handles(),
            &registry,
        );
        assert!(matches!(
            result,
            Err(PipelineAssemblyError::FactoryFailure { .. })
        ));
    }
}
