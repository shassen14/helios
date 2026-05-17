//! Registers built-in measurement model factories.
//!
//! Models are registered by string key (e.g. `"gps_position"`) and built by
//! the assembler when constructing aiding handlers for `GaussianEstimatorNode`.
//! All four models resolve their sensor geometry from the TF tree at tick time,
//! so all require `agent_handle` and `sensor_handle` in the build context.
//! Physical constants (gravity, magnetic field) are read from
//! `ctx.model_config` so they originate from config, not external callers.

use helios_core::estimation::measurement::{
    accelerometer::AccelerometerModel, gps::GpsPositionModel, gyroscope::GyroscopeModel,
    magnetometer::MagnetometerModel, MeasurementModel,
};
use nalgebra::Vector3;

use super::{contexts::MeasurementModelBuildContext, AutonomyRegistry};

pub fn register(registry: &mut AutonomyRegistry) {
    registry.register_measurement_model("gps_position", build_gps_position);
    registry.register_measurement_model("accelerometer", build_accelerometer);
    registry.register_measurement_model("gyroscope", build_gyroscope);
    registry.register_measurement_model("magnetometer", build_magnetometer);
}

fn build_gps_position(
    ctx: MeasurementModelBuildContext,
) -> Result<Box<dyn MeasurementModel>, String> {
    Ok(Box::new(GpsPositionModel {
        agent_handle: ctx.agent_handle,
        sensor_handle: ctx.sensor_handle,
    }))
}

fn build_accelerometer(
    ctx: MeasurementModelBuildContext,
) -> Result<Box<dyn MeasurementModel>, String> {
    Ok(Box::new(AccelerometerModel {
        agent_handle: ctx.agent_handle,
        sensor_handle: ctx.sensor_handle,
        gravity_magnitude: ctx.model_config.gravity,
    }))
}

fn build_gyroscope(
    ctx: MeasurementModelBuildContext,
) -> Result<Box<dyn MeasurementModel>, String> {
    Ok(Box::new(GyroscopeModel {
        agent_handle: ctx.agent_handle,
        sensor_handle: ctx.sensor_handle,
    }))
}

fn build_magnetometer(
    ctx: MeasurementModelBuildContext,
) -> Result<Box<dyn MeasurementModel>, String> {
    let field_arr = ctx.model_config.magnetic_field_enu.ok_or(
        "magnetometer model requires `magnetic_field_enu` in its SensorModelConfig",
    )?;
    let field = Vector3::new(field_arr[0], field_arr[1], field_arr[2]);
    Ok(Box::new(MagnetometerModel {
        agent_handle: ctx.agent_handle,
        sensor_handle: ctx.sensor_handle,
        world_magnetic_field: field,
    }))
}
