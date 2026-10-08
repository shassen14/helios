//! The built-in measurement kinds, each registered with the payload type it
//! reads.
//!
//! Every model resolves its sensor's geometry from the TF tree at tick time,
//! so each is built for the sensor frame its channel names. Physical constants
//! the model believes (gravity, the local magnetic field) come from its own
//! config.

use crate::assembly::{BuildContext, NoParams};
use crate::nodes::estimation::gravity::default_gravity_enu;
use crate::nodes::estimation::EstimatorComponents;

use helios_core::estimation::measurement::{
    accelerometer::SpecificForceModel, gps::GpsPositionModel, gyroscope::AngularRateModel,
    magnetometer::MagneticFieldModel, MeasurementModel,
};
use helios_core::interchange::measurement::sensor::{
    Acceleration, AngularRate, GpsPosition, MagneticField,
};
use helios_core::spatial::FrameId;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// The `kind` of a GNSS position model, reading `GpsPosition`.
pub(crate) const GPS_POSITION_KIND: &str = "gps_position";
/// The `kind` of a specific-force model, reading `Acceleration`.
pub(crate) const ACCELEROMETER_KIND: &str = "accelerometer";
/// The `kind` of an angular-rate model, reading `AngularRate`.
pub(crate) const GYROSCOPE_KIND: &str = "gyroscope";
/// The `kind` of a magnetic-field model, reading `MagneticField`.
pub(crate) const MAGNETOMETER_KIND: &str = "magnetometer";

pub(super) fn register(components: &mut EstimatorComponents) {
    const ONCE: &str = "built-in measurement kinds are registered once, by the default components";
    components
        .register_measurement::<GpsPosition, _, _>(GPS_POSITION_KIND, build_gps_position)
        .expect(ONCE);
    components
        .register_measurement::<Acceleration, _, _>(ACCELEROMETER_KIND, build_accelerometer)
        .expect(ONCE);
    components
        .register_measurement::<AngularRate, _, _>(GYROSCOPE_KIND, build_gyroscope)
        .expect(ONCE);
    components
        .register_measurement::<MagneticField, _, _>(MAGNETOMETER_KIND, build_magnetometer)
        .expect(ONCE);
}

/// The `accelerometer` kind's config.
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct AccelerometerConfig {
    /// The filter's believed gravity, world ENU `[east, north, up]` (m/s²).
    /// Must match the simulated world's gravity unless the mismatch is the
    /// experiment.
    #[serde(default = "default_gravity_enu")]
    gravity_enu: [f64; 3],
}

/// The `magnetometer` kind's config.
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct MagnetometerConfig {
    /// The filter's believed local field, world ENU (µT). Required: there is
    /// no sensible default. Must match the simulated world's field unless the
    /// mismatch is the experiment.
    magnetic_field_enu: [f64; 3],
}

fn build_gps_position(
    _config: NoParams,
    ctx: &BuildContext<'_>,
    sensor: &FrameId,
) -> Result<Box<dyn MeasurementModel>, String> {
    Ok(Box::new(GpsPositionModel {
        agent: ctx.agent().clone(),
        sensor: sensor.clone(),
    }))
}

fn build_accelerometer(
    config: AccelerometerConfig,
    ctx: &BuildContext<'_>,
    sensor: &FrameId,
) -> Result<Box<dyn MeasurementModel>, String> {
    Ok(Box::new(SpecificForceModel {
        agent: ctx.agent().clone(),
        sensor: sensor.clone(),
        gravity_world: Vector3::from(config.gravity_enu),
    }))
}

fn build_gyroscope(
    _config: NoParams,
    ctx: &BuildContext<'_>,
    sensor: &FrameId,
) -> Result<Box<dyn MeasurementModel>, String> {
    Ok(Box::new(AngularRateModel {
        agent: ctx.agent().clone(),
        sensor: sensor.clone(),
    }))
}

fn build_magnetometer(
    config: MagnetometerConfig,
    ctx: &BuildContext<'_>,
    sensor: &FrameId,
) -> Result<Box<dyn MeasurementModel>, String> {
    Ok(Box::new(MagneticFieldModel {
        agent: ctx.agent().clone(),
        sensor: sensor.clone(),
        world_magnetic_field: Vector3::from(config.magnetic_field_enu),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::{AutonomyRegistry, ComponentError};
    use crate::nodes::estimation::measurement::{MeasurementSource, MeasurementWiring};
    use crate::port::{ChannelKey, SensorChannel};

    use helios_core::interchange::measurement::envelope::SensorReading;
    use helios_core::interchange::measurement::sensor::SensorPayload;
    use helios_core::prelude::AgentId;

    use nalgebra::DMatrix;
    use std::collections::HashSet;

    const NODE: &str = "primary";
    const INPUT: &str = "sensor";

    fn build(section: &str) -> Result<Box<dyn MeasurementSource>, ComponentError> {
        let registry = AutonomyRegistry::default();
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("car"), NODE, &channels);
        let wiring = MeasurementWiring {
            input: INPUT.to_string(),
            noise: DMatrix::identity(3, 3),
        };
        registry
            .extension::<EstimatorComponents>()
            .expect("the default registry has estimator components")
            .build_measurement("aiding.sensor.model", section, &ctx, wiring)
            .map(|built| built.component)
    }

    fn reads<P: SensorPayload>(source: &dyn MeasurementSource) -> bool {
        let key: ChannelKey = SensorChannel::named::<Vec<SensorReading<P>>>(INPUT).into();
        source.channel() == &key
    }

    /// Each built-in kind reads the payload it was registered with: the pairing
    /// the old `sensor_payload` key left to config.
    #[test]
    fn each_built_in_kind_reads_its_own_payload() {
        let gps = build(&format!("kind = \"{GPS_POSITION_KIND}\"")).expect("gps builds");
        let accel = build(&format!("kind = \"{ACCELEROMETER_KIND}\"")).expect("accel builds");
        let gyro = build(&format!("kind = \"{GYROSCOPE_KIND}\"")).expect("gyro builds");
        let mag = build(&format!(
            "kind = \"{MAGNETOMETER_KIND}\"\nmagnetic_field_enu = [0.0, 22.0, -42.0]"
        ))
        .expect("mag builds");

        assert!(reads::<GpsPosition>(&*gps));
        assert!(reads::<Acceleration>(&*accel));
        assert!(reads::<AngularRate>(&*gyro));
        assert!(reads::<MagneticField>(&*mag));
    }

    /// The magnetometer has no default field; leaving it out is a config error
    /// at its path, not a model believing in zero field.
    #[test]
    fn the_magnetometer_requires_its_field() {
        let Err(err) = build(&format!("kind = \"{MAGNETOMETER_KIND}\"")) else {
            panic!("a magnetometer without a field must not build");
        };
        assert_eq!(
            err.to_string(),
            format!(
                "nodes.{NODE}.aiding.sensor.model (kind '{MAGNETOMETER_KIND}'): \
                 missing field `magnetic_field_enu`"
            )
        );
    }

    /// A model taking no parameters rejects one, rather than ignore a key the
    /// author thought did something (gravity on a GPS model, say).
    #[test]
    fn a_parameterless_model_rejects_a_parameter() {
        let Err(err) = build(&format!(
            "kind = \"{GPS_POSITION_KIND}\"\ngravity_enu = [0.0, 0.0, -9.81]"
        )) else {
            panic!("a stray key must not build");
        };
        assert!(matches!(err, ComponentError::InvalidConfig { .. }), "{err}");
    }
}
