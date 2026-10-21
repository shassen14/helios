//! The two halves of a sensor's config: the **device** (`entities/sensors/`),
//! which holds what is true of every unit of a model — kind, rate, noise,
//! bias, beam pattern, and which channel fields it publishes on — and the
//! **installation** (`entities/sensor_fits/`), which places one unit on one
//! vehicle: the device it is, its mount, and the channel name for each of the
//! device's channel fields.

use super::pose::Pose;

use helios_core::prelude::AgentId;
use helios_core::sensors::{
    accelerometer::AccelerometerModel, gps::GpsModel, lidar::LidarModel,
    magnetometer::MagnetometerModel,
};
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

use figment::value::Value;
use serde::Deserialize;
use std::collections::BTreeMap;

/// The channel field of every device kind that publishes one quantity.
pub const CHANNEL_FIELD: &str = "channel";
/// The IMU's channel field for `Vec<SensorReading<Acceleration>>`.
pub const ACCEL_CHANNEL_FIELD: &str = "accel_channel";
/// The IMU's channel field for `Vec<SensorReading<AngularRate>>`.
pub const GYRO_CHANNEL_FIELD: &str = "gyro_channel";

/// One physical sensor unit on a vehicle: a device, where it is mounted, and
/// the bus channel for each channel field the device declares.
///
/// In TOML the channel fields sit flat beside `device` and `transform`
/// (`channel`, or `accel_channel` + `gyro_channel` for an IMU). They are
/// checked once, at load, against [`SensorDeviceConfig::channel_fields`]: one
/// rule for every kind, so a new kind only declares its fields.
#[derive(Debug, Clone, Deserialize)]
#[serde(try_from = "SensorInstallationToml")]
pub struct SensorInstallationConfig {
    pub device: SensorDeviceConfig,
    /// The sensor frame relative to `base_link` (FLU). Required: the mount is
    /// the fact an installation declares, so it never defaults to the origin.
    pub transform: Pose,
    /// Bus channel name per declared channel field. Holds exactly the
    /// device's fields, so a lookup of one of them always succeeds.
    channels: BTreeMap<&'static str, String>,
}

impl SensorInstallationConfig {
    /// The bus channel this unit publishes on for `field`, one of its
    /// device's [`channel_fields`](SensorDeviceConfig::channel_fields).
    /// `None` only for a field the device does not declare.
    pub fn channel(&self, field: &str) -> Option<&str> {
        self.channels.get(field).map(String::as_str)
    }

    /// The frame this unit contributes to `agent`'s body, where `key` is its
    /// table key in the sensor fit: one frame per installation, named by the
    /// key, whatever channels the unit publishes on. The sensor spawners stamp
    /// it on the sensor's frame entity and as each channel's mount, and the
    /// estimated tf buffer is seeded from it, so the two cannot fork a mount.
    pub fn frame(&self, agent: &AgentId, key: &str) -> InstallationFrame {
        InstallationFrame {
            id: FrameId::agent(agent.clone(), key),
            pose: self.transform,
            convention: self.device.convention(),
        }
    }
}

impl TryFrom<SensorInstallationToml> for SensorInstallationConfig {
    type Error = String;

    /// Checks the channel fields against the device's declared ones: every
    /// key given must be declared, every declared field must be given, and
    /// each must be a string. Undeclared keys are reported first and by name,
    /// so a misspelled `transform` reads as a misspelling, not a type error.
    fn try_from(raw: SensorInstallationToml) -> Result<Self, Self::Error> {
        let SensorInstallationToml {
            device,
            transform,
            mut channel_fields,
        } = raw;
        let kind = device.get_kind_str();
        let declared = device.channel_fields();

        if let Some(stray) = channel_fields
            .keys()
            .find(|key| !declared.contains(&key.as_str()))
        {
            return Err(format!(
                "`{stray}` is not a field of a {kind} installation; its channel \
                 fields are {declared:?}"
            ));
        }

        let mut channels = BTreeMap::new();
        for &field in declared {
            let Some(value) = channel_fields.remove(field) else {
                return Err(format!(
                    "an installation of a {kind} device needs `{field}`; its channel \
                     fields are {declared:?}"
                ));
            };
            let Some(channel) = value.into_string() else {
                return Err(format!(
                    "`{field}` of a {kind} installation must be a channel name (a string)"
                ));
            };
            channels.insert(field, channel);
        }

        Ok(Self {
            device,
            transform,
            channels,
        })
    }
}

/// An installation as written in a sensor-fit file, before its channel
/// fields are checked against the device kind.
///
/// No `deny_unknown_fields`: it can't be combined with `flatten`, and every
/// key other than `device` and `transform` lands in `channel_fields`, where
/// the conversion refuses any the device does not declare. The values are
/// collected untyped so a stray key of any type is refused by name.
#[derive(Debug, Clone, Deserialize)]
struct SensorInstallationToml {
    device: SensorDeviceConfig,
    transform: Pose,
    #[serde(flatten)]
    channel_fields: BTreeMap<String, Value>,
}

/// One installation's frame on its body: its identity, its pose relative to
/// `base_link`, and the frame's axis convention.
///
/// Always a child of `base_link`: the spawners attach the frame's entity to
/// the agent's root, the IMU's lever arm is the pose's translation, and the
/// static seed names `base_link` as parent. A fit naming another parent needs
/// all three to follow it, or the truth and estimated trees fork.
#[derive(Debug, Clone)]
pub struct InstallationFrame {
    pub id: FrameId,
    pub pose: Pose,
    pub convention: Convention,
}

/// A sensor device: the properties every unit of one model shares. Where a
/// unit is mounted and the channel names it publishes on belong to its
/// installation; which channel fields it has is the device kind's.
/// The `tag = "kind"` tells Serde to look for a `kind = "..."` field in the TOML.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum SensorDeviceConfig {
    Imu(ImuConfig),
    Gps(GpsConfig),
    Lidar(LidarConfig),
    Magnetometer(MagnetometerConfig),
}

impl SensorDeviceConfig {
    pub fn get_kind_str(&self) -> &'static str {
        match self {
            SensorDeviceConfig::Imu(_) => "Imu",
            SensorDeviceConfig::Gps(_) => "Gps",
            SensorDeviceConfig::Lidar(_) => "Lidar",
            SensorDeviceConfig::Magnetometer(_) => "Magnetometer",
        }
    }

    /// The installation fields naming this kind's bus channels, one per
    /// quantity it publishes. An IMU publishes acceleration and angular rate
    /// on two channels; every other kind publishes one.
    pub fn channel_fields(&self) -> &'static [&'static str] {
        match self {
            SensorDeviceConfig::Imu(_) => &[ACCEL_CHANNEL_FIELD, GYRO_CHANNEL_FIELD],
            SensorDeviceConfig::Gps(_)
            | SensorDeviceConfig::Lidar(_)
            | SensorDeviceConfig::Magnetometer(_) => &[CHANNEL_FIELD],
        }
    }

    /// The axis convention of this kind's sensor frame, as its forward model
    /// declares it. An IMU's accelerometer and gyroscope share one frame,
    /// so they must declare one convention.
    pub fn convention(&self) -> Convention {
        match self {
            SensorDeviceConfig::Imu(_) => AccelerometerModel::FRAME_CONVENTION,
            SensorDeviceConfig::Gps(_) => GpsModel::FRAME_CONVENTION,
            SensorDeviceConfig::Lidar(_) => LidarModel::FRAME_CONVENTION,
            SensorDeviceConfig::Magnetometer(_) => MagnetometerModel::FRAME_CONVENTION,
        }
    }
}

/// Configuration parameters for a simulated IMU sensor.
///
/// An IMU chip always produces two physical quantities: linear acceleration
/// (`Acceleration`) and angular rate (`AngularRate`). If the
/// chip also has an onboard magnetometer, add a separate `Magnetometer` entry
/// to the sensor fit — that quantity is independently configured and spawns
/// its own sensor with its own forward model.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuConfig {
    pub rate: f64,
    /// Constant accelerometer offset along the sensor's [X, Y, Z] FLU axes,
    /// in m/s². Defaults to zero (a perfectly calibrated sensor).
    #[serde(default)]
    pub accel_bias: [f64; 3],
    /// Per-axis noise standard deviation for the accelerometer, in m/s².
    #[serde(default)]
    pub accel_noise_stddev: [f64; 3],
    /// Constant gyroscope offset along the sensor's [X, Y, Z] FLU axes, in
    /// rad/s. This is the drift rate a stationary gyro reports; it is the
    /// dominant error term in dead reckoning. Defaults to zero.
    #[serde(default)]
    pub gyro_bias: [f64; 3],
    /// Per-axis noise standard deviation for the gyroscope, in rad/s.
    #[serde(default)]
    pub gyro_noise_stddev: [f64; 3],
}

impl ImuConfig {
    pub fn get_rate(&self) -> f64 {
        self.rate
    }
    /// The accelerometer and gyroscope noise standard deviations, in that order.
    pub fn get_noise_stddevs(&self) -> ([f64; 3], [f64; 3]) {
        (self.accel_noise_stddev, self.gyro_noise_stddev)
    }
    /// The accelerometer and gyroscope biases, in that order.
    pub fn get_biases(&self) -> ([f64; 3], [f64; 3]) {
        (self.accel_bias, self.gyro_bias)
    }
}

/// Configuration parameters for a simulated GPS sensor.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GpsConfig {
    pub rate: f64,
    /// Constant measurement offset in [East, North, Up] axes, in meters.
    #[serde(default)]
    pub bias: [f64; 3],

    /// Standard deviation of noise in [East, North, Up] axes, in meters.
    #[serde(default)]
    pub noise_stddev: [f64; 3],
}

/// Configuration parameters for a simulated 3-axis magnetometer.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MagnetometerConfig {
    pub rate: f64,
    /// Constant measurement offset along the sensor's [X, Y, Z] FLU axes, in
    /// µT. This models hard-iron distortion — the field of ferrous material
    /// mounted near the sensor — which is fixed in the *sensor* frame and so
    /// rotates with the vehicle, unlike the world's reference field. Defaults
    /// to zero.
    #[serde(default)]
    pub bias: [f64; 3],
    /// Standard deviation of noise along the sensor's [X, Y, Z] FLU axes, in µT.
    #[serde(default)]
    pub noise_stddev: [f64; 3],
}

impl MagnetometerConfig {
    pub fn get_rate(&self) -> f64 {
        self.rate
    }
}

/// Configuration parameters for a simulated ray lidar of any beam layout.
///
/// The layout is not a variant — one struct spans the 2D planar scanner, the
/// spinning multi-ring unit, the forward-looking solid-state, and the flash
/// sensor. Which one a config describes is entirely the scan geometry: a single
/// entry in `ring_elevations` is a 2D lidar, several entries a 3D one; a `360`
/// `azimuth_fov` is a spinning unit, a narrow one a forward-looking scanner; a
/// zero `sweep_period` is a flash capture.
///
/// Angles are degrees here for readability and become radians when the forward
/// model is built. The geometry angles are f64 to feed the model's f64 scan
/// math without a lossy cast; the range/noise scalars are f32, the type
/// the raycasting model reports back to the physics engine's f32-native query.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LidarConfig {
    pub rate: f64,
    /// Minimum reported range, in meters. Nearer returns are misses — the
    /// unit's blind zone around its own window.
    pub range_min: f32,
    /// Maximum reported range, in meters.
    pub max_range: f32,
    /// Azimuth field of view in degrees. `360` sweeps a full circle and drops
    /// the duplicate seam beam; a smaller value is a forward-looking sector.
    pub azimuth_fov: f64,
    /// Number of beams across the azimuth field of view.
    pub azimuth_beams: u32,
    /// Elevation of each ring, in degrees. One entry is a planar (2D) lidar;
    /// several are the rings of a 3D unit. Datasheet tables are non-uniform, so
    /// this is an explicit list rather than a field-of-view span.
    pub ring_elevations: Vec<f64>,
    /// Seconds for one full azimuth sweep, stamped onto each point as a per-point
    /// time offset. Zero models a flash capture (every point shares the reading's
    /// instant). Defaults to a flash capture.
    #[serde(default)]
    pub sweep_period: f64,
    /// Range noise standard deviation, in meters. Must be strictly positive.
    pub range_noise_stddev: f32,
    /// Angular noise standard deviation, in degrees, applied to both azimuth and
    /// elevation. Must be strictly positive.
    pub angular_noise_stddev: f32,
}

impl LidarConfig {
    pub fn get_rate(&self) -> f64 {
        self.rate
    }
}

#[cfg(test)]
mod tests {
    //! `frame` is the single source of an installation's frame: the spawners
    //! stamp it on the sensor's frame entity and as every channel's mount, and the
    //! estimated tf buffer is seeded from it (`brain_bridge::spawn::
    //! build_static_seeds`). These pin that it is named by the installation,
    //! never by a channel, and carries the installation's pose.
    //!
    //! The channel-check tests parse TOML, since the check runs as part of
    //! deserializing an installation.

    use super::*;

    use helios_core::sensors::gyroscope::GyroscopeModel;

    use figment::providers::{Format, Toml};
    use figment::Figment;
    use nalgebra::{UnitQuaternion, Vector3};

    /// A distinctive, non-identity mount so the pose pass-through (not only the
    /// channel name) is actually observed.
    fn mount_pose() -> Pose {
        Pose {
            translation: Vector3::new(0.2, 0.0, 0.3),
            rotation: UnitQuaternion::identity(),
        }
    }

    fn installed(
        device: SensorDeviceConfig,
        channels: &[(&'static str, &str)],
    ) -> SensorInstallationConfig {
        SensorInstallationConfig {
            device,
            transform: mount_pose(),
            channels: channels
                .iter()
                .map(|(field, channel)| (*field, channel.to_string()))
                .collect(),
        }
    }

    fn parse(toml: &str) -> Result<SensorInstallationConfig, String> {
        Figment::new()
            .merge(Toml::string(toml))
            .extract()
            .map_err(|e| e.to_string())
    }

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    /// An IMU publishes on two channels but is one device in one place: its
    /// frame is named by its installation, not by either channel, so the
    /// accelerometer and gyroscope share one mount.
    #[test]
    fn an_imu_is_one_frame_named_by_its_installation() {
        let imu = installed(
            SensorDeviceConfig::Imu(ImuConfig {
                rate: 100.0,
                accel_bias: [0.0; 3],
                accel_noise_stddev: [0.0; 3],
                gyro_bias: [0.0; 3],
                gyro_noise_stddev: [0.0; 3],
            }),
            &[
                (ACCEL_CHANNEL_FIELD, "body.accel"),
                (GYRO_CHANNEL_FIELD, "body.gyro"),
            ],
        );

        let frame = imu.frame(&agent(), "imu");

        assert_eq!(frame.id, FrameId::agent(agent(), "imu"));
        assert_eq!(frame.convention, AccelerometerModel::FRAME_CONVENTION);
    }

    /// The IMU's one frame is named by the accelerometer's forward model, so
    /// the gyroscope's must name its axes alike.
    #[test]
    fn an_imus_two_forward_models_declare_one_frame_convention() {
        assert_eq!(
            AccelerometerModel::FRAME_CONVENTION,
            GyroscopeModel::FRAME_CONVENTION
        );
    }

    /// The frame carries the installation's pose.
    #[test]
    fn the_frame_is_placed_at_the_installation_pose() {
        let gps = installed(
            SensorDeviceConfig::Gps(GpsConfig {
                rate: 10.0,
                bias: [0.0; 3],
                noise_stddev: [0.0; 3],
            }),
            &[(CHANNEL_FIELD, "sensor.gps.primary")],
        );

        let frame = gps.frame(&agent(), "gps");

        assert_eq!(frame.id, FrameId::agent(agent(), "gps"));
        assert_eq!(frame.pose.translation, mount_pose().translation);
        assert_eq!(frame.pose.rotation, mount_pose().rotation);
    }

    /// An IMU publishes two quantities, so an installation naming only one of
    /// its channels would leave the other quantity with nowhere to go. Refused
    /// at load, naming the device kind and the fields it needs.
    #[test]
    fn an_imu_installation_missing_gyro_channel_is_refused() {
        let error = parse(
            r#"
            device = { kind = "Imu", rate = 100.0 }
            transform = { translation = [0.0, 0.0, 0.0] }
            accel_channel = "sensor.imu.accel"
            "#,
        )
        .expect_err("an IMU without gyro_channel should be refused");

        assert!(
            error.contains("Imu"),
            "error should name the kind, got: {error}"
        );
        assert!(
            error.contains("gyro_channel"),
            "error should name the missing field, got: {error}"
        );
    }

    /// A single-channel device given the IMU's channel field is a config
    /// mistake (most likely a copied IMU entry), not a field to ignore.
    #[test]
    fn a_gps_installation_writing_accel_channel_is_refused() {
        let error = parse(
            r#"
            device = { kind = "Gps", rate = 10.0 }
            transform = { translation = [0.0, 0.0, 0.0] }
            channel = "sensor.gps.primary"
            accel_channel = "sensor.imu.accel"
            "#,
        )
        .expect_err("a GPS with accel_channel should be refused");

        assert!(
            error.contains("Gps"),
            "error should name the kind, got: {error}"
        );
        assert!(
            error.contains("accel_channel"),
            "error should name the stray field, got: {error}"
        );
    }

    /// A misspelled key lands among the channel fields, since everything but
    /// `device` and `transform` does. It must still be refused by name — a
    /// table value (`transfrom = { … }`) once read as "expected a string",
    /// which hides the typo.
    #[test]
    fn a_stray_table_key_is_refused_by_name() {
        let error = parse(
            r#"
            device = { kind = "Gps", rate = 10.0 }
            transform = { translation = [0.0, 0.0, 0.0] }
            channel = "sensor.gps.primary"
            transfrom = { translation = [1.0, 0.0, 0.0] }
            "#,
        )
        .expect_err("an undeclared key should be refused");

        assert!(
            error.contains("transfrom"),
            "error should name the stray key, got: {error}"
        );
    }
}
