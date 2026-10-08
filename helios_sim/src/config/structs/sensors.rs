//! The two halves of a sensor's config: the **device** (`entities/sensors/`),
//! which holds what is true of every unit of a model — kind, rate, noise,
//! bias, beam pattern, and which channel fields it publishes on — and the
//! **installation** (`entities/sensor_fits/`), which places one unit on one
//! vehicle: the device it is, its mount, and the channel name for each of the
//! device's channel fields.

use super::pose::Pose;

use helios_core::spatial::transforms::Convention;

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

    /// The tf-tree frame mounts this unit contributes: for each bus channel it
    /// publishes, the `(channel, pose, convention)` of that sensor frame
    /// relative to `base_link`. The pose and convention are the same values the
    /// sensor spawners stamp onto the truth tf tree, so seeding the estimated
    /// buffer from here cannot fork a mount between the two trees. An IMU
    /// yields two mounts (accelerometer and gyroscope share one pose but
    /// publish on separate channels); every other sensor yields one.
    pub fn frame_mounts(&self) -> Vec<(String, Pose, Convention)> {
        self.channels
            .values()
            .map(|channel| (channel.clone(), self.transform, self.device.convention()))
            .collect()
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

    /// The axis convention of this kind's sensor frame. Every kind today
    /// reports in its body frame, FLU.
    pub fn convention(&self) -> Convention {
        match self {
            SensorDeviceConfig::Imu(_)
            | SensorDeviceConfig::Gps(_)
            | SensorDeviceConfig::Lidar(_)
            | SensorDeviceConfig::Magnetometer(_) => Convention::Flu,
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
    //! `frame_mounts` is the single source of the `(channel → base_link → sensor)`
    //! mounts that seed the estimated tf buffer (`brain_bridge::spawn::
    //! build_static_seeds`). The leaf of each mount's `FrameId::sensor(agent,
    //! channel)` is this channel string, which is *also* what the assembler wires
    //! as the estimator's aiding-input frame — so if `frame_mounts` surfaces the
    //! wrong channel, or drops one, the sensor's frame silently fails to resolve
    //! and the filter runs unaided. These assert the channel/convention contract
    //! that keeps those two sites from forking.
    //!
    //! The channel-check tests parse TOML, since the check runs as part of
    //! deserializing an installation.

    use super::*;

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

    /// An IMU is one installation but two frames: accelerometer and gyroscope
    /// share a pose yet publish on separate channels, so each must surface as
    /// its own mount keyed by its own channel. This is the leaf-granularity
    /// decision the estimated seed and the assembler both depend on — collapse
    /// it to one mount, or key it off the wrong string, and one of the two
    /// aiding inputs silently resolves to no frame.
    #[test]
    fn imu_yields_one_mount_per_channel() {
        let imu = installed(
            SensorDeviceConfig::Imu(ImuConfig {
                rate: 100.0,
                accel_bias: [0.0; 3],
                accel_noise_stddev: [0.0; 3],
                gyro_bias: [0.0; 3],
                gyro_noise_stddev: [0.0; 3],
            }),
            &[
                (ACCEL_CHANNEL_FIELD, "body/accel"),
                (GYRO_CHANNEL_FIELD, "body/gyro"),
            ],
        );

        let mounts = imu.frame_mounts();

        assert_eq!(mounts.len(), 2);
        let channels: Vec<&str> = mounts.iter().map(|(c, _, _)| c.as_str()).collect();
        assert!(channels.contains(&"body/accel"));
        assert!(channels.contains(&"body/gyro"));
        // Both share the IMU's single pose and are body-frame (FLU).
        for (_, pose, convention) in &mounts {
            assert_eq!(pose.translation, mount_pose().translation);
            assert_eq!(*convention, Convention::Flu);
        }
    }

    /// A GPS is one frame on one channel; the mount carries its channel verbatim
    /// and the sensor's own convention.
    #[test]
    fn gps_yields_a_single_mount_on_its_channel() {
        let gps = installed(
            SensorDeviceConfig::Gps(GpsConfig {
                rate: 10.0,
                bias: [0.0; 3],
                noise_stddev: [0.0; 3],
            }),
            &[(CHANNEL_FIELD, "gps")],
        );

        let mounts = gps.frame_mounts();

        assert_eq!(mounts.len(), 1);
        assert_eq!(mounts[0].0, "gps");
        assert_eq!(mounts[0].1.translation, mount_pose().translation);
        assert_eq!(mounts[0].2, Convention::Flu);
    }

    /// A magnetometer follows the same single-mount shape as the GPS — this pins
    /// the second single-channel sensor so the one-mount path is not tied to one
    /// variant's quirks.
    #[test]
    fn magnetometer_yields_a_single_mount_on_its_channel() {
        let mag = installed(
            SensorDeviceConfig::Magnetometer(MagnetometerConfig {
                rate: 50.0,
                bias: [0.0; 3],
                noise_stddev: [0.0; 3],
            }),
            &[(CHANNEL_FIELD, "mag")],
        );

        let mounts = mag.frame_mounts();

        assert_eq!(mounts.len(), 1);
        assert_eq!(mounts[0].0, "mag");
        assert_eq!(mounts[0].2, Convention::Flu);
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
