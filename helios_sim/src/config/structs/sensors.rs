use super::pose::Pose;

use helios_core::spatial::transforms::Convention;

use serde::Deserialize;

/// Discriminated union of all sensor kinds.
/// The `tag = "kind"` tells Serde to look for a `kind = "..."` field in the TOML.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum SensorConfig {
    Imu(ImuConfig),
    Gps(GpsConfig),
    Lidar(LidarConfig),
    Magnetometer(MagnetometerConfig),
}

impl SensorConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            SensorConfig::Imu(_) => "Imu",
            SensorConfig::Gps(_) => "Gps",
            SensorConfig::Lidar(_) => "Lidar",
            SensorConfig::Magnetometer(_) => "Magnetometer",
        }
    }

    /// The tf-tree frame mounts this sensor contributes: for each bus channel it
    /// publishes, the `(channel, pose, convention)` of that sensor frame relative
    /// to `base_link`. The pose and convention are the same values the sensor
    /// spawners stamp onto the truth tf tree, so seeding the estimated buffer from
    /// here cannot fork a mount between the two trees. An IMU yields two mounts
    /// (accelerometer and gyroscope share one pose but publish on separate
    /// channels); every other sensor yields one.
    pub fn frame_mounts(&self) -> Vec<(String, Pose, Convention)> {
        match self {
            SensorConfig::Imu(c) => vec![
                (
                    c.get_accel_channel().to_string(),
                    c.get_relative_pose(),
                    Convention::Flu,
                ),
                (
                    c.get_gyro_channel().to_string(),
                    c.get_relative_pose(),
                    Convention::Flu,
                ),
            ],
            SensorConfig::Gps(c) => {
                vec![(c.channel.clone(), c.get_relative_pose(), Convention::Flu)]
            }
            SensorConfig::Lidar(c) => vec![(
                c.get_channel().to_string(),
                c.get_relative_pose(),
                Convention::Flu,
            )],
            SensorConfig::Magnetometer(c) => {
                vec![(c.channel.clone(), c.get_relative_pose(), Convention::Flu)]
            }
        }
    }
}

/// Configuration parameters for a simulated IMU sensor.
///
/// An IMU chip always produces two physical quantities: linear acceleration
/// (`Acceleration`) and angular rate (`AngularRate`). If the
/// chip also has an onboard magnetometer, add a separate `Magnetometer` entry
/// to the sensor suite — that quantity is independently configured and spawns
/// its own sensor with its own forward model.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuConfig {
    pub rate: f64,
    #[serde(default)]
    pub transform: Pose,
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

    /// Bus channel for `Vec<SensorReading<Acceleration>>`. Must match the
    /// accelerometer `input_channel` in the estimator's aiding config. Distinct
    /// from `gyro_channel` — one IMU publishes its two quantities on two separate
    /// channels. The name disambiguates multiple IMUs on a single agent's bus; it
    /// need not be unique across agents, since each agent owns its own bus.
    pub accel_channel: String,
    /// Bus channel for `Vec<SensorReading<AngularRate>>`. Must match the
    /// gyroscope `input_channel` in the estimator's aiding config. Same
    /// per-agent uniqueness rule as `accel_channel`.
    pub gyro_channel: String,
}

impl ImuConfig {
    pub fn get_rate(&self) -> f64 {
        self.rate
    }
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
    /// The accelerometer and gyroscope noise standard deviations, in that order.
    pub fn get_noise_stddevs(&self) -> ([f64; 3], [f64; 3]) {
        (self.accel_noise_stddev, self.gyro_noise_stddev)
    }
    /// The accelerometer and gyroscope biases, in that order.
    pub fn get_biases(&self) -> ([f64; 3], [f64; 3]) {
        (self.accel_bias, self.gyro_bias)
    }

    pub fn get_accel_channel(&self) -> &str {
        &self.accel_channel
    }
    pub fn get_gyro_channel(&self) -> &str {
        &self.gyro_channel
    }
}

/// Configuration parameters for a simulated GPS sensor.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GpsConfig {
    pub rate: f64,
    /// Bus channel name for `Vec<SensorReading<GpsPosition>>` published to the pipeline.
    /// Must match the `input_channel` in the estimator's aiding config.
    pub channel: String,
    #[serde(default)]
    pub transform: Pose,

    /// Constant measurement offset in [East, North, Up] axes, in meters.
    #[serde(default)]
    pub bias: [f64; 3],

    /// Standard deviation of noise in [East, North, Up] axes, in meters.
    #[serde(default)]
    pub noise_stddev: [f64; 3],
}

impl GpsConfig {
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
}

/// Configuration parameters for a simulated 3-axis magnetometer.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MagnetometerConfig {
    pub rate: f64,
    #[serde(default)]
    pub transform: Pose,
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
    /// Bus channel name for `Vec<SensorReading<MagneticField>>` published to the pipeline.
    /// Must match the `input_channel` in the estimator's aiding config.
    pub channel: String,
}

impl MagnetometerConfig {
    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }
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
    #[serde(default)]
    pub transform: Pose,
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
    /// Bus channel name for `Vec<SensorReading<PointCloud<Flu, ()>>>` published to the pipeline.
    pub channel: String,
}

impl LidarConfig {
    pub fn get_rate(&self) -> f64 {
        self.rate
    }

    pub fn get_relative_pose(&self) -> Pose {
        self.transform
    }

    pub fn get_channel(&self) -> &str {
        self.channel.as_str()
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

    use super::*;

    use nalgebra::{UnitQuaternion, Vector3};

    /// A distinctive, non-identity mount so the pose pass-through (not only the
    /// channel name) is actually observed.
    fn mount_pose() -> Pose {
        Pose {
            translation: Vector3::new(0.2, 0.0, 0.3),
            rotation: UnitQuaternion::identity(),
        }
    }

    fn imu(accel: &str, gyro: &str) -> SensorConfig {
        SensorConfig::Imu(ImuConfig {
            rate: 100.0,
            transform: mount_pose(),
            accel_bias: [0.0; 3],
            accel_noise_stddev: [0.0; 3],
            gyro_bias: [0.0; 3],
            gyro_noise_stddev: [0.0; 3],
            accel_channel: accel.to_string(),
            gyro_channel: gyro.to_string(),
        })
    }

    /// An IMU is one config but two frames: accelerometer and gyroscope share a
    /// pose yet publish on separate channels, so each must surface as its own
    /// mount keyed by its own channel. This is the leaf-granularity decision the
    /// estimated seed and the assembler both depend on — collapse it to one mount,
    /// or key it off the wrong string, and one of the two aiding inputs silently
    /// resolves to no frame.
    #[test]
    fn imu_yields_one_mount_per_channel() {
        let mounts = imu("body/accel", "body/gyro").frame_mounts();

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
        let gps = SensorConfig::Gps(GpsConfig {
            rate: 10.0,
            channel: "gps".to_string(),
            transform: mount_pose(),
            bias: [0.0; 3],
            noise_stddev: [0.0; 3],
        });

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
        let mag = SensorConfig::Magnetometer(MagnetometerConfig {
            rate: 50.0,
            transform: mount_pose(),
            bias: [0.0; 3],
            noise_stddev: [0.0; 3],
            channel: "mag".to_string(),
        });

        let mounts = mag.frame_mounts();

        assert_eq!(mounts.len(), 1);
        assert_eq!(mounts[0].0, "mag");
        assert_eq!(mounts[0].2, Convention::Flu);
    }
}
