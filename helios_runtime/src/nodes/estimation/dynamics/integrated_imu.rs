//! The `IntegratedImu` dynamics kind: a strapdown INS driven by the IMU's
//! specific force and angular rate, read from the bus each predict.

use super::{DynamicsComponent, EstimatorInputBuilder};
use crate::assembly::BuildContext;
use crate::nodes::estimation::gravity::default_gravity_enu;
use crate::nodes::estimation::EstimatorComponents;
use crate::port::{ChannelKey, PortBus, SensorChannel};
use crate::prelude::TickContext;

use helios_core::estimation::dynamics::integrated_imu::{
    ImuInitialUncertainty, ImuProcessNoise, IntegratedImuModel,
};
use helios_core::estimation::schema::{InputSchema, InputSchemaBlock};
use helios_core::estimation::EstimatorInputs;
use helios_core::prelude::{Acceleration, AgentId, AngularRate, SensorReading};
use helios_core::spatial::state::Quantity;
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

use nalgebra::{DVector, Vector3};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// The `kind` a `dynamics` sub-table writes for the IMU-driven INS.
pub(crate) const INTEGRATED_IMU_KIND: &str = "IntegratedImu";

/// Default prior std dev on position (m): large, for a cold start whose pose
/// is unknown until GNSS arrives.
pub(crate) const DEFAULT_POSITION_UNCERTAINTY_M: f64 = 1000.0;
/// Default prior std dev on velocity (m/s).
pub(crate) const DEFAULT_VELOCITY_UNCERTAINTY_MPS: f64 = 1.0;
/// Default prior std dev on attitude (degrees): any heading.
pub(crate) const DEFAULT_ORIENTATION_UNCERTAINTY_DEG: f64 = 180.0;
/// Default prior std dev on accelerometer bias (m/s²).
pub(crate) const DEFAULT_ACCEL_BIAS_UNCERTAINTY_MPS2: f64 = 0.1;
/// Default prior std dev on gyro bias (rad/s).
pub(crate) const DEFAULT_GYRO_BIAS_UNCERTAINTY_RADPS: f64 = 0.01;

/// The `[dynamics]` sub-table of the `IntegratedImu` kind.
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct IntegratedImuConfig {
    /// The filter's believed gravity, world ENU `[east, north, up]` (m/s²).
    #[serde(default = "default_gravity_enu")]
    gravity_enu: [f64; 3],
    /// Velocity random walk: accelerometer white-noise std dev (m/s²/√Hz).
    accel_noise_stddev: f64,
    /// Angle random walk: gyro white-noise std dev (rad/s/√Hz).
    gyro_noise_stddev: f64,
    /// Accelerometer bias instability std dev (m/s²/√Hz); the bias block's Q.
    accel_bias_instability: f64,
    /// Gyro bias instability std dev (rad/s/√Hz); the bias block's Q.
    gyro_bias_instability: f64,
    /// The prior's std dev per block; squared onto P₀.
    #[serde(default)]
    initial_uncertainty: InitialUncertaintyConfig,
    /// Channel the predict step reads `Vec<SensorReading<Acceleration>>` from.
    accel_channel: String,
    /// Channel the predict step reads `Vec<SensorReading<AngularRate>>` from.
    gyro_channel: String,
}

/// Prior std dev on each INS block: the state's P₀, which the model bakes
/// into its schema.
#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct InitialUncertaintyConfig {
    #[serde(default = "default_position_m")]
    position_m: f64,
    #[serde(default = "default_velocity_mps")]
    velocity_mps: f64,
    /// Degrees in config; converted to radians on load.
    #[serde(default = "default_orientation_deg")]
    orientation_deg: f64,
    /// Distinct from `accel_bias_instability`, which is the block's Q.
    #[serde(default = "default_accel_bias_mps2")]
    accel_bias_mps2: f64,
    /// Too large seeds attitude error outside the filter's linear regime.
    #[serde(default = "default_gyro_bias_radps")]
    gyro_bias_radps: f64,
}

impl Default for InitialUncertaintyConfig {
    fn default() -> Self {
        Self {
            position_m: DEFAULT_POSITION_UNCERTAINTY_M,
            velocity_mps: DEFAULT_VELOCITY_UNCERTAINTY_MPS,
            orientation_deg: DEFAULT_ORIENTATION_UNCERTAINTY_DEG,
            accel_bias_mps2: DEFAULT_ACCEL_BIAS_UNCERTAINTY_MPS2,
            gyro_bias_radps: DEFAULT_GYRO_BIAS_UNCERTAINTY_RADPS,
        }
    }
}

fn default_position_m() -> f64 {
    DEFAULT_POSITION_UNCERTAINTY_M
}

fn default_velocity_mps() -> f64 {
    DEFAULT_VELOCITY_UNCERTAINTY_MPS
}

fn default_orientation_deg() -> f64 {
    DEFAULT_ORIENTATION_UNCERTAINTY_DEG
}

fn default_accel_bias_mps2() -> f64 {
    DEFAULT_ACCEL_BIAS_UNCERTAINTY_MPS2
}

fn default_gyro_bias_radps() -> f64 {
    DEFAULT_GYRO_BIAS_UNCERTAINTY_RADPS
}

/// Assembles a 6-element IMU control vector `[ax, ay, az, wx, wy, wz]` from
/// the most recent linear acceleration and angular velocity readings on the bus.
/// Returns `None` if either channel is empty (cold-start or sensor dropout).
///
/// Declares its input as body (`base_link`) specific force and angular
/// velocity in FLU. The readings are in the IMU's own frame, so the
/// declaration holds only for an IMU mounted at identity on `base_link`.
pub(crate) struct IntegratedImuInputBuilder {
    input_schema: Arc<InputSchema>,
    accel_channel: ChannelKey,
    gyro_channel: ChannelKey,
    required: Vec<ChannelKey>,
}

impl IntegratedImuInputBuilder {
    pub(crate) fn new(
        agent: AgentId,
        accel_channel: impl Into<Arc<str>>,
        gyro_channel: impl Into<Arc<str>>,
    ) -> Self {
        let body = FrameId::base_link(agent);
        let input_schema = Arc::new(InputSchema::compose(vec![
            InputSchemaBlock::new(Quantity::SpecificForce(body.clone()), Convention::Flu),
            InputSchemaBlock::new(Quantity::AngularVelocity(body), Convention::Flu),
        ]));

        let accel_channel: ChannelKey =
            SensorChannel::named::<Vec<SensorReading<Acceleration>>>(accel_channel).into();
        let gyro_channel: ChannelKey =
            SensorChannel::named::<Vec<SensorReading<AngularRate>>>(gyro_channel).into();

        Self {
            input_schema,
            accel_channel: accel_channel.clone(),
            gyro_channel: gyro_channel.clone(),
            required: vec![accel_channel, gyro_channel],
        }
    }
}

impl EstimatorInputBuilder for IntegratedImuInputBuilder {
    fn input_schema(&self) -> Arc<InputSchema> {
        Arc::clone(&self.input_schema)
    }

    fn assemble(&self, bus: &PortBus, _tick: &TickContext) -> Option<EstimatorInputs> {
        let accel_stamped =
            bus.read::<Vec<SensorReading<Acceleration>>>(self.accel_channel.clone())?;
        let gyro_stamped =
            bus.read::<Vec<SensorReading<AngularRate>>>(self.gyro_channel.clone())?;

        let accel = accel_stamped.value.last()?;
        let gyro = gyro_stamped.value.last()?;

        let control = DVector::from_row_slice(&[
            accel.data.0.x,
            accel.data.0.y,
            accel.data.0.z,
            gyro.data.0.x,
            gyro.data.0.y,
            gyro.data.0.z,
        ]);

        Some(EstimatorInputs { control })
    }

    fn required_channels(&self) -> &[ChannelKey] {
        &self.required
    }

    fn optional_channels(&self) -> &[ChannelKey] {
        &[]
    }
}

pub(super) fn register(components: &mut EstimatorComponents) {
    components
        .register_dynamics(INTEGRATED_IMU_KIND, build)
        .expect("IntegratedImu is registered once, by the default components");
}

fn build(config: IntegratedImuConfig, ctx: &BuildContext<'_>) -> Result<DynamicsComponent, String> {
    let prior = &config.initial_uncertainty;
    let dynamics = IntegratedImuModel::new(
        ctx.agent().clone(),
        Vector3::from(config.gravity_enu),
        ImuProcessNoise {
            accel_noise_var: config.accel_noise_stddev.powi(2),
            gyro_noise_var: config.gyro_noise_stddev.powi(2),
            accel_bias_var: config.accel_bias_instability.powi(2),
            gyro_bias_var: config.gyro_bias_instability.powi(2),
        },
        ImuInitialUncertainty {
            pos_var: prior.position_m.powi(2),
            vel_var: prior.velocity_mps.powi(2),
            ori_var: prior.orientation_deg.to_radians().powi(2),
            accel_bias_var: prior.accel_bias_mps2.powi(2),
            gyro_bias_var: prior.gyro_bias_radps.powi(2),
        },
    );
    let input = IntegratedImuInputBuilder::new(
        ctx.agent().clone(),
        config.accel_channel.as_str(),
        config.gyro_channel.as_str(),
    );

    DynamicsComponent::new(Box::new(dynamics), Box::new(input)).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assembly::{AutonomyRegistry, ComponentError};

    use std::collections::HashSet;

    const NODE: &str = "primary";

    const SECTION: &str = r#"
        kind = "IntegratedImu"
        accel_noise_stddev = 0.1
        gyro_noise_stddev = 0.01
        accel_bias_instability = 0.001
        gyro_bias_instability = 0.0001
        accel_channel = "imu/accel"
        gyro_channel = "imu/gyro"
    "#;

    /// The component and its resolved sub-table.
    struct Built {
        component: DynamicsComponent,
        resolved: toml::Table,
    }

    fn build_section(section: &str) -> Result<Built, ComponentError> {
        let registry = AutonomyRegistry::default();
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let channels = HashSet::new();
        let ctx = BuildContext::new(AgentId::new("car"), NODE, &channels);
        registry
            .extension::<EstimatorComponents>()
            .expect("the default registry has estimator components")
            .build_dynamics("dynamics", section, &ctx)
            .map(|built| Built {
                component: built.component,
                resolved: built.resolved,
            })
    }

    /// The built-in kind builds a 16-stored / 15-tangent INS whose builder
    /// reads the two named channels, and its builder agrees with the model
    /// (the component would not exist otherwise).
    #[test]
    fn integrated_imu_builds_the_ins_and_its_builder() {
        let Ok(built) = build_section(SECTION) else {
            panic!("the built-in kind builds");
        };
        let component = built.component;
        let schema = component.dynamics().schema();
        assert_eq!((schema.storage_dim(), schema.tangent_dim()), (16, 15));
        assert_eq!(component.input().required_channels().len(), 2);
        assert_eq!(
            component.input().input_schema().blocks(),
            component.dynamics().input_schema().blocks()
        );
    }

    /// The resolved sub-table shows the defaulted gravity and prior, so the
    /// dump says what the filter believes.
    #[test]
    fn resolved_sub_table_shows_the_defaults() {
        let Ok(built) = build_section(SECTION) else {
            panic!("the built-in kind builds");
        };
        let prior = built.resolved["initial_uncertainty"]
            .as_table()
            .expect("prior table");
        assert_eq!(
            prior["position_m"].as_float(),
            Some(DEFAULT_POSITION_UNCERTAINTY_M)
        );
        assert_eq!(
            built.resolved["gravity_enu"].as_array().map(Vec::len),
            Some(3)
        );
        assert_eq!(built.resolved["kind"].as_str(), Some(INTEGRATED_IMU_KIND));
    }

    /// The prior's std devs land squared on P₀, orientation in radians.
    #[test]
    fn initial_uncertainty_sets_the_prior_covariance() {
        let section =
            format!("{SECTION}\n[initial_uncertainty]\nposition_m = 3.0\norientation_deg = 10.0");
        let Ok(built) = build_section(&section) else {
            panic!("builds with a custom prior");
        };
        let p0 = built
            .component
            .dynamics()
            .schema()
            .initial_covariance()
            .clone();
        assert_eq!(p0[(0, 0)], 9.0, "position variance");
        assert!((p0[(6, 6)] - 10.0_f64.to_radians().powi(2)).abs() < 1e-15);
    }

    #[test]
    fn a_misspelled_prior_key_is_rejected_at_its_path() {
        let section = format!("{SECTION}\n[initial_uncertainty]\npositon_m = 3.0");
        let Err(err) = build_section(&section) else {
            panic!("a misspelled key must not build");
        };
        let ComponentError::InvalidConfig { key, .. } = &err else {
            panic!("expected InvalidConfig, got {err}");
        };
        assert_eq!(key, "initial_uncertainty.positon_m");
    }
}
