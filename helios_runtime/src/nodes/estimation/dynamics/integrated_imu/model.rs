//! The `IntegratedImu` model's input builder and factory.

use super::super::{
    AssembledInput, DeclaredInput, DynamicsComponent, EstimatorInputBuilder, InputUnavailable,
};
use super::config::IntegratedImuConfig;
use crate::assembly::BuildContext;
use crate::nodes::estimation::EstimatorComponents;
use crate::pipeline::InputMounts;
use crate::port::{ChannelKey, PortBus, SensorChannel};
use crate::prelude::TickContext;

use helios_core::estimation::dynamics::integrated_imu::{
    ImuInitialUncertainty, ImuProcessNoise, IntegratedImuModel,
};
use helios_core::estimation::imu_mount::{body_angular_velocity, body_specific_force};
use helios_core::estimation::schema::{InputSchema, InputSchemaBlock};
use helios_core::estimation::EstimatorInputs;
use helios_core::prelude::{Acceleration, AngularRate, MonotonicTime, SensorReading};
use helios_core::spatial::conventions::Flu;
use helios_core::spatial::state::{Component, Quantity};
use helios_core::spatial::tf::TfProvider;
use helios_core::spatial::transforms::{Convention, ConventionOf};
use helios_core::spatial::{FrameId, StateVariable};

use nalgebra::{DVector, Isometry3, Vector3};
use std::sync::Arc;

/// The `kind` a `dynamics` sub-table writes for the IMU-driven INS.
pub(crate) const INTEGRATED_IMU_KIND: &str = "IntegratedImu";

/// The axes the builder reads each IMU channel's mount in.
type MountAxes = Flu;

/// Assembles a 6-element IMU control vector `[fx, fy, fz, wx, wy, wz]`, body
/// (`base_link`) specific force and angular velocity in FLU, from the most
/// recent accelerometer and gyro readings on the bus.
///
/// Each reading is in its own mount's axes, so each tick the builder looks up
/// `base_link ← mount` for both and moves the readings to the body origin:
/// the rate rotated, the specific force rotated and cleared of the
/// centripetal term its lever arm adds. It never reads the state, so the
/// tangential term is left in (see [`body_specific_force`]). The transforms
/// are read each tick, so an extrinsic updated while running is used.
pub(crate) struct IntegratedImuInputBuilder {
    input_schema: Arc<InputSchema>,
    /// Row of the input where the specific force starts.
    specific_force_row: usize,
    /// Row of the input where the angular velocity starts.
    angular_velocity_row: usize,
    body: FrameId,
    accel: MountedChannel,
    gyro: MountedChannel,
    required: Vec<ChannelKey>,
}

impl IntegratedImuInputBuilder {
    /// A builder moving readings to `body`, reading `accel` and `gyro`, each
    /// mounted where its writer states. Each quantity is written at the row
    /// its schema gives it, so the schema is the one place the order is
    /// stated.
    fn new(body: FrameId, accel: MountedChannel, gyro: MountedChannel) -> Result<Self, String> {
        let input_schema = imu_input_schema(&body);
        let row_of = |quantity: Quantity| {
            input_schema
                .offset_of(&StateVariable::new(quantity.clone(), Component::X))
                .ok_or_else(|| format!("the IMU input schema has no {quantity:?} block"))
        };
        Ok(Self {
            specific_force_row: row_of(Quantity::SpecificForce(body.clone()))?,
            angular_velocity_row: row_of(Quantity::AngularVelocity(body.clone()))?,
            input_schema,
            body,
            required: vec![accel.channel.clone(), gyro.channel.clone()],
            accel,
            gyro,
        })
    }

    /// `base_link ← mount` for `channel`'s mount at `at`, as an isometry in
    /// FLU, or why it can't be had.
    fn mount_in_body(
        &self,
        channel: &MountedChannel,
        tf: &dyn TfProvider,
        at: MonotonicTime,
    ) -> Result<Isometry3<f64>, InputUnavailable> {
        let erased = tf
            .get_transform(channel.mount.clone(), self.body.clone(), at)
            .ok_or_else(|| InputUnavailable::MissingTransform {
                from: channel.mount.clone(),
                to: self.body.clone(),
            })?;
        erased
            .typed::<MountAxes, Flu>()
            .map(|transform| transform.into_inner())
            .map_err(|_| InputUnavailable::ConventionMismatch {
                from: channel.mount.clone(),
                to: self.body.clone(),
            })
    }

    /// The input from the latest reading on each channel, moved to the body
    /// origin, or why there is none.
    fn try_assemble(
        &self,
        bus: &PortBus,
        tf: &dyn TfProvider,
    ) -> Result<EstimatorInputs, InputUnavailable> {
        let accel_stamped = bus
            .read::<Vec<SensorReading<Acceleration>>>(self.accel.channel.clone())
            .ok_or(InputUnavailable::NoReading)?;
        let gyro_stamped = bus
            .read::<Vec<SensorReading<AngularRate>>>(self.gyro.channel.clone())
            .ok_or(InputUnavailable::NoReading)?;
        let accel = accel_stamped
            .value
            .last()
            .ok_or(InputUnavailable::NoReading)?;
        let gyro = gyro_stamped
            .value
            .last()
            .ok_or(InputUnavailable::NoReading)?;

        let gyro_in_body = self.mount_in_body(&self.gyro, tf, gyro.timestamp)?;
        let accel_in_body = self.mount_in_body(&self.accel, tf, accel.timestamp)?;

        let rate = body_angular_velocity(&gyro_in_body.rotation, &gyro.data.0);
        let force = body_specific_force(
            &accel_in_body.rotation,
            &accel_in_body.translation.vector,
            &accel.data.0,
            &rate,
            None,
        );

        let mut control = DVector::zeros(self.input_schema.dim());
        control
            .fixed_rows_mut::<3>(self.specific_force_row)
            .copy_from(&force);
        control
            .fixed_rows_mut::<3>(self.angular_velocity_row)
            .copy_from(&rate);
        Ok(EstimatorInputs { control })
    }
}

impl EstimatorInputBuilder for IntegratedImuInputBuilder {
    fn input_schema(&self) -> Arc<InputSchema> {
        Arc::clone(&self.input_schema)
    }

    fn assemble(&self, bus: &PortBus, tf: &dyn TfProvider, _tick: &TickContext) -> AssembledInput {
        match self.try_assemble(bus, tf) {
            Ok(inputs) => AssembledInput::Ready(inputs),
            Err(reason) => AssembledInput::Unavailable(reason),
        }
    }

    fn required_channels(&self) -> &[ChannelKey] {
        &self.required
    }

    fn optional_channels(&self) -> &[ChannelKey] {
        &[]
    }
}

/// A channel the builder reads, with the mount its writer states.
struct MountedChannel {
    channel: ChannelKey,
    mount: FrameId,
}

impl MountedChannel {
    /// `channel` at its mount in `mounts`.
    fn new(channel: ChannelKey, mounts: &InputMounts) -> Result<Self, String> {
        let mount = mounts
            .mount(&channel)
            .map_err(|err| err.to_string())?
            .clone();
        Ok(Self { channel, mount })
    }
}

/// The input the builder assembles: specific force and angular velocity of
/// `body`, FLU.
fn imu_input_schema(body: &FrameId) -> Arc<InputSchema> {
    Arc::new(InputSchema::compose(vec![
        InputSchemaBlock::new(Quantity::SpecificForce(body.clone()), Convention::Flu),
        InputSchemaBlock::new(Quantity::AngularVelocity(body.clone()), Convention::Flu),
    ]))
}

pub(in crate::nodes::estimation) fn register(components: &mut EstimatorComponents) {
    components
        .register_dynamics(INTEGRATED_IMU_KIND, IntegratedImuConfig::check, build)
        .expect("IntegratedImu is registered once, by the default components");
}

fn build(config: IntegratedImuConfig, ctx: &BuildContext<'_>) -> Result<DynamicsComponent, String> {
    let prior = &config.initial_uncertainty;
    let dynamics = IntegratedImuModel::new(
        ctx.frames().clone(),
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
    let accel: ChannelKey =
        SensorChannel::named::<Vec<SensorReading<Acceleration>>>(config.accel_channel).into();
    let gyro: ChannelKey =
        SensorChannel::named::<Vec<SensorReading<AngularRate>>>(config.gyro_channel).into();
    let body = ctx.frames().body.clone();
    let input = DeclaredInput::new(
        imu_input_schema(&body),
        vec![accel.clone(), gyro.clone()],
        vec![],
        {
            let (accel, gyro) = (accel.clone(), gyro.clone());
            move |mounts: &InputMounts| {
                let accel = MountedChannel::new(accel, mounts)?;
                let gyro = MountedChannel::new(gyro, mounts)?;
                Ok(Box::new(IntegratedImuInputBuilder::new(body, accel, gyro)?)
                    as Box<dyn EstimatorInputBuilder>)
            }
        },
    )
    .reads_mount_in(&accel, MountAxes::CONVENTION)
    .reads_mount_in(&gyro, MountAxes::CONVENTION);

    DynamicsComponent::new(Box::new(dynamics), input).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::super::config::DEFAULT_POSITION_UNCERTAINTY_M;
    use super::*;

    use crate::assembly::{AutonomyRegistry, ComponentError};
    use crate::port::{
        AlgorithmNodePortDescriptor, ChannelDescription, PortDescriptor, Provenance,
    };
    use crate::stamped::{Health, Stamped};
    use crate::BodyChannel;

    use helios_core::prelude::AgentId;
    use helios_core::spatial::transforms::ErasedTransform;

    use nalgebra::{Translation3, UnitQuaternion};
    use std::collections::HashMap;
    use std::f64::consts::FRAC_PI_2;

    const NODE: &str = "primary";
    /// The kind of the node drawing the component; the table never reads it.
    const NODE_KIND: &str = "TestEstimator";

    const SECTION: &str = r#"
        kind = "IntegratedImu"
        accel_noise_stddev = 0.1
        gyro_noise_stddev = 0.01
        accel_bias_instability = 0.001
        gyro_bias_instability = 0.0001
        accel_channel = "imu.accel"
        gyro_channel = "imu.gyro"
    "#;

    /// The component and its resolved sub-table.
    struct Built {
        component: DynamicsComponent,
        resolved: toml::Table,
    }

    fn build_section(section: &str) -> Result<Built, ComponentError> {
        let registry = AutonomyRegistry::default();
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let ctx = BuildContext::in_odom(AgentId::new("car"), NODE, NODE_KIND);
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
        let (dynamics, input) = built.component.into_parts();
        let schema = dynamics.schema();
        assert_eq!((schema.storage_dim(), schema.tangent_dim()), (16, 15));
        assert_eq!(input.required_channels().len(), 2);
        assert_eq!(
            input.input_schema().blocks(),
            dynamics.input_schema().blocks()
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

    /// A zero std dev would make Q or P₀ singular, and a non-finite one
    /// would poison the state; each is refused naming its key, never reaching
    /// the model.
    #[test]
    fn an_unusable_std_dev_is_refused_naming_its_key() {
        for value in ["0.0", "-0.1", "nan", "inf"] {
            for (key, section) in [
                (
                    "accel_noise_stddev",
                    SECTION.replace(
                        "accel_noise_stddev = 0.1",
                        &format!("accel_noise_stddev = {value}"),
                    ),
                ),
                (
                    "initial_uncertainty.velocity_mps",
                    format!("{SECTION}\n[initial_uncertainty]\nvelocity_mps = {value}"),
                ),
            ] {
                let Err(err) = build_section(&section) else {
                    panic!("{key} = {value} must not build");
                };
                assert!(matches!(err, ComponentError::BuildFailed { .. }), "{err}");
                assert!(
                    err.to_string().contains(&format!("{key} must be finite")),
                    "{err}"
                );
            }
        }
    }

    #[test]
    fn a_non_finite_gravity_is_refused() {
        let section = format!("{SECTION}\ngravity_enu = [0.0, nan, -9.81]");
        let Err(err) = build_section(&section) else {
            panic!("a non-finite gravity must not build");
        };
        assert!(err.to_string().contains("gravity_enu"), "{err}");
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

    /// The unit the IMU channels come from.
    const UNIT: &str = "imu";
    /// The frame the IMU is mounted at.
    const MOUNT: &str = "imu_link";
    /// Tolerance on an input built from a rotation and a few cross products.
    const EPS: f64 = 1e-12;
    const G: f64 = 9.81;
    /// A steady yaw rate (rad/s).
    const YAW_RATE: f64 = 2.0;

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    fn mount() -> FrameId {
        FrameId::agent(agent(), MOUNT)
    }

    /// A TF provider holding one edge: the IMU's mount in `base_link`, if
    /// given.
    struct MountEdge(Option<Isometry3<f64>>);

    impl TfProvider for MountEdge {
        fn get_transform(
            &self,
            from: FrameId,
            to: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            let edge = self.0?;
            (from == mount() && to == FrameId::base_link(agent()))
                .then(|| ErasedTransform::from_parts(edge, Convention::Flu, Convention::Flu))
        }
    }

    /// The `IntegratedImu` input declared from [`SECTION`].
    fn declared_input() -> DeclaredInput {
        let Ok(built) = build_section(SECTION) else {
            panic!("the built-in kind builds");
        };
        built.component.into_parts().1
    }

    /// The node's ports for `input`'s channels.
    fn ports(input: &DeclaredInput) -> PortDescriptor {
        AlgorithmNodePortDescriptor::new()
            .inputs_from_slices(input.required_channels(), input.optional_channels())
            .build()
    }

    /// Builds the declared input with every channel described by `describe`.
    fn build_input(
        describe: fn(ChannelKey) -> ChannelDescription,
    ) -> Result<Box<dyn EstimatorInputBuilder>, String> {
        let input = declared_input();
        let descriptor = ports(&input);
        let descriptions: Vec<ChannelDescription> = input
            .required_channels()
            .iter()
            .cloned()
            .map(describe)
            .collect();
        let by_key: HashMap<&ChannelKey, &ChannelDescription> = descriptions
            .iter()
            .map(|description| (description.key(), description))
            .collect();
        input.build(&InputMounts::new(NODE, &descriptor, &by_key))
    }

    fn mounted(key: ChannelKey) -> ChannelDescription {
        ChannelDescription::body(
            &BodyChannel::new(key, UNIT, Provenance::Simulated).with_mount(mount()),
        )
    }

    fn unmounted(key: ChannelKey) -> ChannelDescription {
        ChannelDescription::body(&BodyChannel::new(key, UNIT, Provenance::Simulated))
    }

    /// Assembles one input from a bus carrying one accelerometer and one gyro
    /// reading, in the mount's axes, with the mount at `edge` in `base_link`.
    fn assemble(
        force: Vector3<f64>,
        rate: Vector3<f64>,
        edge: Option<Isometry3<f64>>,
    ) -> AssembledInput {
        let builder = match build_input(mounted) {
            Ok(builder) => builder,
            Err(err) => panic!("the mounted input builds: {err}"),
        };
        let bus = PortBus::new(&[PortDescriptor::new(
            vec![],
            vec![],
            builder.required_channels().to_vec(),
            None,
        )]);
        let at = MonotonicTime(1.0);
        fn stamped<T>(value: T, at: MonotonicTime) -> Stamped<T> {
            Stamped {
                value,
                timestamp: at,
                health: Health::Ok,
                producer: 0,
            }
        }
        bus.write(
            builder.required_channels()[0].clone(),
            stamped(
                vec![SensorReading {
                    sensor: mount(),
                    timestamp: at,
                    data: Acceleration(force),
                }],
                at,
            ),
        )
        .expect("the bus carries the accelerometer channel");
        bus.write(
            builder.required_channels()[1].clone(),
            stamped(
                vec![SensorReading {
                    sensor: mount(),
                    timestamp: at,
                    data: AngularRate(rate),
                }],
                at,
            ),
        )
        .expect("the bus carries the gyro channel");

        builder.assemble(&bus, &MountEdge(edge), &TickContext::detached(at, 0.1, 0))
    }

    fn ready(assembled: AssembledInput) -> DVector<f64> {
        match assembled {
            AssembledInput::Ready(inputs) => inputs.control,
            AssembledInput::Unavailable(reason) => panic!("expected an input, got {reason:?}"),
        }
    }

    /// An IMU turned a quarter turn and mounted off the origin, on a body
    /// turning at a steady rate, assembles the input an IMU at identity on the
    /// origin would: the same input, so the same estimate.
    #[test]
    fn a_rotated_offset_mount_assembles_the_identity_mount_input() {
        let origin_force = Vector3::new(0.4, 0.0, G);
        let omega = Vector3::new(0.0, 0.0, YAW_RATE);
        let offset = Vector3::new(0.5, 0.2, 0.1);
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2);
        let edge = Isometry3::from_parts(Translation3::from(offset), rotation);

        let at_sensor = origin_force + omega.cross(&omega.cross(&offset));
        let rotated = ready(assemble(
            rotation.inverse() * at_sensor,
            rotation.inverse() * omega,
            Some(edge),
        ));
        let identity = ready(assemble(origin_force, omega, Some(Isometry3::identity())));

        assert!(
            (&rotated - &identity).amax() < EPS,
            "rotated {rotated:?} vs identity {identity:?}"
        );
    }

    /// A mount whose transform is missing is a loud reason naming the edge,
    /// never the quiet one a dropout gives.
    #[test]
    fn a_missing_extrinsic_is_a_loud_reason() {
        let AssembledInput::Unavailable(reason) =
            assemble(Vector3::zeros(), Vector3::zeros(), None)
        else {
            panic!("no transform, no input");
        };
        assert_eq!(
            reason,
            InputUnavailable::MissingTransform {
                from: mount(),
                to: FrameId::base_link(agent()),
            }
        );
        assert!(reason.loud_cause().is_some());
        assert_eq!(InputUnavailable::NoReading.loud_cause(), None);
    }

    /// The builder is built for its channels' mounts, so an IMU channel whose
    /// writer states none is refused when the node is built, naming the node.
    #[test]
    fn an_unmounted_imu_channel_is_refused() {
        let Err(err) = build_input(unmounted) else {
            panic!("an unmounted IMU must not build");
        };
        assert!(
            err.contains(&format!("node \"{NODE}\" needs the mount of channel")),
            "{err}"
        );
    }
}
