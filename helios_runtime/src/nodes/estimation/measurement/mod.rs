//! The measurement component: a measurement model, the typed reader for the
//! payload it consumes, and its noise R, as one [`MeasurementSource`].
//!
//! The source only reads. Applying a reading (an EKF update, a factor added
//! to a graph) is the estimator family's job, so every family shares one
//! source per sensor.
//!
//! - `reader` — [`PayloadReader`], which takes each reading once, in order.
//! - `models` — the built-in measurement kinds.

mod models;
mod reader;

pub(crate) use reader::{Measurement, PayloadReader};

use crate::assembly::BuildContext;
use crate::nodes::estimation::EstimatorComponents;
use crate::port::{ChannelKey, PortBus, SensorChannel};

use helios_core::estimation::measurement::MeasurementModel;
use helios_core::estimation::schema::MeasurementAgreementError;
use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::interchange::measurement::sensor::SensorPayload;
use helios_core::spatial::transforms::Convention;
use helios_core::spatial::FrameId;

use nalgebra::DMatrix;

/// One sensor's readings, the model that predicts them, and their noise R.
///
/// Built only by the measurement table, from a model registered together with
/// its payload type, so the model can't be paired with a channel carrying a
/// different payload.
pub(crate) trait MeasurementSource: Send + Sync {
    /// The sensor channel the readings arrive on.
    fn channel(&self) -> &ChannelKey;

    /// The model predicting each reading from the state.
    fn model(&self) -> &dyn MeasurementModel;

    /// The measurement noise covariance R, square and sized to the model's
    /// measurement.
    fn noise(&self) -> &DMatrix<f64>;

    /// Every reading not taken before, oldest first.
    fn take_new(&self, bus: &PortBus) -> Vec<Measurement>;
}

/// What the measurement table receives besides the model's config: where the
/// readings come from, the frame they are measured in, and how noisy they
/// are. These come from the estimator's aiding entry and the channel's
/// writer, not from the model.
pub(crate) struct MeasurementWiring {
    /// The sensor channel name.
    pub(crate) input: String,
    /// The channel's mount, as its writer states it: the sensor frame the
    /// model predicts readings in.
    pub(crate) mount: FrameId,
    /// The measurement noise covariance R.
    pub(crate) noise: DMatrix<f64>,
}

/// The [`MeasurementSource`] for payload `P`.
struct TypedMeasurementSource<P: SensorPayload> {
    reader: PayloadReader<P>,
    model: Box<dyn MeasurementModel>,
    noise: DMatrix<f64>,
}

impl<P: SensorPayload> TypedMeasurementSource<P> {
    /// Fails if R is not square or not sized to the model's measurement: the
    /// filter would skip every update from this sensor, which looks like a
    /// dead sensor rather than a config error.
    fn new(
        input: &str,
        model: Box<dyn MeasurementModel>,
        noise: DMatrix<f64>,
    ) -> Result<Self, String> {
        if !noise.is_square() {
            return Err(format!(
                "R is {}×{}; it must be square",
                noise.nrows(),
                noise.ncols()
            ));
        }
        let schema_dim = model.schema().dim();
        if schema_dim != noise.nrows() {
            return Err(MeasurementAgreementError::DimensionMismatch {
                schema_dim,
                expected: noise.nrows(),
            }
            .to_string());
        }

        Ok(Self {
            reader: PayloadReader::new(readings_channel::<P>(input)),
            model,
            noise,
        })
    }
}

impl<P: SensorPayload> MeasurementSource for TypedMeasurementSource<P> {
    fn channel(&self) -> &ChannelKey {
        self.reader.channel()
    }

    fn model(&self) -> &dyn MeasurementModel {
        &*self.model
    }

    fn noise(&self) -> &DMatrix<f64> {
        &self.noise
    }

    fn take_new(&self, bus: &PortBus) -> Vec<Measurement> {
        self.reader.take_new(bus)
    }
}

/// The channel a measurement kind reads, from the input its aiding entry
/// names. Registered with the kind, so a node knows the typed channel before
/// the model is built.
pub(crate) type ReadingChannel = fn(&str) -> ChannelKey;

/// What a measurement kind declares before its model is built: the channel
/// it reads, and the axes its model reads that channel's mount in through
/// the tf tree (`None` for a model that never resolves the mount).
#[derive(Clone, Copy)]
pub(crate) struct MeasurementDeclaration {
    pub(crate) channel: ReadingChannel,
    pub(crate) mount_convention: Option<Convention>,
}

/// The typed channel readings of payload `P` arrive on, named `input`.
fn readings_channel<P: SensorPayload>(input: &str) -> SensorChannel {
    SensorChannel::named::<Vec<SensorReading<P>>>(input)
}

/// The [`ReadingChannel`] of payload `P`.
pub(crate) fn reading_channel<P: SensorPayload>() -> ReadingChannel {
    |input| readings_channel::<P>(input).into()
}

/// Turns a model factory into the measurement table's factory: builds the
/// model for the wiring's mount, then pairs it with a reader for payload `P`
/// and the wiring's R.
pub(crate) fn source_factory<P, C, F>(
    build_model: F,
) -> impl Fn(C, &BuildContext<'_>, MeasurementWiring) -> Result<Box<dyn MeasurementSource>, String>
       + Send
       + Sync
       + 'static
where
    P: SensorPayload,
    F: Fn(C, &BuildContext<'_>, &FrameId) -> Result<Box<dyn MeasurementModel>, String>
        + Send
        + Sync
        + 'static,
{
    move |config: C, ctx: &BuildContext<'_>, wiring: MeasurementWiring| {
        let model = build_model(config, ctx, &wiring.mount)?;
        let source = TypedMeasurementSource::<P>::new(&wiring.input, model, wiring.noise)?;
        Ok(Box::new(source) as Box<dyn MeasurementSource>)
    }
}

pub(super) fn register(components: &mut EstimatorComponents) {
    models::register(components);
}

#[cfg(test)]
mod tests {
    use super::models::GPS_POSITION_KIND;
    use super::*;

    use crate::assembly::{no_check, AutonomyRegistry, ComponentError, NoParams};
    use crate::port::PortDescriptor;
    use crate::stamped::{Health, Stamped};

    use helios_core::estimation::measurement::Prediction;
    use helios_core::estimation::schema::{MeasurementSchema, MeasurementSchemaBlock};
    use helios_core::interchange::measurement::sensor::{GpsPosition, MagneticField};
    use helios_core::prelude::{AgentId, MonotonicTime, TfProvider};
    use helios_core::spatial::state::Quantity;
    use helios_core::spatial::transforms::Convention;
    use helios_core::spatial::FrameAwareState;

    use nalgebra::{DVector, Vector3};

    const NODE: &str = "primary";
    /// The kind of the node drawing the component; the table never reads it.
    const NODE_KIND: &str = "TestEstimator";
    const DUMMY_KIND: &str = "mocap_position";
    const INPUT: &str = "mocap";
    /// The mount the wiring states, apart from the channel name, so a model
    /// built for the name instead would be caught.
    const MOUNT: &str = "mocap_marker";

    fn agent() -> AgentId {
        AgentId::new("car")
    }

    /// A dummy position model, standing in for one a researcher registers.
    struct MocapModel;

    impl MeasurementModel for MocapModel {
        fn schema(&self) -> MeasurementSchema {
            MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::odom(agent())),
                Convention::Enu,
            )])
        }

        fn predict_measurement(
            &self,
            _: &FrameAwareState,
            _: Option<&dyn TfProvider>,
            _: MonotonicTime,
        ) -> Prediction {
            Prediction::Ready(DVector::zeros(3))
        }
    }

    fn build_mocap(
        _: NoParams,
        _: &BuildContext<'_>,
        _: &FrameId,
    ) -> Result<Box<dyn MeasurementModel>, String> {
        Ok(Box::new(MocapModel))
    }

    /// A registry with the dummy model registered for `GpsPosition` payloads.
    fn registry() -> AutonomyRegistry {
        let mut registry = AutonomyRegistry::default();
        registry
            .extension_mut::<EstimatorComponents>()
            .register_measurement::<GpsPosition, _, _, _>(DUMMY_KIND, None, no_check, build_mocap)
            .expect("not a built-in kind");
        registry
    }

    fn build(
        registry: &AutonomyRegistry,
        section: &str,
        r_len: usize,
    ) -> Result<Box<dyn MeasurementSource>, ComponentError> {
        let section: toml::Table = toml::from_str(section).expect("test TOML parses");
        let ctx = BuildContext::new(agent(), NODE, NODE_KIND);
        let wiring = MeasurementWiring {
            input: INPUT.to_string(),
            mount: FrameId::agent(agent(), MOUNT),
            noise: DMatrix::identity(r_len, r_len),
        };
        registry
            .extension::<EstimatorComponents>()
            .expect("the default registry has estimator components")
            .parse_measurement("aiding.mocap.model", section, &ctx)
            .and_then(|parsed| parsed.deferred.build(&ctx, wiring))
    }

    /// A registered model yields a source reading its payload's type on the
    /// wired channel, with the model built for that channel's sensor frame.
    /// The kind declares that channel before the source is built.
    #[test]
    fn a_registered_model_reads_its_payload_on_the_wired_channel() {
        let registry = registry();
        let section = format!("kind = \"{DUMMY_KIND}\"");
        let Ok(source) = build(&registry, &section, 3) else {
            panic!("the dummy model builds");
        };

        let expected: ChannelKey =
            SensorChannel::named::<Vec<SensorReading<GpsPosition>>>(INPUT).into();
        assert_eq!(source.channel(), &expected);
        let parsed = registry
            .extension::<EstimatorComponents>()
            .expect("the default registry has estimator components")
            .parse_measurement(
                "aiding.mocap.model",
                toml::from_str(&section).expect("test TOML parses"),
                &BuildContext::new(agent(), NODE, NODE_KIND),
            )
            .expect("the dummy model parses");
        assert_eq!((parsed.declared.channel)(INPUT), expected);
        assert_eq!(parsed.declared.mount_convention, None);
        assert_eq!(source.noise().nrows(), 3);

        // The source reads that channel end to end.
        let bus = PortBus::new(&[PortDescriptor::new(
            vec![],
            vec![],
            vec![expected.clone()],
            None,
        )]);
        bus.write(
            expected,
            Stamped {
                value: vec![SensorReading {
                    sensor: FrameId::agent(agent(), INPUT),
                    timestamp: MonotonicTime(1.0),
                    data: GpsPosition {
                        position: Vector3::new(1.0, 2.0, 3.0),
                    },
                }],
                timestamp: MonotonicTime(1.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("channel is on the bus");
        let taken = source.take_new(&bus);
        assert_eq!(taken.len(), 1);
        assert_eq!(taken[0].z, DVector::from_row_slice(&[1.0, 2.0, 3.0]));
    }

    /// The model is built for the mount the wiring states, not a frame named
    /// after the channel.
    #[test]
    fn the_model_is_built_for_the_wired_mount() {
        let ctx = BuildContext::new(agent(), NODE, NODE_KIND);
        let factory = source_factory::<GpsPosition, NoParams, _>(
            |_: NoParams, _: &BuildContext<'_>, sensor: &FrameId| {
                assert_eq!(sensor, &FrameId::agent(agent(), MOUNT));
                Ok(Box::new(MocapModel) as Box<dyn MeasurementModel>)
            },
        );
        let wiring = MeasurementWiring {
            input: INPUT.to_string(),
            mount: FrameId::agent(agent(), MOUNT),
            noise: DMatrix::identity(3, 3),
        };
        assert!(factory(NoParams {}, &ctx, wiring).is_ok());
    }

    /// An R sized for another measurement would make every update a silent
    /// no-op; the build refuses it.
    #[test]
    fn r_of_the_wrong_size_is_refused() {
        let Err(err) = build(&registry(), &format!("kind = \"{DUMMY_KIND}\""), 2) else {
            panic!("a mis-sized R must not build");
        };
        assert!(matches!(err, ComponentError::BuildFailed { .. }), "{err}");
    }

    #[test]
    fn a_non_square_r_is_refused() {
        let model: Box<dyn MeasurementModel> = Box::new(MocapModel);
        let result = TypedMeasurementSource::<GpsPosition>::new(INPUT, model, DMatrix::zeros(3, 2));
        assert!(result.is_err());
    }

    /// One model kind, two payload registrations under two names: each source
    /// reads its own payload type, so the two can't be crossed in config.
    #[test]
    fn a_model_registered_for_two_payloads_reads_each_type() {
        let mut registry = registry();
        registry
            .extension_mut::<EstimatorComponents>()
            .register_measurement::<MagneticField, _, _, _>(
                "mocap_as_field",
                None,
                no_check,
                build_mocap,
            )
            .expect("distinct name");

        let field = build(&registry, "kind = \"mocap_as_field\"", 3).expect("builds");
        let expected: ChannelKey =
            SensorChannel::named::<Vec<SensorReading<MagneticField>>>(INPUT).into();
        assert_eq!(field.channel(), &expected);
    }

    #[test]
    fn registering_a_built_in_measurement_kind_again_is_rejected() {
        let mut registry = AutonomyRegistry::default();
        let err = registry
            .extension_mut::<EstimatorComponents>()
            .register_measurement::<GpsPosition, _, _, _>(
                GPS_POSITION_KIND,
                None,
                no_check,
                build_mocap,
            )
            .expect_err("gps_position is a built-in");
        assert_eq!(
            err.to_string(),
            format!("measurement kind '{GPS_POSITION_KIND}' is already registered")
        );
    }
}
