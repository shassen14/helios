//! [`DeprojectNode`] — turns each organized range field in a sensor batch into
//! a point cloud, one reading in, one reading out.
//!
//! A world sensor publishes what it measured: a grid of ranges along known beam
//! directions. Consumers that want geometry (a mapper, a detector) read points.
//! This node sits between them, so the host never flattens and the consumer
//! never needs to know the sensor's beam layout. Cells that hold a miss are
//! dropped by the conversion.

use crate::{
    pipeline::descriptor::AlgorithmNodePortDescriptor,
    port::{ChannelError, PortBus, SensorChannel},
    ChannelKey, PipelineNode, PortDescriptor, Stamped, TickContext,
};

use helios_core::{
    prelude::{GridAttributes, PointCloud, RangeField, SensorReading, TfProvider},
    spatial::conventions::Frame,
};

use std::{
    marker::PhantomData,
    sync::{atomic::Ordering, Arc},
};

use atomic_float::AtomicF64;

/// The batch the node reads: every range field the host produced this cycle.
type FieldBatch<F, G> = Vec<SensorReading<RangeField<F, G>>>;

/// The batch the node writes: one cloud per input field, same sensors, same
/// order.
type CloudBatch<F, G> = Vec<SensorReading<PointCloud<F, <G as GridAttributes>::Cloud>>>;

/// Reads [`RangeField`]s from a host sensor channel and publishes the matching
/// [`PointCloud`]s on another sensor channel: a flattened scan is still a
/// measurement, so consumers read it exactly as they would a host channel.
///
/// Generic over the frame and grid attributes so any organized sensor can use
/// it; the payload types are derived from `F` and `G` inside [`Self::new`], so
/// a caller cannot wire channels whose types disagree with the conversion.
pub(crate) struct DeprojectNode<F: Frame, G: GridAttributes> {
    name: Arc<str>,
    descriptor: PortDescriptor,
    input: ChannelKey,
    output: ChannelKey,
    /// Timestamp of the last batch converted; the bus is last-known-good, so
    /// without this every tick would re-publish the same batch.
    last_processed: AtomicF64,
    _data: PhantomData<fn() -> (F, G)>,
}

impl<F: Frame, G: GridAttributes> DeprojectNode<F, G> {
    /// Builds a node reading the host sensor channel `input` and writing the
    /// derived sensor channel `output`, both named by the agent profile.
    pub(crate) fn new(name: impl Into<Arc<str>>, input: &str, output: &str) -> Self {
        let input = SensorChannel::named::<FieldBatch<F, G>>(input);
        let output = SensorChannel::named::<CloudBatch<F, G>>(output);
        let descriptor = AlgorithmNodePortDescriptor::new()
            .input_sensor(input.clone())
            .output_sensor(output.clone())
            .build();

        Self {
            name: name.into(),
            descriptor,
            input: input.into(),
            output: output.into(),
            last_processed: AtomicF64::new(f64::NEG_INFINITY),
            _data: PhantomData,
        }
    }
}

impl<F: Frame, G: GridAttributes> PipelineNode for DeprojectNode<F, G> {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    /// Convert the latest batch once and publish it.
    ///
    /// The output keeps the *input's* timestamp and health, not this tick's,
    /// so a consumer that dedups by timestamp sees one batch per sensor scan
    /// rather than one per tick. A batch no newer than the last one converted
    /// is skipped, and an empty slot (cold start) publishes nothing. An empty
    /// batch is converted to an empty batch, so the output mirrors the input.
    fn execute(&self, bus: &PortBus, _tf: &dyn TfProvider, tick: TickContext) {
        let Some(fields) = bus.read::<FieldBatch<F, G>>(self.input.clone()) else {
            return;
        };

        let batch_ts = fields.timestamp.0;
        if batch_ts <= self.last_processed.load(Ordering::Relaxed) {
            return;
        }

        let clouds: CloudBatch<F, G> = fields
            .value
            .iter()
            .map(|reading| SensorReading {
                sensor: reading.sensor.clone(),
                timestamp: reading.timestamp,
                data: reading.data.to_point_cloud(),
            })
            .collect();

        let stamped = Stamped {
            value: clouds,
            timestamp: fields.timestamp,
            health: fields.health.clone(),
            producer: tick.node_id,
        };

        if let Err(ChannelError::UnknownChannel) = bus.write(self.output.clone(), stamped) {
            tracing::warn!(
                channel = %self.output,
                "deproject output channel is not wired into the DAG"
            );
        }

        self.last_processed.store(batch_ts, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    //! Wiring and bookkeeping tests for [`DeprojectNode`]. The conversion
    //! itself (`RangeField::to_point_cloud`) is tested in `helios_core`; here
    //! we check the node reads the sensor channel, keeps the input's stamp,
    //! converts each reading, and publishes each batch exactly once.

    use super::*;
    use crate::{Health, NodeId};

    use helios_core::{
        prelude::{DirectionModel, FrameId, RangeFieldBuilder, SphericalAngular, NO_INFORMATION},
        spatial::{
            conventions::Flu,
            primitives::{AgentId, MonotonicTime},
            transforms::ErasedTransform,
        },
    };

    const INPUT: &str = "sensor.lidar.front";
    const OUTPUT: &str = "lidar.front.points";
    const NODE_ID: NodeId = 7;
    const HOST_ID: NodeId = 1;
    const RANGE_MIN: f64 = 0.1;
    const RANGE_MAX: f64 = 10.0;
    const N_AZIMUTH: u32 = 4;
    const AZIMUTH_INCREMENT: f64 = std::f64::consts::FRAC_PI_2;

    type Node = DeprojectNode<Flu, ()>;

    struct NoTf;

    impl TfProvider for NoTf {
        fn get_transform(
            &self,
            _: FrameId,
            _: FrameId,
            _: MonotonicTime,
        ) -> Option<ErasedTransform> {
            None
        }
    }

    fn make_node() -> Node {
        Node::new("deproject", INPUT, OUTPUT)
    }

    fn input_key() -> ChannelKey {
        SensorChannel::named::<FieldBatch<Flu, ()>>(INPUT).into()
    }

    fn output_key() -> ChannelKey {
        SensorChannel::named::<CloudBatch<Flu, ()>>(OUTPUT).into()
    }

    fn bus_for(node: &Node) -> PortBus {
        PortBus::new(std::slice::from_ref(node.port_descriptor()))
    }

    fn sensor(name: &str) -> FrameId {
        FrameId::sensor(AgentId::new("bot"), name)
    }

    /// A planar four-beam scan: two returns, one cell with no information
    /// (`NaN`), and one left blank (nothing returned, `inf`). Only the two
    /// returns survive the conversion.
    fn field() -> RangeField<Flu> {
        let geometry = SphericalAngular::new(vec![0.0], 0.0, AZIMUTH_INCREMENT, N_AZIMUTH)
            .expect("valid test geometry");
        let mut builder = RangeFieldBuilder::<Flu>::new(
            DirectionModel::SphericalAngular(geometry),
            RANGE_MIN,
            RANGE_MAX,
        )
        .expect("valid test range limits");
        builder.set(0, 0, 1.0).expect("cell in bounds");
        builder.set(0, 1, 2.0).expect("cell in bounds");
        builder.set(0, 2, NO_INFORMATION).expect("cell in bounds");
        builder.finalize()
    }

    fn reading(name: &str, timestamp: f64) -> SensorReading<RangeField<Flu>> {
        SensorReading {
            sensor: sensor(name),
            timestamp: MonotonicTime(timestamp),
            data: field(),
        }
    }

    fn write_batch(bus: &PortBus, batch: FieldBatch<Flu, ()>, timestamp: f64, health: Health) {
        bus.write(
            input_key(),
            Stamped {
                value: batch,
                timestamp: MonotonicTime(timestamp),
                health,
                producer: HOST_ID,
            },
        )
        .unwrap();
    }

    fn tick_at(now: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt: 0.1,
            node_id: NODE_ID,
        }
    }

    fn read_output(bus: &PortBus) -> Option<Arc<Stamped<CloudBatch<Flu, ()>>>> {
        bus.read::<CloudBatch<Flu, ()>>(output_key())
    }

    #[test]
    fn descriptor_reads_and_writes_sensor_channels() {
        let node = make_node();
        let descriptor = node.port_descriptor();

        assert_eq!(descriptor.required_inputs, vec![input_key()]);
        assert_eq!(descriptor.outputs, vec![output_key()]);
    }

    #[test]
    fn empty_input_slot_publishes_nothing() {
        let node = make_node();
        let bus = bus_for(&node);

        node.execute(&bus, &NoTf, tick_at(1.0));

        assert!(read_output(&bus).is_none());
    }

    #[test]
    fn converts_each_reading_keeping_its_sensor_and_time() {
        let node = make_node();
        let bus = bus_for(&node);
        write_batch(
            &bus,
            vec![reading("lidar_front", 0.9), reading("lidar_rear", 0.95)],
            1.0,
            Health::Ok,
        );

        node.execute(&bus, &NoTf, tick_at(1.0));

        let out = read_output(&bus).unwrap();
        assert_eq!(out.value.len(), 2);
        assert_eq!(out.value[0].sensor, sensor("lidar_front"));
        assert_eq!(out.value[0].timestamp, MonotonicTime(0.9));
        assert_eq!(out.value[1].sensor, sensor("lidar_rear"));
        assert_eq!(out.value[1].timestamp, MonotonicTime(0.95));
    }

    #[test]
    fn misses_do_not_reach_the_cloud() {
        let node = make_node();
        let bus = bus_for(&node);
        write_batch(&bus, vec![reading("lidar_front", 1.0)], 1.0, Health::Ok);

        node.execute(&bus, &NoTf, tick_at(1.0));

        let out = read_output(&bus).unwrap();
        assert_eq!(out.value[0].data.len(), 2);
    }

    #[test]
    fn output_carries_the_input_stamp_and_health_not_the_tick() {
        let node = make_node();
        let bus = bus_for(&node);
        let health = Health::Degraded {
            reason: "partial scan".into(),
        };
        write_batch(&bus, vec![reading("lidar_front", 1.0)], 1.0, health);

        node.execute(&bus, &NoTf, tick_at(3.0));

        let out = read_output(&bus).unwrap();
        assert_eq!(out.timestamp, MonotonicTime(1.0));
        assert!(
            matches!(&out.health, Health::Degraded { reason } if reason == "partial scan"),
            "expected the input's health, got {:?}",
            out.health
        );
        assert_eq!(out.producer, NODE_ID);
    }

    #[test]
    fn empty_batch_is_published_as_an_empty_batch() {
        let node = make_node();
        let bus = bus_for(&node);
        write_batch(&bus, Vec::new(), 1.0, Health::Ok);

        node.execute(&bus, &NoTf, tick_at(1.0));

        assert!(read_output(&bus).unwrap().value.is_empty());
    }

    // The bus keeps the last batch, so the node sees it again next tick. A
    // sentinel written over the output between ticks must survive: if the node
    // re-published, the sentinel would be replaced.
    #[test]
    fn same_batch_is_published_once() {
        let node = make_node();
        let bus = bus_for(&node);
        write_batch(&bus, vec![reading("lidar_front", 1.0)], 1.0, Health::Ok);
        node.execute(&bus, &NoTf, tick_at(1.0));

        let sentinel_producer = HOST_ID;
        bus.write(
            output_key(),
            Stamped {
                value: CloudBatch::<Flu, ()>::new(),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: sentinel_producer,
            },
        )
        .unwrap();

        node.execute(&bus, &NoTf, tick_at(1.1));

        assert_eq!(read_output(&bus).unwrap().producer, sentinel_producer);
    }

    #[test]
    fn newer_batch_is_published() {
        let node = make_node();
        let bus = bus_for(&node);
        write_batch(&bus, vec![reading("lidar_front", 1.0)], 1.0, Health::Ok);
        node.execute(&bus, &NoTf, tick_at(1.0));

        write_batch(&bus, vec![reading("lidar_front", 2.0)], 2.0, Health::Ok);
        node.execute(&bus, &NoTf, tick_at(2.0));

        assert_eq!(read_output(&bus).unwrap().timestamp, MonotonicTime(2.0));
    }

    // A batch stamped at the very start of time is still new: the watermark
    // starts below any real timestamp.
    #[test]
    fn batch_at_time_zero_is_published() {
        let node = make_node();
        let bus = bus_for(&node);
        write_batch(&bus, vec![reading("lidar_front", 0.0)], 0.0, Health::Ok);

        node.execute(&bus, &NoTf, tick_at(0.0));

        assert!(read_output(&bus).is_some());
    }
}
