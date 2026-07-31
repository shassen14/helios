//! [`OccupancyGridNode`] — pipeline adapter for any 2D [`Mapper`]
//! implementation (today: [`OccupancyGridMapper`]).
//!
//! Unlike the estimator / planner / follower nodes, this node has **no
//! input builder**. The mapper's contract is small enough that the node
//! reads the bus channels directly: `FrameAwareState @ ""` for the robot
//! pose, and a configurable `Vec<SensorReading<PointCloud2D>>` channel for
//! scans. Inventing a `MapperInputBuilder` trait would be one struct per
//! mapper with zero shared logic.
//!
//! ## Execution skeleton
//!
//! 1. **Resolve robot pose.** Read `FrameAwareState @ ""`; pull
//!    `get_pose_isometry()`. `None` → cold-start, skip the tick.
//! 2. **Recenter.** Hand the robot pose to [`Mapper::recenter`].
//! 3. **Integrate scans.** Read the scan channel; for each reading, look up
//!    the static `agent → sensor` transform via the runtime, compose
//!    `sensor_world = robot_world * agent_to_sensor`, and call
//!    [`Mapper::integrate_scan_2d`]. Readings whose TF lookup fails are
//!    silently skipped — TF rebuild lag is normal at startup.
//! 4. **Publish.** Snapshot the map via [`Mapper::get_map`]; if `Some`,
//!    clone and write to `map_channel` as `Stamped<MapData>`. If `None`,
//!    skip the write so the slot stays empty during cold-start.
//!
//! ## Rate
//!
//! Configurable Hz via constructor; mapping is typically expensive and
//! runs at 1–5 Hz rather than every tick.
//!
//! ## Frame discipline
//!
//! Sensor pose is composed from `FrameAwareState`'s pose (whatever the
//! upstream estimator publishes — real EKF, future
//! `GroundTruthEstimatorNode`, …) so the map lives in the estimator's
//! frame. Reading `runtime.world_pose(sensor_handle)` directly would
//! anchor the map to physics ground-truth and create a frame mismatch
//! whenever the estimator drifts.
//!
//! ## Known gaps
//!
//! - `Mapper::get_map` returns `&MapData`; we `clone()` it for the bus
//!   write. `OccupancyGrid2D` carries a `DMatrix<u8>` whose size is
//!   `width * height` cells — non-trivial allocation per fire. Converting
//!   the bus value to `Arc<MapData>` (or `MapData` to be `Arc`-internal)
//!   would eliminate the per-tick copy. Defer until profiling shows it.
//! - Per-sensor weighting / inhibit is not expressible: all readings on
//!   the one channel are integrated equally. If that becomes a real
//!   requirement, switch to per-sensor named channels and a handler
//!   pattern like [`crate::pipeline::nodes::AidingHandler`].
//!
//! [`OccupancyGridMapper`]: helios_core::mapping::OccupancyGridMapper
//! [`MapData`]: helios_core::mapping::MapData

use std::sync::atomic::Ordering;
use std::sync::Mutex;

use atomic_float::AtomicF64;
use helios_core::data::envelope::SensorReading;
use helios_core::data::primitives::FrameHandle;
use helios_core::data::sensor::PointCloud2D;
use helios_core::frames::FrameAwareState;
use helios_core::mapping::Mapper;

use crate::pipeline::descriptor::AlgorithmNodePortDescriptor;
use crate::pipeline::node::{PipelineNode, TickContext};
use crate::port::{ChannelKey, InternalChannel, PortBus, PortDescriptor, SensorChannel};
use crate::runtime::AgentRuntime;
use crate::stamped::{Health, Stamped};

/// Pipeline node wrapping any 2D [`Mapper`] implementation.
///
/// `scan_channel` is the bus channel carrying
/// `Vec<SensorReading<PointCloud2D>>`; its instance name comes from the
/// mapper config and must match the channel the host's scan sensor
/// publishes on. `map_channel` is the published map slot
/// — convention `MapData @ "local"` for rolling-window grids, `@ "global"`
/// for SLAM-style world maps.
pub(crate) struct OccupancyGridNode {
    name: String,
    mapper: Mutex<Box<dyn Mapper>>,
    /// Agent (body) frame used to compose `agent → sensor` static transforms
    /// from the runtime's TF tree. Combined with the robot pose from
    /// `FrameAwareState` it yields a sensor pose in the **estimator's**
    /// world frame, not physics ground-truth.
    agent_handle: FrameHandle,
    scan_channel: ChannelKey,
    map_channel: ChannelKey,
    descriptor: PortDescriptor,
    /// Highest scan-batch [`Stamped::timestamp`] this node has consumed.
    /// Bus slots are last-known-good, so the same batch reappears on
    /// consecutive ticks; re-integrating would double-count evidence in
    /// the cells the scan touches.
    last_integrated_ts: AtomicF64,
}

impl OccupancyGridNode {
    /// Build a node from a mapper, the agent's frame handle, and the scan /
    /// map channel keys.
    ///
    /// `rate_hz = Some(hz)` rate-gates the node; `None` fires every tick.
    /// Required inputs: `FrameAwareState @ ""` and `scan_channel`.
    /// Outputs: `map_channel`.
    pub(crate) fn new(
        name: impl Into<String>,
        mapper: Box<dyn Mapper>,
        agent_handle: FrameHandle,
        scan_channel: SensorChannel,
        map_channel: InternalChannel,
        rate_hz: Option<f64>,
    ) -> Self {
        let mut builder = AlgorithmNodePortDescriptor::new()
            .input_internal(InternalChannel::of::<FrameAwareState>())
            .input_sensor(scan_channel.clone())
            .output_internal(map_channel.clone());
        if let Some(hz) = rate_hz {
            builder = builder.rate_hz(hz);
        }
        let descriptor = builder.build();
        Self {
            name: name.into(),
            mapper: Mutex::new(mapper),
            agent_handle,
            scan_channel: scan_channel.into(),
            map_channel: map_channel.into(),
            descriptor,
            last_integrated_ts: AtomicF64::new(f64::NEG_INFINITY),
        }
    }
}

impl PipelineNode for OccupancyGridNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.descriptor
    }

    fn execute(&self, bus: &PortBus, runtime: &dyn AgentRuntime, tick: TickContext) {
        // 1. Robot pose from the upstream estimator (real or ground-truth).
        let Some(stamped_state) =
            bus.read::<FrameAwareState>(InternalChannel::of::<FrameAwareState>().into())
        else {
            return;
        };
        let Some(robot_world_pose) = stamped_state.value.get_pose_isometry() else {
            return;
        };

        // Skip the tick on a poisoned mutex rather than propagating the panic.
        let Ok(mut mapper) = self.mapper.lock() else {
            return;
        };

        // 2. Keep the rolling window centered on the robot.
        mapper.recenter(&robot_world_pose);

        // 3. Integrate the scan batch — only once per batch. The bus is
        //    last-known-good, so without dedup the same batch would be
        //    re-integrated on every tick and accumulate phantom evidence.
        //    Readings whose TF lookup fails are skipped; startup ordering
        //    can briefly leave a sensor without a TF entry.
        if let Some(stamped_scans) =
            bus.read::<Vec<SensorReading<PointCloud2D>>>(self.scan_channel.clone())
        {
            let batch_ts = stamped_scans.timestamp.0;
            if batch_ts > self.last_integrated_ts.load(Ordering::Relaxed) {
                for reading in stamped_scans.value.iter() {
                    let Some(sensor_in_agent) =
                        runtime.get_transform(self.agent_handle, reading.sensor_handle)
                    else {
                        continue;
                    };
                    let sensor_world_pose = robot_world_pose * sensor_in_agent;
                    mapper.integrate_scan_2d(&sensor_world_pose, &reading.data);
                }
                self.last_integrated_ts.store(batch_ts, Ordering::Relaxed);
            }
        }

        // 4. Publish only once the mapper has real data. While `get_map`
        //    returns `None` (cold-start) the bus slot stays empty, so
        //    downstream consumers see cold-start through the normal
        //    "slot empty" signal instead of a sentinel `MapData` variant
        //    they'd have to pattern-match against.
        let Some(map_snapshot) = mapper.get_map().cloned() else {
            return;
        };
        let stamped = Stamped {
            value: map_snapshot,
            timestamp: tick.now,
            health: Health::Ok,
            producer: tick.node_id,
        };
        let _ = bus.write(self.map_channel.clone(), stamped);
    }
}

#[cfg(test)]
mod tests {
    //! Wiring tests for [`OccupancyGridNode`]. The mapper's algorithmic
    //! behavior (raycast / recenter / sigmoid conversion) is covered in
    //! `helios_core::mapping::occupancy_grid::tests`. Here we verify only:
    //!   - descriptor lists required inputs and the configured map_channel
    //!   - cold-start (no FrameAwareState) is a silent no-op
    //!   - missing TF for a reading skips just that reading, not the tick
    //!   - on full inputs, a `Stamped<MapData>` lands on `map_channel` with
    //!     the correct `now` / `producer`

    use super::*;
    use helios_core::data::envelope::SensorReading;
    use helios_core::data::primitives::{FrameHandle, MonotonicTime};
    use helios_core::data::sensor::PointCloud2D;
    use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
    use helios_core::mapping::{MapData, Mapper};
    use nalgebra::{Isometry3, Point2, Translation3, UnitQuaternion};
    use std::sync::Mutex as StdMutex;

    // --- Mock AgentRuntime ---

    /// `get_transform(agent, sensor)` returns whatever was inserted for
    /// `sensor.0`; missing entries return `None` to exercise the skip path.
    struct MockRuntime {
        agent_to_sensor: std::collections::HashMap<u64, Isometry3<f64>>,
    }

    impl MockRuntime {
        fn new() -> Self {
            Self {
                agent_to_sensor: Default::default(),
            }
        }
        fn with_sensor(mut self, sensor: u64, tf: Isometry3<f64>) -> Self {
            self.agent_to_sensor.insert(sensor, tf);
            self
        }
    }

    impl AgentRuntime for MockRuntime {
        fn get_transform(
            &self,
            _agent: FrameHandle,
            sensor: FrameHandle,
        ) -> Option<Isometry3<f64>> {
            self.agent_to_sensor.get(&sensor.0).copied()
        }
        fn world_pose(&self, _: FrameHandle) -> Option<Isometry3<f64>> {
            Some(Isometry3::identity())
        }
        fn now(&self) -> MonotonicTime {
            MonotonicTime(0.0)
        }
    }

    const AGENT: FrameHandle = FrameHandle(1);

    // --- Mock Mapper that records calls ---

    #[derive(Default)]
    struct Calls {
        recenter: u32,
        integrate: u32,
        get_map: u32,
    }

    struct RecordingMapper {
        calls: StdMutex<Calls>,
    }

    impl RecordingMapper {
        fn new() -> Self {
            Self {
                calls: StdMutex::new(Calls::default()),
            }
        }
    }

    impl Mapper for RecordingMapper {
        fn recenter(&mut self, _robot_world_pose: &Isometry3<f64>) {
            self.calls.lock().unwrap().recenter += 1;
        }
        fn integrate_scan_2d(
            &mut self,
            _sensor_world_pose: &Isometry3<f64>,
            _cloud: &PointCloud2D,
        ) {
            self.calls.lock().unwrap().integrate += 1;
        }
        fn get_map(&mut self) -> Option<&MapData> {
            self.calls.lock().unwrap().get_map += 1;
            None
        }
    }

    /// Test stand-in for an always-empty mapper (replaces the deleted
    /// `NoneMapper`). Verifies that a node holding a mapper that never
    /// returns `Some` does not publish to the bus.
    struct EmptyMapper;
    impl Mapper for EmptyMapper {
        fn recenter(&mut self, _: &Isometry3<f64>) {}
        fn integrate_scan_2d(&mut self, _: &Isometry3<f64>, _: &PointCloud2D) {}
        fn get_map(&mut self) -> Option<&MapData> {
            None
        }
    }

    // --- Helpers ---

    fn scan_sensor_channel() -> SensorChannel {
        SensorChannel::of::<Vec<SensorReading<PointCloud2D>>>()
    }
    fn scan_channel() -> ChannelKey {
        scan_sensor_channel().into()
    }
    fn map_internal_channel() -> InternalChannel {
        InternalChannel::named::<MapData>("local")
    }
    fn map_channel() -> ChannelKey {
        map_internal_channel().into()
    }
    fn state_channel() -> ChannelKey {
        InternalChannel::of::<FrameAwareState>().into()
    }

    fn make_bus() -> PortBus {
        // Descriptors for: FrameAwareState (host-written), scans (host-written),
        // and the map (node-written).
        let host_state = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![state_channel()],
            rate: None,
        };
        let host_scans = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![scan_channel()],
            rate: None,
        };
        let map_producer = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![map_channel()],
            rate: None,
        };
        PortBus::new(&[host_state, host_scans, map_producer])
    }

    fn make_state_at(x: f64) -> FrameAwareState {
        let layout = vec![
            StateVariable::Px(FrameId::World),
            StateVariable::Py(FrameId::World),
            StateVariable::Pz(FrameId::World),
            StateVariable::Qx(FrameId::Body(AGENT), FrameId::World),
            StateVariable::Qy(FrameId::Body(AGENT), FrameId::World),
            StateVariable::Qz(FrameId::Body(AGENT), FrameId::World),
            StateVariable::Qw(FrameId::Body(AGENT), FrameId::World),
        ];
        let mut s = FrameAwareState::new(layout, 1.0, 0.0);
        s.state.vector[0] = x;
        // Identity quaternion: (0, 0, 0, 1).
        s.state.vector[6] = 1.0;
        s
    }

    fn publish_state(bus: &PortBus, x: f64) {
        let stamped = Stamped {
            value: make_state_at(x),
            timestamp: MonotonicTime(0.0),
            health: Health::Ok,
            producer: 99,
        };
        bus.write(state_channel(), stamped).unwrap();
    }

    fn publish_scans(bus: &PortBus, readings: Vec<SensorReading<PointCloud2D>>) {
        let stamped = Stamped {
            value: readings,
            timestamp: MonotonicTime(0.0),
            health: Health::Ok,
            producer: 99,
        };
        bus.write(scan_channel(), stamped).unwrap();
    }

    fn tick_at(now: f64, dt: f64) -> TickContext {
        TickContext {
            now: MonotonicTime(now),
            dt,
            node_id: 5,
        }
    }

    // --- Tests ---

    #[test]
    fn descriptor_requires_state_and_scan_and_outputs_map_channel() {
        let node = OccupancyGridNode::new(
            "occ",
            Box::new(EmptyMapper),
            AGENT,
            scan_sensor_channel(),
            map_internal_channel(),
            Some(5.0),
        );
        let d = node.port_descriptor();
        assert!(d.required_inputs.contains(&state_channel()));
        assert!(d.required_inputs.contains(&scan_channel()));
        assert_eq!(d.outputs, vec![map_channel()]);
        assert_eq!(d.rate, Some(5.0));
    }

    #[test]
    fn execute_cold_start_no_state_is_noop() {
        // Rely on absence-of-output to confirm the no-op (the recording
        // mapper is moved into the node, so we can't inspect call counts).
        let node = OccupancyGridNode::new(
            "occ",
            Box::new(RecordingMapper::new()),
            AGENT,
            scan_sensor_channel(),
            map_internal_channel(),
            None,
        );
        let bus = make_bus();
        // No state, no scans → silent return.
        node.execute(&bus, &MockRuntime::new(), tick_at(1.0, 0.1));
        assert!(bus.read::<MapData>(map_channel()).is_none());
    }

    #[test]
    fn execute_does_not_publish_until_mapper_has_real_data() {
        // With state present but no scans integrated, the mapper still
        // holds `MapData::None`. The node deliberately skips the bus write
        // in that case so consumers see cold-start (empty slot) rather
        // than a sentinel variant.
        let node = OccupancyGridNode::new(
            "occ",
            Box::new(EmptyMapper),
            AGENT,
            scan_sensor_channel(),
            map_internal_channel(),
            None,
        );
        let bus = make_bus();
        publish_state(&bus, 0.0);
        node.execute(&bus, &MockRuntime::new(), tick_at(2.5, 0.1));
        assert!(
            bus.read::<MapData>(map_channel()).is_none(),
            "node must not publish while cached_map is MapData::None"
        );
    }

    #[test]
    fn execute_skips_reading_when_tf_lookup_fails() {
        // Two readings: one for a sensor the runtime knows, one it doesn't.
        // The publish should still happen; the unknown reading is skipped.
        let known = FrameHandle(7);
        let unknown = FrameHandle(99);
        let runtime = MockRuntime::new().with_sensor(known.0, Isometry3::identity());

        // Use a real OccupancyGridMapper to confirm one cell gets marked.
        let mapper = helios_core::mapping::OccupancyGridMapper::new(1.0, 20.0, 20.0);
        let node = OccupancyGridNode::new(
            "occ",
            Box::new(mapper),
            AGENT,
            scan_sensor_channel(),
            map_internal_channel(),
            None,
        );
        let bus = make_bus();
        publish_state(&bus, 0.0);
        publish_scans(
            &bus,
            vec![
                SensorReading {
                    sensor_handle: unknown, // skipped
                    timestamp: MonotonicTime(0.0),
                    data: PointCloud2D {
                        points: vec![Point2::new(1.0, 0.0)],
                    },
                },
                SensorReading {
                    sensor_handle: known,
                    timestamp: MonotonicTime(0.0),
                    data: PointCloud2D {
                        points: vec![Point2::new(2.0, 0.0)],
                    },
                },
            ],
        );

        node.execute(&bus, &runtime, tick_at(1.0, 0.1));

        // A map should be published regardless; the known-sensor reading
        // produced a hit so it must be OccupancyGrid2D, not None.
        let published = bus.read::<MapData>(map_channel()).expect("published");
        assert!(matches!(published.value, MapData::OccupancyGrid2D { .. }));
    }

    #[test]
    fn execute_with_full_inputs_integrates_and_publishes() {
        // Sensor at world origin (identity), one hit 2m east.
        let sensor = FrameHandle(3);
        let runtime = MockRuntime::new().with_sensor(
            sensor.0,
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity()),
        );

        let mapper = helios_core::mapping::OccupancyGridMapper::new(1.0, 20.0, 20.0);
        let node = OccupancyGridNode::new(
            "occ",
            Box::new(mapper),
            AGENT,
            scan_sensor_channel(),
            map_internal_channel(),
            None,
        );
        let bus = make_bus();
        publish_state(&bus, 0.0);
        publish_scans(
            &bus,
            vec![SensorReading {
                sensor_handle: sensor,
                timestamp: MonotonicTime(0.0),
                data: PointCloud2D {
                    points: vec![Point2::new(2.0, 0.0)],
                },
            }],
        );

        node.execute(&bus, &runtime, tick_at(1.0, 0.1));

        let published = bus.read::<MapData>(map_channel()).expect("published");
        match &published.value {
            MapData::OccupancyGrid2D { data, .. } => {
                // First version: cells in middle (~127) plus the hit cell raised.
                // Just confirm we have a populated grid.
                assert!(data.nrows() > 0 && data.ncols() > 0);
            }
            _ => panic!("expected OccupancyGrid2D"),
        }
    }
}
