// Assembler integration tests: build_pipeline topology resolution.

use std::collections::HashMap;

use helios_runtime::channels::control;
use helios_runtime::config::{
    AidingConfig, AllocatorConfig, AugmentationConfig, AutonomyStack, CommandArbitrationConfig,
    CommandSource, ControllerConfig, EkfConfig, EkfDynamicsConfig, EkfInitialStateConfig,
    EstimatorConfig, IntegratedImuConfig, MapLayerConfig, SearchPlannerConfig, SensorModelConfig,
    TeleopMapperConfig,
};
use helios_runtime::port::{ChannelKey, InternalChannel, SensorChannel};
use helios_runtime::prelude::{Health, Stamped};
use helios_runtime::{
    build_pipeline, AutonomyRegistry, BodyCapabilities, ConfigValidationError, PipelineAssemblyError,
    Provenance, PublishedChannel,
};

use helios_core::control::actuators::{ActuatorCommand, ActuatorId, SetpointValue};
use helios_core::control::commands::{BodyTwist, DriveForce, SteerAngle, TwistIntent};
use helios_core::data::envelope::SensorReading;
use helios_core::data::primitives::{FrameHandle, MonotonicTime};
use helios_core::data::sensor::MagneticField3D;
use helios_core::estimation::augmentation::MAGNETOMETER_BIAS;
use helios_core::frames::conventions::Flu;
use helios_core::frames::quantities::{FluVector, FreeVector};
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
use helios_core::state::{Component, Quantity};

use nalgebra::Vector3;

use crate::common::MockRuntime;

/// A car's teleop mapper tuning: only surge and yaw are active. Required on any
/// stack that declares a `Teleop` command source.
fn twist_teleop() -> TeleopMapperConfig {
    TeleopMapperConfig::Twist {
        surge: 4.0,
        sway: 0.0,
        heave: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 1.0,
    }
}

/// A body with no autonomy stack, declaring only that it consumes control.
fn teleop_body() -> BodyCapabilities {
    BodyCapabilities {
        name: "teleop_only".to_string(),
        publishes: vec![],
        consumes_control: true,
    }
}

/// The setpoint an [`ActuatorCommand`] carries for `id`, failing if absent.
fn setpoint_value(cmd: &ActuatorCommand, id: &str) -> SetpointValue {
    cmd.setpoints()
        .iter()
        .find(|sp| sp.actuator() == &ActuatorId::new(id))
        .expect("actuator present in command")
        .value()
        .clone()
}

// =========================================================================
// == Teleop-only: a single teleop source drives `command` with no autonomy ==
// =========================================================================

#[test]
fn teleop_only_stack_builds_without_a_controller() {
    // `sources = ["teleop"]` and no controllers: the assembler must resolve a
    // direct topology (no arbiter) in which the host publishes `command`
    // itself. Nothing in the brain produces commands.
    let stack = AutonomyStack {
        teleop: Some(twist_teleop()),
        command_arbitration: CommandArbitrationConfig {
            sources: vec![CommandSource::Teleop],
            ..Default::default()
        },
        ..Default::default()
    };

    let result = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        teleop_body(),
    );

    assert!(
        result.is_ok(),
        "teleop-only stack must build, got: {:?}",
        result.err()
    );
}

#[test]
fn teleop_only_command_is_host_published_not_brain_produced() {
    // The headline capability: with no autonomy controller, `read_control`
    // stays empty until the host writes `command`, then reflects that write.
    // This proves `command` is a real host-published slot and that nothing in
    // the brain fabricates a command.
    let stack = AutonomyStack {
        teleop: Some(twist_teleop()),
        command_arbitration: CommandArbitrationConfig {
            sources: vec![CommandSource::Teleop],
            ..Default::default()
        },
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        teleop_body(),
    )
    .expect("teleop-only stack must build");

    assert!(
        pipeline.read_control().is_none(),
        "no source has published yet, so there must be no command"
    );

    let command_key: ChannelKey = control::command::<BodyTwist>().into();
    pipeline
        .bus()
        .write(
            command_key,
            Stamped {
                value: BodyTwist::zero(),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("host write to `command` must succeed");

    pipeline.tick(&MockRuntime, 0.1);

    assert!(
        pipeline.read_control().is_some(),
        "read_control must reflect the host-published command"
    );
}

#[test]
fn teleop_intent_is_mapped_into_the_command() {
    // The commit-3 path proper: the host publishes normalised intent, and the
    // synthesized teleop mapper node scales it into a `BodyTwist` on `command`.
    // Writing intent and ticking must make `read_control` reflect the *scaled*
    // twist — proof the mapper is a real brain node fed by host intent, not a
    // host-published command.
    let stack = AutonomyStack {
        teleop: Some(twist_teleop()),
        command_arbitration: CommandArbitrationConfig {
            sources: vec![CommandSource::Teleop],
            ..Default::default()
        },
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        teleop_body(),
    )
    .expect("teleop stack with a mapper config must build");

    let intent_key: ChannelKey = control::intent::<TwistIntent>().into();
    pipeline
        .bus()
        .write(
            intent_key,
            Stamped {
                value: TwistIntent {
                    surge: 1.0,
                    ..TwistIntent::neutral()
                },
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("host write to `intent` must succeed");

    pipeline.tick(&MockRuntime, 0.1);

    let command = pipeline
        .read_control()
        .expect("the mapper must publish a command derived from the intent");
    // surge 1.0 × the car's 4.0 surge scale, with the other five DOF at zero.
    assert_eq!(command.value.linear(), FluVector::new(4.0, 0.0, 0.0));
    assert_eq!(command.value.angular(), FluVector::new(0.0, 0.0, 0.0));
}

// =========================================================================
// == Node identity: every node is named by its config-map key, not its kind ==
// =========================================================================

#[test]
fn nodes_are_named_by_their_config_key_not_their_kind() {
    // Whereas each factory's unit test injects `instance_name` directly, this
    // exercises the whole assembler: the estimator and map-layer loops must
    // thread the `HashMap` key through to the built node's identity. Two nodes
    // of a shared kind under distinct keys would otherwise collide on one name
    // in any name-keyed tooling (observability paths, `channels()`).
    let mut estimators = HashMap::new();
    estimators.insert(
        "nav_ekf".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(IntegratedImuConfig {
                gravity_enu: [0.0, 0.0, -9.81],
                accel_noise_stddev: 0.1,
                gyro_noise_stddev: 0.01,
                accel_bias_instability: 0.001,
                gyro_bias_instability: 0.0001,
                accel_bias_uncertainty_mps2: 0.1,
                gyro_bias_uncertainty_radps: 0.01,
                accel_channel: "imu/accel".to_string(),
                gyro_channel: "imu/gyro".to_string(),
            }),
            aiding: vec![],
            augmentation: vec![],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );

    let mut map_layers = HashMap::new();
    map_layers.insert(
        "local_grid".to_string(),
        MapLayerConfig::OccupancyGrid2D {
            rate: 5.0,
            resolution: 0.1,
            scan_channel: "scan".to_string(),
            width_m: 10.0,
            height_m: 10.0,
            pose_source: Default::default(),
        },
    );

    let stack = AutonomyStack {
        estimators,
        map_layers,
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        consumes_control: false,
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        body,
    )
    .expect("estimator + map-layer stack must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"nav_ekf"),
        "estimator node must carry its config key `nav_ekf`, got {names:?}"
    );
    assert!(
        names.contains(&"local_grid"),
        "map-layer node must carry its config key `local_grid`, got {names:?}"
    );
}

#[test]
fn two_map_layers_of_one_kind_publish_to_distinct_channels() {
    // Two `OccupancyGrid2D` layers under distinct keys must not collide on the
    // map producer slot. Each publishes `MapData` on a channel named by its own
    // config key; a single hardcoded output name would make the second layer a
    // `DuplicateProducer` and fail the build.
    let mut estimators = HashMap::new();
    estimators.insert(
        "nav_ekf".to_string(),
        EstimatorConfig::Ekf(EkfConfig {
            dynamics: EkfDynamicsConfig::IntegratedImu(IntegratedImuConfig {
                gravity_enu: [0.0, 0.0, -9.81],
                accel_noise_stddev: 0.1,
                gyro_noise_stddev: 0.01,
                accel_bias_instability: 0.001,
                gyro_bias_instability: 0.0001,
                accel_bias_uncertainty_mps2: 0.1,
                gyro_bias_uncertainty_radps: 0.01,
                accel_channel: "imu/accel".to_string(),
                gyro_channel: "imu/gyro".to_string(),
            }),
            aiding: vec![],
            augmentation: vec![],
            initial_state: EkfInitialStateConfig::default(),
        }),
    );

    let occupancy = |scan: &str| MapLayerConfig::OccupancyGrid2D {
        rate: 5.0,
        resolution: 0.1,
        scan_channel: scan.to_string(),
        width_m: 10.0,
        height_m: 10.0,
        pose_source: Default::default(),
    };
    let mut map_layers = HashMap::new();
    map_layers.insert("local".to_string(), occupancy("scan/near"));
    map_layers.insert("global".to_string(), occupancy("scan/far"));

    let stack = AutonomyStack {
        estimators,
        map_layers,
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        consumes_control: false,
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        body,
    )
    .expect("two same-kind map layers under distinct keys must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"local") && names.contains(&"global"),
        "both map-layer nodes must be present, got {names:?}"
    );
}

// =========================================================================
// == Allocator: converts the `command` terminal into the actuator terminal ==
// =========================================================================

#[test]
fn allocator_converts_command_into_the_actuator_terminal() {
    // A teleop source feeds `command`; an allocator sits downstream and converts
    // it into the `actuators` terminal. This exercises the whole seam through the
    // assembler: the allocator must be wired to *read* `command` and *write* the
    // terminal, and `read_actuators` must surface the result. It is the assembler
    // counterpart to the allocator node's own wiring tests.
    let mut allocators = HashMap::new();
    allocators.insert(
        "ackermann".to_string(),
        AllocatorConfig::KinematicAckermann {
            wheelbase: 2.0,
            wheel_radius: 0.3,
            drive: "drive_motor".to_string(),
            steer: "steer_servo".to_string(),
        },
    );

    let stack = AutonomyStack {
        allocators,
        teleop: Some(twist_teleop()),
        command_arbitration: CommandArbitrationConfig {
            sources: vec![CommandSource::Teleop],
            ..Default::default()
        },
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        teleop_body(),
    )
    .expect("allocator + teleop stack must build");

    // The node carries its config-map key, not its kind.
    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"ackermann"),
        "allocator node must be present under its config key, got {names:?}"
    );

    // Cold-start: nothing on `command`, so the terminal is empty.
    assert!(
        pipeline.read_actuators().is_none(),
        "no command published yet → no actuator terminal"
    );

    // Host publishes a straight-ahead twist (vx = 6 m/s, no yaw).
    let command_key: ChannelKey = control::command::<BodyTwist>().into();
    pipeline
        .bus()
        .write(
            command_key,
            Stamped {
                value: BodyTwist::new(FluVector::new(6.0, 0.0, 0.0), FluVector::zeros()),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("host write to `command` must succeed");

    pipeline.tick(&MockRuntime, 0.1);

    let actuators = pipeline
        .read_actuators()
        .expect("allocator must publish the actuator terminal after a command");

    // Bicycle inverse on the configured actuator ids: straight drive centers the
    // steer and scales wheel speed by vx / r. (The math itself is covered in
    // helios_core; here we only assert the command flows to the right ids.)
    assert_eq!(
        setpoint_value(&actuators.value, "steer_servo"),
        SetpointValue::Position(0.0)
    );
    assert_eq!(
        setpoint_value(&actuators.value, "drive_motor"),
        SetpointValue::Velocity(6.0 / 0.3)
    );
}

// =========================================================================
// == DriveForce command space: FB + FF fold into the wheel-torque terminal ==
// =========================================================================
// The command-space counterpart to the BodyTwist tests above. The allocator
// kind (WheelTorque) makes DriveForce the command space, so the assembler takes
// its DriveForce branch: each controller writes an instance-named contribution,
// a synthesized fold sums them into `command`, and the allocator converts that to
// the actuator terminal. These exercise that branch end-to-end through
// build_pipeline — the assembler counterpart to the Sum and WheelTorque unit
// tests.

/// A longitudinal speed feedback controller: emits DriveForce, folds as feedback.
fn longitudinal_velocity() -> ControllerConfig {
    ControllerConfig::LongitudinalVelocity {
        state_source: Default::default(),
        proportional_gain: 1.0,
        integral_gain: 0.0,
        derivative_gain: 0.0,
    }
}

/// A road-load feedforward controller: emits DriveForce, folds as feedforward.
fn road_load() -> ControllerConfig {
    ControllerConfig::RoadLoad {
        c_roll: 0.01,
        c_drag: 0.3,
    }
}

/// A DriveForce stack: a feedback leg and a feedforward leg folded into a
/// wheel-torque allocator. The drive actuator id is `drive`; τ = F · r with
/// r = 0.3.
fn drive_force_stack() -> AutonomyStack {
    let mut controllers = HashMap::new();
    controllers.insert("speed_ctrl".to_string(), longitudinal_velocity());
    controllers.insert("road_load".to_string(), road_load());

    let mut allocators = HashMap::new();
    allocators.insert(
        "wheels".to_string(),
        AllocatorConfig::WheelTorque {
            wheel_radius: 0.3,
            drive: "drive".to_string(),
        },
    );

    AutonomyStack {
        controllers,
        allocators,
        ..Default::default()
    }
}

/// A body that advertises `FrameAwareState` on the bus. The controllers require
/// state, so an oracle-style body publishing it satisfies the build without an
/// estimator node — keeping these tests focused on the DriveForce wiring.
fn state_publishing_body() -> BodyCapabilities {
    BodyCapabilities {
        name: "oracle_state".to_string(),
        publishes: vec![PublishedChannel {
            key: InternalChannel::of::<FrameAwareState>().into(),
            provenance: Provenance::Exact,
        }],
        consumes_control: true,
    }
}

#[test]
fn drive_force_stack_builds_both_controllers_and_the_allocator() {
    // Command-space dispatch on the allocator kind: both controllers and the
    // wheel-torque allocator are present under their config keys, each built by
    // its registry factory (the step-4 factories) through the DriveForce branch.
    let pipeline = build_pipeline(
        &drive_force_stack(),
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        state_publishing_body(),
    )
    .expect("DriveForce stack must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"speed_ctrl"),
        "feedback controller must be present under its config key, got {names:?}"
    );
    assert!(
        names.contains(&"road_load"),
        "feedforward controller must be present under its config key, got {names:?}"
    );
    assert!(
        names.contains(&"wheels"),
        "wheel-torque allocator must be present under its config key, got {names:?}"
    );

    // Cold-start: nothing folded yet, so the terminal is empty.
    assert!(
        pipeline.read_actuators().is_none(),
        "no contributions folded yet → no actuator terminal"
    );
}

#[test]
fn feedback_and_feedforward_fold_into_the_wheel_torque_terminal() {
    // The headline of the DriveForce branch: the two legs sum into `command` and
    // the allocator converts that force to a wheel torque. State is never written
    // to the bus, so the controller nodes cold-start and publish nothing; we
    // inject their contributions directly to test the fold + allocator wiring the
    // assembler built, independent of the control math (covered in helios_core).
    let pipeline = build_pipeline(
        &drive_force_stack(),
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        state_publishing_body(),
    )
    .expect("DriveForce stack must build");

    // Each controller writes an instance-named DriveForce contribution; those are
    // exactly the channels the synthesized fold reads.
    let feedback: ChannelKey = InternalChannel::named::<DriveForce>("speed_ctrl").into();
    let feedforward: ChannelKey = InternalChannel::named::<DriveForce>("road_load").into();
    pipeline
        .bus()
        .write(
            feedback,
            Stamped {
                value: DriveForce::new(100.0),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("write to the feedback contribution channel must succeed");
    pipeline
        .bus()
        .write(
            feedforward,
            Stamped {
                value: DriveForce::new(50.0),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("write to the feedforward contribution channel must succeed");

    pipeline.tick(&MockRuntime, 0.1);

    let actuators = pipeline
        .read_actuators()
        .expect("the fold → allocator must publish the terminal after contributions");

    // τ = (100 + 50) N · 0.3 m = 45 N·m on the drive actuator. The sum is folded
    // in the pipeline, the radius scaling in the allocator; asserting the same
    // arithmetic the code runs keeps the float comparison exact.
    assert_eq!(
        setpoint_value(&actuators.value, "drive"),
        SetpointValue::Torque((100.0 + 50.0) * 0.3)
    );
}

// =========================================================================
// == Decoupled: two allocators, two command spaces, one merged terminal ==
// =========================================================================
// The E3 headline. A car's longitudinal and lateral degrees of freedom are
// separate seams: a DriveForce leg folds into a wheel-torque allocator, a
// SteerAngle leg into a steer-position allocator, and one `Merge` unions the two
// partial commands into the single actuator terminal. This is what the old
// single-space assembler (and the >1-allocator validation ban) could not build.

/// A bicycle-steer feedforward controller: emits SteerAngle, folds as feedforward.
fn bicycle_steer() -> ControllerConfig {
    ControllerConfig::BicycleSteer { wheelbase: 2.0 }
}

/// A decoupled car stack: a DriveForce leg into a wheel-torque allocator (drive
/// actuator `drive`, τ = F · r, r = 0.3) and a SteerAngle leg into a steer-position
/// allocator (steer actuator `steer`, identity angle → position). The two
/// allocators own disjoint actuators, so a `Merge` unions their partials.
fn decoupled_car_stack() -> AutonomyStack {
    let mut controllers = HashMap::new();
    controllers.insert("speed_ctrl".to_string(), longitudinal_velocity());
    controllers.insert("steer_ff".to_string(), bicycle_steer());

    let mut allocators = HashMap::new();
    allocators.insert(
        "drive_wheels".to_string(),
        AllocatorConfig::WheelTorque {
            wheel_radius: 0.3,
            drive: "drive".to_string(),
        },
    );
    allocators.insert(
        "steer_axle".to_string(),
        AllocatorConfig::SteerPosition {
            steer: "steer".to_string(),
        },
    );

    AutonomyStack {
        controllers,
        allocators,
        ..Default::default()
    }
}

#[test]
fn decoupled_stack_builds_both_spaces_and_both_allocators() {
    // Two command spaces coexist: the assembler wires one Sum per space and one
    // allocator per space, each present under its config key. What the old
    // single-space assembler could not express.
    let pipeline = build_pipeline(
        &decoupled_car_stack(),
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        state_publishing_body(),
    )
    .expect("decoupled two-allocator stack must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    for expected in ["speed_ctrl", "steer_ff", "drive_wheels", "steer_axle"] {
        assert!(
            names.contains(&expected),
            "`{expected}` must be present under its config key, got {names:?}"
        );
    }

    // Cold-start: neither space has folded, so the merged terminal is empty.
    assert!(
        pipeline.read_actuators().is_none(),
        "no contributions folded yet → no actuator terminal"
    );
}

#[test]
fn decoupled_legs_merge_into_one_actuator_terminal() {
    // The headline: a DriveForce contribution and a SteerAngle contribution flow
    // through their own folds and allocators, and the merge unions the two partial
    // commands into one terminal carrying *both* actuators. State is never written,
    // so the controllers cold-start and publish nothing; we inject their
    // contributions directly to test the fold → allocator → merge wiring the
    // assembler built, independent of the control math (covered in helios_core).
    let pipeline = build_pipeline(
        &decoupled_car_stack(),
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        state_publishing_body(),
    )
    .expect("decoupled two-allocator stack must build");

    // Each leg's instance-named contribution — exactly the channels its space's
    // fold reads.
    let drive_leg: ChannelKey = InternalChannel::named::<DriveForce>("speed_ctrl").into();
    let steer_leg: ChannelKey = InternalChannel::named::<SteerAngle>("steer_ff").into();
    pipeline
        .bus()
        .write(
            drive_leg,
            Stamped {
                value: DriveForce::new(100.0),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("write to the drive contribution channel must succeed");
    pipeline
        .bus()
        .write(
            steer_leg,
            Stamped {
                value: SteerAngle::new(0.2),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("write to the steer contribution channel must succeed");

    pipeline.tick(&MockRuntime, 0.1);

    let actuators = pipeline
        .read_actuators()
        .expect("the two folds → allocators → merge must publish the terminal");

    // Both actuators are present in the one merged command: τ = 100 N · 0.3 m on
    // `drive`, and the identity steer lift → Position(0.2) on `steer`.
    assert_eq!(
        setpoint_value(&actuators.value, "drive"),
        SetpointValue::Torque(100.0 * 0.3)
    );
    assert_eq!(
        setpoint_value(&actuators.value, "steer"),
        SetpointValue::Position(0.2)
    );
}

#[test]
fn build_pipeline_rejects_invalid_config_before_assembly() {
    // A planner reading map level "local" with no such layer declared is a
    // config error. `build_pipeline` runs static validation first, so this
    // surfaces as `InvalidConfig` up front rather than as a downstream
    // `UnsatisfiedInput` on the missing `MapData` producer.
    let mut search_planners = HashMap::new();
    search_planners.insert(
        "local_path".to_string(),
        SearchPlannerConfig::AStar {
            rate: 5.0,
            arrival_tolerance_m: 1.5,
            occupancy_threshold: 180,
            max_search_depth: 20_000,
            enable_path_smoothing: false,
            replan_on_path_deviation: false,
            deviation_tolerance_m: 3.0,
            level: "local".to_string(),
            goal_channel: "mission".to_string(),
        },
    );
    let stack = AutonomyStack {
        search_planners,
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        consumes_control: false,
    };

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        body,
    ) else {
        panic!("a planner with no matching map layer must not build");
    };

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::InvalidConfig(v)
                if v.iter().any(|c| matches!(
                    c,
                    ConfigValidationError::PlannerReferencesUnknownMapLayer { level, .. }
                        if level == "local"
                ))
        )),
        "expected InvalidConfig carrying PlannerReferencesUnknownMapLayer, got {errors:?}"
    );
}

// =========================================================================
// == Augmentation exit-proof: a config-declared mag-bias block converges   ==
// =========================================================================

#[test]
fn declared_mag_bias_augmentation_is_observed_end_to_end() {
    // The full augmentation path end-to-end through `build_pipeline`: an
    // `EkfConfig` that declares a magnetometer aiding *and* a `magnetometer_bias`
    // augmentation is assembled, then driven with biased readings. This exercises
    // the correctness crux the mechanism exists for — the augmentation `sensor`
    // and the `MagneticFieldModel`'s `sensor_handle` must resolve to the *same*
    // `FrameHandle` through `sensor_frame_handles`, or the appended `MagBias`
    // slots carry a `FrameId` the model never reads and the block rides inert.
    // Here they agree, so the bias state absorbs the injected offset and its
    // variance collapses below the prior.
    const AGENT: FrameHandle = FrameHandle(0);
    const MAG_SENSOR: FrameHandle = FrameHandle(7);
    const MAG_CHANNEL: &str = "mag/primary";

    // North-pointing world field (ENU, µT) and a purely vertical hard-iron bias.
    // A Z bias against a horizontal field is unconfounded with heading — no
    // rotation of a horizontal field yields a Z component — so it is cleanly
    // observable rather than smeared into an orientation error.
    let world_field = [0.0, 1.0, 0.0];
    let true_bias_z = 3.0;

    let ekf = EkfConfig {
        dynamics: EkfDynamicsConfig::IntegratedImu(IntegratedImuConfig {
            gravity_enu: [0.0, 0.0, -9.81],
            accel_noise_stddev: 0.1,
            gyro_noise_stddev: 0.01,
            accel_bias_instability: 0.001,
            gyro_bias_instability: 0.0001,
            accel_bias_uncertainty_mps2: 0.1,
            gyro_bias_uncertainty_radps: 0.01,
            // Never published here, so the predict step is skipped and the
            // trajectory stays frozen — every mag residual flows into the update.
            accel_channel: "imu/accel".to_string(),
            gyro_channel: "imu/gyro".to_string(),
        }),
        aiding: vec![AidingConfig {
            sensor_payload: "MagneticField3D".to_string(),
            model: SensorModelConfig {
                kind: "magnetometer".to_string(),
                gravity_enu: [0.0, 0.0, -9.81],
                magnetic_field_enu: Some(world_field),
            },
            input_channel: MAG_CHANNEL.to_string(),
            r_diag: vec![0.25, 0.25, 0.25],
        }],
        augmentation: vec![AugmentationConfig {
            kind: MAGNETOMETER_BIAS.to_string(),
            // Must string-match the aiding input_channel above: that shared key
            // is what ties the block's FrameId::Sensor to the model observing it.
            sensor: MAG_CHANNEL.to_string(),
            init_uncertainty: 5.0,
            random_walk: 0.01,
        }],
        initial_state: EkfInitialStateConfig {
            // Pin orientation tightly so the residual lands on the bias, not on a
            // spurious tilt (belt-and-suspenders with the Z-bias choice above).
            orientation_uncertainty_deg: 1.0,
            ..Default::default()
        },
    };

    let mut estimators = HashMap::new();
    estimators.insert("nav_ekf".to_string(), EstimatorConfig::Ekf(ekf));
    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };

    let mut sensor_frame_handles = HashMap::new();
    sensor_frame_handles.insert(MAG_CHANNEL.to_string(), MAG_SENSOR);

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        consumes_control: false,
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AGENT,
        &sensor_frame_handles,
        body,
    )
    .expect("mag-bias augmentation stack must build");

    // Drive: publish the same biased reading each tick with a strictly
    // increasing timestamp. The node dedups by per-reading timestamp, so a
    // repeated stamp would be dropped and nothing would converge.
    let mag_key: ChannelKey =
        SensorChannel::named::<Vec<SensorReading<MagneticField3D>>>(MAG_CHANNEL).into();
    let measured = Vector3::new(world_field[0], world_field[1], world_field[2] + true_bias_z);
    let dt = 0.05;
    for i in 1..=40 {
        let t = MonotonicTime(i as f64 * dt);
        pipeline
            .bus()
            .write(
                mag_key.clone(),
                Stamped {
                    value: vec![SensorReading {
                        sensor_handle: MAG_SENSOR,
                        timestamp: t,
                        data: MagneticField3D { value: measured },
                    }],
                    timestamp: t,
                    health: Health::Ok,
                    producer: 0,
                },
            )
            .expect("host write to the mag channel must succeed");
        pipeline.tick(&MockRuntime, dt);
    }

    let state = pipeline
        .read_state()
        .expect("the estimator must publish a state");
    let sensor = FrameId::Sensor(MAG_SENSOR);

    // The appended block exists and the base grew by exactly 3 storage dims.
    assert_eq!(
        state.value.schema().storage_dim(),
        19,
        "16-state INS base + 3-DOF mag-bias block"
    );
    // The covariance is tangent-indexed, so the bias block's variance is found at
    // its *tangent* offset (which sits below the storage offset, the SO(3) block
    // spending one fewer tangent DOF than stored component).
    let tangent_off = state
        .value
        .schema()
        .tangent_offset_of(&StateVariable::new(Quantity::MagBias(sensor.clone()), Component::X))
        .expect("the mag-bias block must be present in the published schema");

    // Mean moved from 0 toward the injected +3 µT offset.
    let bias = state
        .value
        .mag_bias::<Flu>(sensor)
        .map(FreeVector::into_inner)
        .expect("the bias block reads back as a vector");
    assert!(
        (bias.z - true_bias_z).abs() < 0.5,
        "estimated bias_z {} must converge toward the true {true_bias_z} µT",
        bias.z
    );

    // Variance collapsed well below the 5 µT prior (25 µT²): the block was
    // genuinely updated, not merely carried through predict.
    let var_z = state.value.covariance[(tangent_off + 2, tangent_off + 2)];
    assert!(
        var_z < 1.0,
        "bias_z variance {var_z} must shrink below the prior 25 µT²"
    );
}

#[test]
fn no_declared_augmentation_leaves_the_base_schema_unchanged() {
    // The dual of the exit-proof: an EKF with no augmentation block must publish
    // the bare 16-state INS shape with no MagBias slots — the build_pipeline-level
    // echo of the builder-level guarantee in register.rs. This freezes the "empty
    // augmentation ⇒ base filter shape unchanged" contract at the config front
    // door.
    let ekf = EkfConfig {
        dynamics: EkfDynamicsConfig::IntegratedImu(IntegratedImuConfig {
            gravity_enu: [0.0, 0.0, -9.81],
            accel_noise_stddev: 0.1,
            gyro_noise_stddev: 0.01,
            accel_bias_instability: 0.001,
            gyro_bias_instability: 0.0001,
            accel_bias_uncertainty_mps2: 0.1,
            gyro_bias_uncertainty_radps: 0.01,
            accel_channel: "imu/accel".to_string(),
            gyro_channel: "imu/gyro".to_string(),
        }),
        aiding: vec![],
        augmentation: vec![],
        initial_state: EkfInitialStateConfig::default(),
    };

    let mut estimators = HashMap::new();
    estimators.insert("nav_ekf".to_string(), EstimatorConfig::Ekf(ekf));
    let stack = AutonomyStack {
        estimators,
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        consumes_control: false,
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        FrameHandle(0),
        &HashMap::new(),
        body,
    )
    .expect("un-augmented stack must build");

    pipeline.tick(&MockRuntime, 0.05);

    let state = pipeline
        .read_state()
        .expect("the estimator must publish a state");
    assert_eq!(
        state.value.schema().storage_dim(),
        16,
        "bare INS base, no augmentation"
    );
    assert!(
        state
            .value
            .schema()
            .storage_offset_of(&StateVariable::new(
                Quantity::MagBias(FrameId::Sensor(FrameHandle(7))),
                Component::X,
            ))
            .is_none(),
        "no augmentation ⇒ no MagBias slots"
    );
}
