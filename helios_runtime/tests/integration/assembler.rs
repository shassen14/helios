// Assembler integration tests: build_pipeline topology resolution.

use std::collections::{BTreeMap, HashSet};

use helios_runtime::channels::{control, estimate};
use helios_runtime::port::{ChannelKey, InternalChannel, SensorChannel};
use helios_runtime::prelude::{Health, Stamped};
use helios_runtime::{
    build_pipeline, ActuatorSeamConfig, AutonomyRegistry, AutonomyStackConfig, BodyCapabilities,
    CommandFoldConfig, EstimateSeamConfig, PipelineAssemblyError, PipelineBuildError, Provenance,
    PublishedChannel, ReferenceSeamConfig,
};

use helios_core::control::actuation_model::{ActuationModel, ActuatorSpec, SignConvention};
use helios_core::control::actuators::{ActuatorCommand, ActuatorId, SetpointKind, SetpointValue};
use helios_core::control::commands::{DriveForce, SteerAngle, TwistIntent};
use helios_core::control::BodyTwistRef;
use helios_core::estimation::augmentation::MAGNETOMETER_BIAS;
use helios_core::estimation::carrier::kinematic_carrier_schema;
use helios_core::interchange::measurement::envelope::SensorReading;
use helios_core::interchange::measurement::sensor::MagneticField;
use helios_core::interchange::path::{Path, PlannerGoal};
use helios_core::interchange::perception::map::MapData;
use helios_core::prelude::{
    AgentId, DirectionModel, PointCloud, RangeField, RangeFieldBuilder, SphericalAngular,
};
use helios_core::spatial::conventions::Flu;
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::quantities::{FluVector, FreeVector};
use helios_core::spatial::state::{Component, Quantity};
use helios_core::spatial::{FrameAwareState, FrameId, StateVariable};

use nalgebra::Vector3;

use crate::common::MockRuntime;

/// A `[nodes]` entry for a teleop mapper named `teleop` with a car's tuning:
/// only surge and yaw are active.
fn twist_teleop() -> (String, toml::Table) {
    let mut section = toml::Table::new();
    section.insert("kind".to_string(), "TwistTeleop".into());
    section.insert("surge".to_string(), 4.0.into());
    section.insert("yaw".to_string(), 1.0.into());
    ("teleop".to_string(), section)
}

/// A `[reference]` section forwarding `base` unless a `preferred` member is
/// fresh.
fn reference_seam(base: &str, preferred: &[&str]) -> ReferenceSeamConfig {
    let mut section = toml::Table::new();
    section.insert("base".to_string(), base.into());
    section.insert(
        "preferred".to_string(),
        preferred
            .iter()
            .map(|name| toml::Value::from(*name))
            .collect::<Vec<_>>()
            .into(),
    );
    section.try_into().expect("a valid reference section")
}

/// An `[estimate]` section naming `source` as the authoritative estimator.
fn estimate_seam(source: &str) -> EstimateSeamConfig {
    EstimateSeamConfig {
        source: source.to_string(),
    }
}

/// A stack whose only node is the teleop mapper, forwarded onto the reference.
fn teleop_only_stack() -> AutonomyStackConfig {
    AutonomyStackConfig {
        nodes: BTreeMap::from([twist_teleop()]),
        reference: Some(reference_seam("teleop", &[])),
        ..Default::default()
    }
}

/// A host channel set holding the IMU channels the IMU-EKF fixtures in this
/// file predict from, plus `others`. The assembler rejects an estimator whose
/// predict inputs the host does not publish.
fn host_channels_with_imu(others: &[&str]) -> HashSet<String> {
    ["imu/accel", "imu/gyro"]
        .iter()
        .chain(others)
        .map(|name| name.to_string())
        .collect()
}

/// A body with no autonomy stack and no published channels.
fn teleop_body() -> BodyCapabilities {
    BodyCapabilities {
        name: "teleop_only".to_string(),
        publishes: vec![],
        ..Default::default()
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
// == Teleop-only: a lone teleop mapper is the reference seam's base       ==
// =========================================================================

#[test]
fn teleop_only_stack_builds_without_a_controller() {
    // No follower and no controllers: the teleop mapper is the seam's lone
    // member, forwarded onto `reference`. The stack must still build.
    let stack = teleop_only_stack();

    let result = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        teleop_body(),
    );

    assert!(
        result.is_ok(),
        "teleop-only stack must build, got: {:?}",
        result.err()
    );
}

#[test]
fn empty_or_dotted_agent_name_is_refused_alone() {
    // The agent name is one part of every watcher path, so `.` or an empty
    // name is refused before anything else is checked.
    for name in ["car.1", ""] {
        let result = build_pipeline(
            &teleop_only_stack(),
            &AutonomyRegistry::default(),
            AgentId::new(name),
            &HashSet::new(),
            teleop_body(),
        );
        let Err(errors) = result else {
            panic!("agent name {name:?} should be refused");
        };
        assert!(
            matches!(
                errors.as_slice(),
                [PipelineAssemblyError::MalformedAgentName { agent }] if agent == name
            ),
            "got {errors:?}"
        );
    }
}

#[test]
fn teleop_reference_is_intent_driven_not_free_running() {
    // The mapper is a brain node fed by host intent, not a free-running source:
    // with no intent on the bus the resolved `reference` slot stays empty, so
    // nothing is fabricated until the host actually publishes intent.
    let stack = teleop_only_stack();

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        teleop_body(),
    )
    .expect("teleop-only stack must build");

    let reference_key: ChannelKey = control::reference::<BodyTwistRef>().into();

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);
    assert!(
        pipeline.bus().read::<BodyTwistRef>(reference_key).is_none(),
        "no intent has been published, so the mapper must fabricate no reference"
    );
}

#[test]
fn teleop_intent_is_mapped_into_the_reference() {
    // The core teleop capability: the host publishes normalised intent, and the
    // mapper scales it into a `BodyTwistRef` that the seam forwards onto the
    // resolved `reference`, the same guidance reference a follower would emit.
    // Writing intent and ticking must make `reference` reflect the *scaled* twist,
    // proving the mapper is a real brain node fed by host intent.
    let stack = teleop_only_stack();

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
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

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

    let reference_key: ChannelKey = control::reference::<BodyTwistRef>().into();
    let reference = pipeline
        .bus()
        .read::<BodyTwistRef>(reference_key)
        .expect("the mapper must publish a reference derived from the intent");
    // surge 1.0 × the car's 4.0 surge scale, with the other five DOF at zero.
    assert_eq!(
        reference.value.twist().linear(),
        FluVector::new(4.0, 0.0, 0.0)
    );
    assert_eq!(
        reference.value.twist().angular(),
        FluVector::new(0.0, 0.0, 0.0)
    );
}

// =========================================================================
// == Node identity: every node is named by its config-map key, not its kind ==
// =========================================================================

#[test]
fn nodes_are_named_by_their_config_key_not_their_kind() {
    // Whereas each factory's unit test builds one node directly, this
    // exercises the whole assembler: `[nodes]` instantiation must thread the
    // table key through to the built node's identity. Two nodes
    // of a shared kind under distinct keys would otherwise collide on one name
    // in any name-keyed tooling (observability paths, `channels()`).
    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([occupancy_grid("local_grid", "scan"), imu_ekf("nav_ekf")]),
        estimate: Some(estimate_seam("nav_ekf")),
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["scan"]),
        body,
    )
    .expect("estimator + grid stack must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"nav_ekf"),
        "estimator node must carry its config key `nav_ekf`, got {names:?}"
    );
    assert!(
        names.contains(&"local_grid"),
        "grid node must carry its config key `local_grid`, got {names:?}"
    );
}

#[test]
fn two_grids_publish_to_distinct_channels() {
    // Two `OccupancyGrid2D` nodes under distinct keys must not collide on the
    // map producer slot. Each publishes `MapData` on a channel named by its own
    // key; a single hardcoded output name would make the second grid a
    // `DuplicateProducer` and fail the build.
    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([
            occupancy_grid("local", "scan/near"),
            occupancy_grid("global", "scan/far"),
            imu_ekf("nav_ekf"),
        ]),
        estimate: Some(estimate_seam("nav_ekf")),
        ..Default::default()
    };

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["scan/near", "scan/far"]),
        body,
    )
    .expect("two grids under distinct keys must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"local") && names.contains(&"global"),
        "both grid nodes must be present, got {names:?}"
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

/// A `[nodes]` entry named `name`, parsed from `section` written as TOML.
fn node_entry(name: &str, section: &str) -> (String, toml::Table) {
    let section: toml::Table = toml::from_str(section).expect("a valid node section");
    (name.to_string(), section)
}

/// A longitudinal speed feedback controller named `name`: emits DriveForce.
fn longitudinal_velocity(name: &str) -> (String, toml::Table) {
    node_entry(
        name,
        r#"
        kind = "LongitudinalVelocity"
        proportional_gain = 1.0
        integral_gain = 0.0
        derivative_gain = 0.0
        "#,
    )
}

/// A road-load feedforward controller named `name`: emits DriveForce.
fn road_load(name: &str) -> (String, toml::Table) {
    node_entry(name, "kind = \"RoadLoad\"\nc_roll = 0.01\nc_drag = 0.3")
}

/// The `[command.<fold>]` tables parsed from `section` written as TOML.
fn command_seam(section: &str) -> BTreeMap<String, CommandFoldConfig> {
    toml::from_str(section).expect("valid command folds")
}

/// A wheel-torque allocator named `name`, reading fold `input` and driving
/// actuator `drive`, with τ = F · 0.3.
fn wheel_torque(name: &str, input: &str, drive: &str) -> (String, toml::Table) {
    node_entry(
        name,
        &format!(
            "kind = \"WheelTorque\"\ninput = \"{input}\"\nwheel_radius = 0.3\ndrive = \"{drive}\""
        ),
    )
}

/// A steer-position allocator named `name`, reading fold `input` and driving
/// actuator `steer`.
fn steer_position(name: &str, input: &str, steer: &str) -> (String, toml::Table) {
    node_entry(
        name,
        &format!("kind = \"SteerPosition\"\ninput = \"{input}\"\nsteer = \"{steer}\""),
    )
}

/// An `[actuators]` section merging `members`.
fn actuator_seam(members: &[&str]) -> Option<ActuatorSeamConfig> {
    Some(ActuatorSeamConfig {
        members: members.iter().map(|member| member.to_string()).collect(),
    })
}

/// The body's actuators these tests drive: torque-driven `drive`, `left` and
/// `right`, and a position-driven `steer`.
fn test_actuation() -> ActuationModel {
    let spec = |id: &str, kind: SetpointKind| {
        ActuatorSpec::new(
            ActuatorId::new(id),
            kind,
            1.0e6,
            kind.value(0.0),
            SignConvention::Normal,
        )
    };
    ActuationModel::new(vec![
        spec("drive", SetpointKind::Torque),
        spec("left", SetpointKind::Torque),
        spec("right", SetpointKind::Torque),
        spec("steer", SetpointKind::Position),
    ])
}

/// A DriveForce stack: a feedback leg and a feedforward leg folded into a
/// wheel-torque allocator. The drive actuator id is `drive`; τ = F · r with
/// r = 0.3.
fn drive_force_stack() -> AutonomyStackConfig {
    let nodes = BTreeMap::from([
        longitudinal_velocity("speed_ctrl"),
        road_load("road_load"),
        wheel_torque("wheels", "drive_cmd", "drive"),
    ]);
    let command = command_seam(
        r#"
        [drive_cmd]
        type = "DriveForce"
        required = ["speed_ctrl"]
        optional = ["road_load"]
        "#,
    );

    AutonomyStackConfig {
        nodes,
        command,
        actuators: actuator_seam(&["wheels"]),
        ..Default::default()
    }
}

/// A body that advertises the estimate and the resolved `reference` on the
/// bus. The controllers read both, so a body publishing them satisfies the build
/// without an estimator, path follower or arbiter — keeping these tests focused
/// on the controller and allocator wiring. It has [`test_actuation`]'s
/// actuators.
fn state_and_reference_body() -> BodyCapabilities {
    BodyCapabilities {
        name: "oracle_state".to_string(),
        publishes: vec![
            PublishedChannel {
                key: estimate::estimate().into(),
                provenance: Provenance::Exact,
            },
            PublishedChannel {
                key: control::reference::<BodyTwistRef>().into(),
                provenance: Provenance::Exact,
            },
        ],
        actuation: test_actuation(),
    }
}

#[test]
fn drive_force_stack_builds_both_controllers_and_the_allocator() {
    // Both controllers, the drive fold and the wheel-torque allocator are
    // present, each under its own name.
    let pipeline = build_pipeline(
        &drive_force_stack(),
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    )
    .expect("DriveForce stack must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"drive_cmd"),
        "the command seam adds the drive fold, got {names:?}"
    );
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
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
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

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

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

/// A bicycle-steer feedforward controller named `name`: emits SteerAngle.
fn bicycle_steer(name: &str) -> (String, toml::Table) {
    node_entry(name, "kind = \"BicycleSteer\"\nwheelbase = 2.0")
}

/// A decoupled car stack: a DriveForce leg into a wheel-torque allocator (drive
/// actuator `drive`, τ = F · r, r = 0.3) and a SteerAngle leg into a steer-position
/// allocator (steer actuator `steer`, identity angle → position). The two
/// allocators own disjoint actuators, so a `Merge` unions their partials.
fn decoupled_car_stack() -> AutonomyStackConfig {
    let nodes = BTreeMap::from([
        longitudinal_velocity("speed_ctrl"),
        bicycle_steer("steer_ff"),
        wheel_torque("drive_wheels", "drive_cmd", "drive"),
        steer_position("steer_axle", "steer_cmd", "steer"),
    ]);
    let command = command_seam(
        r#"
        [drive_cmd]
        type = "DriveForce"
        required = ["speed_ctrl"]

        [steer_cmd]
        type = "SteerAngle"
        optional = ["steer_ff"]
        "#,
    );

    AutonomyStackConfig {
        nodes,
        command,
        actuators: actuator_seam(&["drive_wheels", "steer_axle"]),
        ..Default::default()
    }
}

#[test]
fn decoupled_stack_builds_both_spaces_and_both_allocators() {
    // Two command spaces coexist: the command seam adds one Sum per space, and
    // there is one allocator per space, each present under its config key.
    let pipeline = build_pipeline(
        &decoupled_car_stack(),
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    )
    .expect("decoupled two-allocator stack must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    for expected in [
        "speed_ctrl",
        "steer_ff",
        "drive_cmd",
        "steer_cmd",
        "drive_wheels",
        "steer_axle",
    ] {
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
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
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

    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

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
fn a_controller_of_the_wrong_type_fails_the_build_naming_it() {
    // The steer feedforward writes a `SteerAngle`, so listing it in the drive
    // fold is a mistake the command seam reports, naming the fold and member.
    let mut stack = decoupled_car_stack();
    stack.command = command_seam(
        r#"
        [drive_cmd]
        type = "DriveForce"
        required = ["speed_ctrl"]
        optional = ["steer_ff"]

        [steer_cmd]
        type = "SteerAngle"
        optional = ["steer_ff"]
        "#,
    );

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    ) else {
        panic!("a controller of the wrong type must not build");
    };

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::SeamMemberOutputMismatch {
                seam,
                member,
                ..
            } if seam == "command.drive_cmd" && member == "steer_ff"
        )),
        "expected SeamMemberOutputMismatch for `steer_ff`, got {errors:?}"
    );
}

#[test]
fn two_folds_of_one_type_drive_two_allocators() {
    // A skid-steer's two sides: two `DriveForce` folds, each read by its own
    // wheel-torque allocator. One fold per type could not express this.
    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([
            longitudinal_velocity("left_speed"),
            longitudinal_velocity("right_speed"),
            wheel_torque("left_wheels", "left_cmd", "left"),
            wheel_torque("right_wheels", "right_cmd", "right"),
        ]),
        command: command_seam(
            r#"
            [left_cmd]
            type = "DriveForce"
            required = ["left_speed"]

            [right_cmd]
            type = "DriveForce"
            required = ["right_speed"]
            "#,
        ),
        actuators: actuator_seam(&["left_wheels", "right_wheels"]),
        ..Default::default()
    };
    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    )
    .expect("two folds of one type must build");

    for (side, force) in [("left", 100.0), ("right", 40.0)] {
        pipeline
            .bus()
            .write(
                InternalChannel::named::<DriveForce>(format!("{side}_speed").as_str()).into(),
                Stamped {
                    value: DriveForce::new(force),
                    timestamp: MonotonicTime(0.0),
                    health: Health::Ok,
                    producer: 0,
                },
            )
            .expect("write to a side's contribution channel must succeed");
    }
    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

    let actuators = pipeline
        .read_actuators()
        .expect("both sides fold, allocate and merge");
    assert_eq!(
        setpoint_value(&actuators.value, "left"),
        SetpointValue::Torque(100.0 * 0.3)
    );
    assert_eq!(
        setpoint_value(&actuators.value, "right"),
        SetpointValue::Torque(40.0 * 0.3)
    );
}

#[test]
fn an_allocator_with_no_command_fold_fails_the_build_naming_its_input() {
    // The steer allocator reads `SteerAngle @ steer_cmd`. With no
    // `[command.steer_cmd]` nothing writes it, and the build names the
    // allocator and the channel.
    let mut stack = decoupled_car_stack();
    stack.command = command_seam("[drive_cmd]\ntype = \"DriveForce\"\nrequired = [\"speed_ctrl\"]");

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    ) else {
        panic!("an allocator with no command source must not build");
    };

    let steer_command: ChannelKey = InternalChannel::named::<SteerAngle>("steer_cmd").into();
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::PipelineBuild(v)
                if v.iter().any(|b| matches!(
                    b,
                    PipelineBuildError::UnsatisfiedInput { node_name, channel, .. }
                        if node_name == "steer_axle" && *channel == steer_command
                ))
        )),
        "expected UnsatisfiedInput for `steer_axle` reading the steer command, got {errors:?}"
    );
}

#[test]
fn an_actuator_the_body_lacks_fails_the_build_naming_the_body() {
    // The steer allocator drives `front_steer`, but the body names its steer
    // actuator `steer`: a typo the actuator seam catches at build.
    let mut stack = decoupled_car_stack();
    let (name, section) = steer_position("steer_axle", "steer_cmd", "front_steer");
    stack.nodes.insert(name, section);

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    ) else {
        panic!("an actuator the body lacks must not build");
    };

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::ActuatorNotOnBody { member, actuator, body }
                if member == "steer_axle" && actuator == "front_steer" && body == "oracle_state"
        )),
        "expected ActuatorNotOnBody for `steer_axle`, got {errors:?}"
    );
}

#[test]
fn a_setpoint_kind_the_body_does_not_accept_fails_the_build() {
    // A torque-driven allocator over the body's position-driven steer: the
    // wrong physical quantity, which nothing downstream could detect.
    let mut stack = decoupled_car_stack();
    let (name, section) = wheel_torque("drive_wheels", "drive_cmd", "steer");
    stack.nodes.insert(name, section);
    stack.actuators = actuator_seam(&["drive_wheels"]);

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    ) else {
        panic!("a setpoint kind the actuator does not accept must not build");
    };

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::ActuatorKindMismatch {
                member,
                writes: SetpointKind::Torque,
                accepts: SetpointKind::Position,
                ..
            } if member == "drive_wheels"
        )),
        "expected ActuatorKindMismatch for `drive_wheels`, got {errors:?}"
    );
}

#[test]
fn two_allocators_driving_one_actuator_fail_the_build_naming_both() {
    let mut stack = decoupled_car_stack();
    let (name, section) = steer_position("steer_axle", "steer_cmd", "drive");
    stack.nodes.insert(name, section);

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    ) else {
        panic!("two members driving one actuator must not build");
    };

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::ActuatorDrivenTwice { actuator, members }
                if actuator == "drive" && members.as_slice() == ["drive_wheels", "steer_axle"]
        )),
        "expected ActuatorDrivenTwice for `drive`, got {errors:?}"
    );
}

#[test]
fn an_allocator_outside_the_actuator_seam_does_not_reach_the_body() {
    // Only the listed members are merged. The steer allocator still runs, but
    // the terminal carries the drive setpoint alone.
    let mut stack = decoupled_car_stack();
    stack.actuators = actuator_seam(&["drive_wheels"]);
    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        state_and_reference_body(),
    )
    .expect("an unmerged allocator is not an error");

    pipeline
        .bus()
        .write(
            InternalChannel::named::<DriveForce>("speed_ctrl").into(),
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
            InternalChannel::named::<SteerAngle>("steer_ff").into(),
            Stamped {
                value: SteerAngle::new(0.2),
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("write to the steer contribution channel must succeed");
    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

    let actuators = pipeline
        .read_actuators()
        .expect("the drive member merges alone");
    assert_eq!(actuators.value.setpoints().len(), 1);
    assert_eq!(
        setpoint_value(&actuators.value, "drive"),
        SetpointValue::Torque(100.0 * 0.3)
    );
}

#[test]
fn planner_without_a_map_fails_the_build_naming_it() {
    // The planner reads `MapData` on the channel its `map_channel` names. With
    // no grid of that name nothing produces it, and the build names the
    // planner and the missing channel.
    let mut stack = planner_stack(&[("local_path", "mission")]);
    stack.nodes.remove("local");

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["scan"]),
        perception_body(),
    ) else {
        panic!("a planner with no map must not build");
    };

    let map_key: ChannelKey = InternalChannel::named::<MapData>("local").into();
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::PipelineBuild(v)
                if v.iter().any(|b| matches!(
                    b,
                    PipelineBuildError::UnsatisfiedInput { node_name, channel, .. }
                        if node_name == "local_path" && *channel == map_key
                ))
        )),
        "expected UnsatisfiedInput for `local_path` reading the `local` map, got {errors:?}"
    );
}

/// A `[nodes]` entry for a pure-pursuit follower named `pure_pursuit` reading
/// its path from `path`.
fn pure_pursuit_reading(path: &str) -> (String, toml::Table) {
    let mut section = toml::Table::new();
    section.insert("kind".to_string(), "PurePursuit".into());
    section.insert("path".to_string(), path.into());
    section.insert("max_speed_m_s".to_string(), 5.0.into());
    section.insert("min_speed_m_s".to_string(), 0.5.into());
    ("pure_pursuit".to_string(), section)
}

#[test]
fn follower_reads_the_path_of_the_planner_its_section_names() {
    let mut stack = planner_stack(&[("local_path", "mission")]);
    stack.nodes.extend([pure_pursuit_reading("local_path")]);

    let built = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["scan"]),
        perception_body(),
    );
    assert!(
        built.is_ok(),
        "a follower naming the planner must build, got {:?}",
        built.err()
    );
}

#[test]
fn follower_naming_no_planner_fails_the_build_naming_it() {
    // The follower reads `Path` on the channel its `path` names. With no
    // planner of that name nothing produces it, and the build names the
    // follower and the missing channel.
    let mut stack = planner_stack(&[("local_path", "mission")]);
    stack.nodes.extend([pure_pursuit_reading("global_path")]);

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["scan"]),
        perception_body(),
    ) else {
        panic!("a follower naming no planner must not build");
    };

    let path_key: ChannelKey = InternalChannel::named::<Path>("global_path").into();
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::PipelineBuild(v)
                if v.iter().any(|b| matches!(
                    b,
                    PipelineBuildError::UnsatisfiedInput { node_name, channel, .. }
                        if node_name == "pure_pursuit" && *channel == path_key
                ))
        )),
        "expected UnsatisfiedInput for `pure_pursuit` reading `global_path`, got {errors:?}"
    );
}

// =========================================================================
// == Reference seam: `[reference]` names the nodes that feed `reference`  ==
// =========================================================================

#[test]
fn teleop_preferred_over_a_follower_wins_the_reference_while_fresh() {
    // The follower is the base and teleop is preferred. The follower has no
    // path yet, so only the operator's intent reaches the seam; a fresh teleop
    // reference is what the selector forwards onto `reference`.
    let mut stack = planner_stack(&[("local_path", "mission")]);
    stack
        .nodes
        .extend([pure_pursuit_reading("local_path"), twist_teleop()]);
    stack.reference = Some(reference_seam("pure_pursuit", &["teleop"]));

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["scan"]),
        perception_body(),
    )
    .expect("a follower and teleop sharing the seam must build");

    let names: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    assert!(
        names.contains(&"reference_arbiter"),
        "the seam adds its selector, got {names:?}"
    );

    pipeline
        .bus()
        .write(
            control::intent::<TwistIntent>().into(),
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
    pipeline.tick(MonotonicTime(0.0), 0.1, &MockRuntime);

    let reference = pipeline
        .bus()
        .read::<BodyTwistRef>(control::reference::<BodyTwistRef>().into())
        .expect("the fresh teleop reference must be forwarded");
    assert_eq!(
        reference.value.twist().linear(),
        FluVector::new(4.0, 0.0, 0.0)
    );
}

#[test]
fn reference_naming_no_node_fails_the_build_naming_it() {
    let stack = AutonomyStackConfig {
        reference: Some(reference_seam("teleop", &["pure_pursuit"])),
        ..teleop_only_stack()
    };

    let Err(errors) = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        teleop_body(),
    ) else {
        panic!("a seam naming a missing node must not build");
    };

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::UnknownSeamMember { seam, member }
                if seam == "reference" && member == "pure_pursuit"
        )),
        "expected UnknownSeamMember for `pure_pursuit`, got {errors:?}"
    );
}

#[test]
fn controllers_without_a_reference_section_fail_naming_the_reference() {
    // Without `[reference]` nothing writes the reference, so each controller's
    // reference input is unsatisfied and the build names it.
    let body = BodyCapabilities {
        actuation: test_actuation(),
        ..perception_body()
    };
    let Err(errors) = build_pipeline(
        &drive_force_stack(),
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        body,
    ) else {
        panic!("controllers with no reference must not build");
    };

    let reference: ChannelKey = control::reference::<BodyTwistRef>().into();
    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::PipelineBuild(v)
                if v.iter().any(|b| matches!(
                    b,
                    PipelineBuildError::UnsatisfiedInput { channel, .. } if *channel == reference
                ))
        )),
        "expected an UnsatisfiedInput on the reference, got {errors:?}"
    );
}

// =========================================================================
// == Augmentation exit-proof: a config-declared mag-bias block converges   ==
// =========================================================================

#[test]
fn declared_mag_bias_augmentation_is_observed_end_to_end() {
    // The full augmentation path end-to-end through `build_pipeline`: an
    // estimator that declares a magnetometer aiding *and* a `magnetometer_bias`
    // augmentation is assembled, then driven with biased readings. This exercises
    // the correctness crux the mechanism exists for — the augmentation `sensor`
    // and the `MagneticFieldModel`'s sensor frame must resolve to the *same*
    // `FrameId`, because both are built as `FrameId::sensor(agent, channel_name)`
    // from the same channel name — otherwise the appended `MagBias` slots would
    // carry a `FrameId` the model never reads and the block would ride inert.
    // Here they agree, so the bias state absorbs the injected offset and its
    // variance collapses below the prior.
    let agent = AgentId::new("rover");
    const MAG_CHANNEL: &str = "mag/primary";

    // North-pointing world field (ENU, µT) and a purely vertical hard-iron bias.
    // A Z bias against a horizontal field is unconfounded with heading — no
    // rotation of a horizontal field yields a Z component — so it is cleanly
    // observable rather than smeared into an orientation error.
    let world_field = [0.0, 1.0, 0.0];
    let true_bias_z = 3.0;

    // The IMU channels are never published here, so the predict step is
    // skipped and the trajectory stays frozen: every mag residual flows into
    // the update. Orientation is pinned tightly so the residual lands on the
    // bias, not on a spurious tilt (belt-and-suspenders with the Z-bias choice
    // above). The augmentation's `sensor` must string-match the aiding
    // `input`: that shared key is what ties the block's sensor FrameId to the
    // model observing it.
    let section = format!(
        r#"{IMU_EKF}
        [dynamics.initial_uncertainty]
        orientation_deg = 1.0

        [aiding.mag]
        input = "{MAG_CHANNEL}"
        r_diag = [0.25, 0.25, 0.25]
        model = {{ kind = "magnetometer", magnetic_field_enu = {world_field:?} }}

        [[augmentation]]
        kind = "{MAGNETOMETER_BIAS}"
        sensor = "{MAG_CHANNEL}"
        init_uncertainty = 5.0
        random_walk = 0.01
        "#
    );
    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([node_entry("nav_ekf", &section)]),
        estimate: Some(estimate_seam("nav_ekf")),
        ..Default::default()
    };

    let sensor_channels = host_channels_with_imu(&[MAG_CHANNEL]);

    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        agent.clone(),
        &sensor_channels,
        body,
    )
    .expect("mag-bias augmentation stack must build");

    // Drive: publish the same biased reading each tick with a strictly
    // increasing timestamp. The node dedups by per-reading timestamp, so a
    // repeated stamp would be dropped and nothing would converge.
    let mag_key: ChannelKey =
        SensorChannel::named::<Vec<SensorReading<MagneticField>>>(MAG_CHANNEL).into();
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
                        sensor: FrameId::sensor(agent.clone(), MAG_CHANNEL),
                        timestamp: t,
                        data: MagneticField(measured),
                    }],
                    timestamp: t,
                    health: Health::Ok,
                    producer: 0,
                },
            )
            .expect("host write to the mag channel must succeed");
        pipeline.tick(MonotonicTime(0.0), dt, &MockRuntime);
    }

    let state = pipeline
        .read_state()
        .expect("the estimator must publish a state");
    let sensor = FrameId::sensor(agent.clone(), MAG_CHANNEL);

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
        .tangent_offset_of(&StateVariable::new(
            Quantity::MagBias(sensor.clone()),
            Component::X,
        ))
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

/// A one-ring, four-beam scan with two returns; the other two cells are blank
/// (nothing returned) and drop out when flattened.
fn two_return_field() -> RangeField<Flu> {
    let geometry = SphericalAngular::new(vec![0.0], 0.0, std::f64::consts::FRAC_PI_2, 4)
        .expect("valid test geometry");
    let mut builder =
        RangeFieldBuilder::<Flu>::new(DirectionModel::SphericalAngular(geometry), 0.1, 10.0)
            .expect("valid test range limits");
    builder.set(0, 0, 1.0).expect("cell in bounds");
    builder.set(0, 1, 2.0).expect("cell in bounds");
    builder.finalize()
}

#[test]
fn deproject_node_turns_host_range_fields_into_clouds() {
    // A stack with only a deproject entry must build on its own: the node's
    // input is a host sensor channel, so the assembler seeds it as external.
    // One tick later the host's field batch is on the derived cloud channel,
    // stamped with the batch's time rather than the tick's.
    let input = "sensor.lidar.front";
    let output = "lidar.front.points";

    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([deproject("front_deproject", input, output)]),
        ..Default::default()
    };
    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::from([input.to_string()]),
        body,
    )
    .expect("a lone deproject node over a host channel must build");

    let batch_time = MonotonicTime(0.5);
    let sensor = FrameId::sensor(AgentId::new("test_agent"), "lidar_front");
    pipeline
        .bus()
        .write(
            SensorChannel::named::<Vec<SensorReading<RangeField<Flu>>>>(input).into(),
            Stamped {
                value: vec![SensorReading {
                    sensor: sensor.clone(),
                    timestamp: batch_time,
                    data: two_return_field(),
                }],
                timestamp: batch_time,
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("the deproject input slot must exist");

    pipeline.tick(MonotonicTime(1.0), 0.1, &MockRuntime);

    let clouds = pipeline
        .bus()
        .read::<Vec<SensorReading<PointCloud<Flu>>>>(
            SensorChannel::named::<Vec<SensorReading<PointCloud<Flu>>>>(output).into(),
        )
        .expect("the deproject node must publish its cloud batch");
    assert_eq!(clouds.timestamp, batch_time);
    assert_eq!(clouds.value.len(), 1);
    assert_eq!(clouds.value[0].sensor, sensor);
    assert_eq!(clouds.value[0].data.len(), 2);
}

/// The section of an IMU-only EKF reading `imu/accel` and `imu/gyro`, as a
/// profile would write it. Tables can follow it.
const IMU_EKF: &str = r#"
    kind = "RecursiveEstimator"
    filter = { kind = "Ekf" }

    [dynamics]
    kind = "IntegratedImu"
    accel_channel = "imu/accel"
    gyro_channel = "imu/gyro"
    accel_noise_stddev = 0.1
    gyro_noise_stddev = 0.01
    accel_bias_instability = 0.001
    gyro_bias_instability = 0.0001
"#;

/// An IMU-only EKF named `name`: the minimum estimator a mapper's state input
/// needs.
fn imu_ekf(name: &str) -> (String, toml::Table) {
    node_entry(name, IMU_EKF)
}

/// Rate of the grids built by [`occupancy_grid`]. The node is rate-gated: it
/// fires only once a full period of tick `dt` has accumulated.
const MAPPER_RATE_HZ: f64 = 5.0;

/// A `[nodes]` entry for an `OccupancyGrid2D` named `name` reading `scan`, as
/// a profile would write it.
fn occupancy_grid(name: &str, scan: &str) -> (String, toml::Table) {
    let mut section = toml::Table::new();
    section.insert("kind".to_string(), "OccupancyGrid2D".into());
    section.insert("rate".to_string(), MAPPER_RATE_HZ.into());
    section.insert("resolution".to_string(), 0.1.into());
    section.insert("scan_channel".to_string(), scan.into());
    section.insert("width_m".to_string(), 10.0.into());
    section.insert("height_m".to_string(), 10.0.into());
    (name.to_string(), section)
}

/// A `[nodes]` entry for a `Deproject` named `name`, as a profile would write
/// it.
fn deproject(name: &str, input: &str, output: &str) -> (String, toml::Table) {
    let mut section = toml::Table::new();
    section.insert("kind".to_string(), "Deproject".into());
    section.insert("input".to_string(), input.into());
    section.insert("output".to_string(), output.into());
    (name.to_string(), section)
}

fn perception_body() -> BodyCapabilities {
    BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![],
        ..Default::default()
    }
}

#[test]
fn mapper_reads_a_deprojected_channel_the_host_does_not_publish() {
    // The host publishes only the range field. The mapper's scan channel is the
    // deproject node's output, a derived sensor channel, so it needs no host
    // publisher, and the producer is ordered ahead of the mapper.
    let stack = AutonomyStackConfig {
        estimate: Some(estimate_seam("nav_ekf")),
        nodes: BTreeMap::from([
            deproject("front_deproject", "lidar", "lidar.points"),
            occupancy_grid("local", "lidar.points"),
            imu_ekf("nav_ekf"),
        ]),
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["lidar"]),
        perception_body(),
    )
    .expect("a mapper reading a deprojected channel must build");

    let order: Vec<&str> = pipeline.channels().map(|(name, _)| name).collect();
    let position = |node: &str| {
        order
            .iter()
            .position(|name| *name == node)
            .unwrap_or_else(|| panic!("node `{node}` missing from {order:?}"))
    };
    assert!(
        position("front_deproject") < position("local"),
        "deproject must run before the mapper that reads it, got {order:?}"
    );
}

#[test]
fn mapper_reading_an_unpublished_channel_fails_the_build() {
    // A scan channel no one publishes used to build fine and leave the mapper
    // silently empty. It must now fail, naming the node and the channel.
    let stack = AutonomyStackConfig {
        estimate: Some(estimate_seam("nav_ekf")),
        nodes: BTreeMap::from([occupancy_grid("local", "lidar.typo"), imu_ekf("nav_ekf")]),
        ..Default::default()
    };

    let errors = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["lidar"]),
        perception_body(),
    )
    .err()
    .expect("a mapper reading an unpublished channel must not build");

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::UnpublishedSensorInput { node_name, channel }
                if node_name == "local" && channel == "lidar.typo"
        )),
        "expected UnpublishedSensorInput for `local` / `lidar.typo`, got {errors:?}"
    );
}

#[test]
fn mapper_builds_a_map_from_a_host_range_field() {
    // End to end through the real graph: the host publishes only a range
    // field, the deproject node flattens it, and the mapper integrates the
    // points. The mapper publishes nothing until it has integrated a scan, so
    // a map on the bus proves the points arrived. The robot state is supplied
    // by the body here so the test needs no estimator.
    let agent = AgentId::new("test_agent");
    let state_key: ChannelKey = estimate::estimate().into();
    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([
            deproject("front_deproject", "lidar", "lidar.points"),
            occupancy_grid("local", "lidar.points"),
        ]),
        ..Default::default()
    };
    let body = BodyCapabilities {
        name: "rover".to_string(),
        publishes: vec![PublishedChannel {
            key: state_key.clone(),
            provenance: Provenance::Exact,
        }],
        ..Default::default()
    };

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        agent.clone(),
        &HashSet::from(["lidar".to_string()]),
        body,
    )
    .expect("deproject feeding a mapper must build");

    let now = MonotonicTime(1.0);
    pipeline
        .bus()
        .write(
            state_key,
            Stamped {
                value: FrameAwareState::from_schema(
                    std::sync::Arc::new(kinematic_carrier_schema(agent.clone())),
                    now,
                ),
                timestamp: now,
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("the state slot must exist");
    pipeline
        .bus()
        .write(
            SensorChannel::named::<Vec<SensorReading<RangeField<Flu>>>>("lidar").into(),
            Stamped {
                value: vec![SensorReading {
                    sensor: FrameId::sensor(agent, "lidar"),
                    timestamp: now,
                    data: two_return_field(),
                }],
                timestamp: now,
                health: Health::Ok,
                producer: 0,
            },
        )
        .expect("the range-field slot must exist");

    // One tick spanning a full mapper period, so the rate-gated mapper fires.
    let mapper_period = 1.0 / MAPPER_RATE_HZ;
    pipeline.tick(now, mapper_period, &MockRuntime);

    assert!(
        pipeline
            .bus()
            .read::<MapData>(InternalChannel::named::<MapData>("local").into())
            .is_some(),
        "the mapper must publish a map once the deprojected scan reaches it"
    );
}

#[test]
fn estimator_predicting_from_an_unpublished_imu_fails_the_build() {
    // The IMU channels feed prediction, not aiding, but they are host inputs
    // all the same: an estimator naming one the host does not publish would
    // build, never receive a sample, and never run.
    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([imu_ekf("nav_ekf")]),
        estimate: Some(estimate_seam("nav_ekf")),
        ..Default::default()
    };

    let errors = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::from(["imu/gyro".to_string()]),
        perception_body(),
    )
    .err()
    .expect("an estimator without its accelerometer channel must not build");

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::UnpublishedSensorInput { node_name, channel }
                if node_name == "nav_ekf" && channel == "imu/accel"
        )),
        "expected UnpublishedSensorInput for `nav_ekf` / `imu/accel`, got {errors:?}"
    );
}

/// [`imu_ekf`] named `nav_ekf`, aided by a magnetometer reading
/// `mag_channel`.
fn mag_aided_imu_ekf(mag_channel: &str) -> (String, toml::Table) {
    let section = format!(
        r#"{IMU_EKF}
        [aiding.mag]
        input = "{mag_channel}"
        r_diag = [0.25, 0.25, 0.25]
        model = {{ kind = "magnetometer", magnetic_field_enu = [0.0, 1.0, 0.0] }}
        "#
    );
    node_entry("nav_ekf", &section)
}

#[test]
fn estimator_aided_by_a_host_channel_builds() {
    // Aiding inputs are optional ports. The build counts one as supplied only
    // because the generic sensor-input pass seeds optional inputs too; were it
    // to skip them, this build would fail with an unsatisfied input.
    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([mag_aided_imu_ekf("mag/primary")]),
        estimate: Some(estimate_seam("nav_ekf")),
        ..Default::default()
    };

    let result = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["mag/primary"]),
        perception_body(),
    );

    assert!(
        result.is_ok(),
        "expected the aided EKF to build, got {:?}",
        result.err()
    );
}

#[test]
fn estimator_aiding_from_an_unpublished_channel_fails_the_build() {
    // An aiding channel the host does not publish is caught by the sensor-input
    // check every node gets, named with the estimator node.
    let stack = AutonomyStackConfig {
        nodes: BTreeMap::from([mag_aided_imu_ekf("mag/typo")]),
        estimate: Some(estimate_seam("nav_ekf")),
        ..Default::default()
    };

    let errors = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["mag/primary"]),
        perception_body(),
    )
    .err()
    .expect("an estimator aided by an unpublished channel must not build");

    assert!(
        errors.iter().any(|e| matches!(
            e,
            PipelineAssemblyError::UnpublishedSensorInput { node_name, channel }
                if node_name == "nav_ekf" && channel == "mag/typo"
        )),
        "expected UnpublishedSensorInput for `nav_ekf` / `mag/typo`, got {errors:?}"
    );
}

// =========================================================================
// == Outside inputs: goals and teleop intent are declared apart from the body ==
// =========================================================================

/// A `[nodes]` entry for an A* planner named `name` over the `local` map,
/// reading its goal from `goal_channel`.
fn astar(name: &str, goal_channel: &str) -> (String, toml::Table) {
    let mut section = toml::Table::new();
    section.insert("kind".to_string(), "AStar".into());
    section.insert("rate".to_string(), 5.0.into());
    section.insert("map_channel".to_string(), "local".into());
    section.insert("goal_channel".to_string(), goal_channel.into());
    (name.to_string(), section)
}

/// An IMU EKF, a `local` occupancy grid, and one A* planner per
/// `(name, goal_channel)` pair: the smallest stack whose planners build.
fn planner_stack(planners: &[(&str, &str)]) -> AutonomyStackConfig {
    let mut nodes = BTreeMap::from([occupancy_grid("local", "scan"), imu_ekf("nav_ekf")]);
    nodes.extend(planners.iter().map(|(name, goal)| astar(name, goal)));
    AutonomyStackConfig {
        estimate: Some(estimate_seam("nav_ekf")),
        nodes,
        ..Default::default()
    }
}

#[test]
fn planner_goals_are_declared_once_per_goal_channel() {
    // Two planners share `mission`, a third reads `waypoints`. Each distinct
    // goal channel is declared exactly once, keyed as the planner reads it.
    let stack = planner_stack(&[
        ("local_path", "mission"),
        ("backup_path", "mission"),
        ("survey_path", "waypoints"),
    ]);

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["scan"]),
        perception_body(),
    )
    .expect("a stack of planners over a declared grid must build");

    let declared = pipeline.outside_inputs();
    let mission: ChannelKey = InternalChannel::named::<PlannerGoal>("mission").into();
    let waypoints: ChannelKey = InternalChannel::named::<PlannerGoal>("waypoints").into();

    assert_eq!(
        declared.len(),
        2,
        "two goal channels must give two outside inputs, got {declared:?}"
    );
    assert!(
        declared.contains(&mission),
        "mission goal missing from {declared:?}"
    );
    assert!(
        declared.contains(&waypoints),
        "waypoints goal missing from {declared:?}"
    );
}

#[test]
fn teleop_intent_is_declared_as_an_outside_input() {
    // The intent is something the operator sends, not a body measurement, so
    // the mapper's factory declares it as an outside input. A teleop-only stack
    // has no planner, so it is the only one.
    let stack = teleop_only_stack();

    let pipeline = build_pipeline(
        &stack,
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &HashSet::new(),
        teleop_body(),
    )
    .expect("teleop-only stack must build");

    let intent: ChannelKey = control::intent::<TwistIntent>().into();
    assert_eq!(pipeline.outside_inputs(), [intent].as_slice());
}

#[test]
fn stack_without_planner_or_teleop_declares_no_outside_inputs() {
    // An estimator and a mapper read only body channels and each other's
    // outputs; nothing is declared as coming from outside the robot.
    let pipeline = build_pipeline(
        &planner_stack(&[]),
        &AutonomyRegistry::default(),
        AgentId::new("test_agent"),
        &host_channels_with_imu(&["scan"]),
        perception_body(),
    )
    .expect("an estimator and a mapper must build");

    assert!(
        pipeline.outside_inputs().is_empty(),
        "no planner and no teleop must declare nothing, got {:?}",
        pipeline.outside_inputs()
    );
}
