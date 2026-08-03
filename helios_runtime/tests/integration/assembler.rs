// Assembler integration tests: build_pipeline topology resolution.

use std::collections::HashMap;

use helios_runtime::channels::control;
use helios_runtime::config::{AutonomyStack, CommandArbitrationConfig, CommandSource};
use helios_runtime::port::ChannelKey;
use helios_runtime::prelude::{Health, Stamped};
use helios_runtime::{build_pipeline, AutonomyRegistry, BodyCapabilities};

use helios_core::control::commands::BodyTwist;
use helios_core::data::primitives::{FrameHandle, MonotonicTime};

use crate::common::MockRuntime;

/// A body with no autonomy stack, declaring only that it consumes control.
fn teleop_body() -> BodyCapabilities {
    BodyCapabilities {
        name: "teleop_only".to_string(),
        publishes: vec![],
        consumes_control: true,
    }
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
