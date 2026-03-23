// helios_sim/src/simulation/plugins/debugging/ui/vehicle_hud.rs
//
// Compact bottom-left HUD showing real-time vehicle control state:
//   - Desired speed (from ControlOutputComponent)
//   - GT and estimated speeds with [CTRL] marker on active source
//   - Horizontal speed bar (fills to active speed / max_speed)
//   - Steering wheel (spoke rotates with steering_torque_norm)
//   - Throttle gauge (fills from bottom, green = fwd, red = rev/brake)
//
// Toggle with C; T switches ControllerStateSource when both control + estimation are active.

use bevy::prelude::*;

use helios_core::control::ControlOutput;
use helios_core::frames::{FrameId, StateVariable};

use crate::simulation::core::components::{
    ControlOutputComponent, ControllerStateSource, GroundTruthState,
};
use crate::simulation::plugins::autonomy::EstimatorComponent;
use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;
use crate::simulation::plugins::vehicles::ackermann::components::{
    AckermannActuator, AckermannCommand,
};
use crate::simulation::profile::CapabilitySet;

/// Entity IDs for nodes that `update_vehicle_hud` mutates each frame.
/// Inserted as a Bevy resource by `spawn_vehicle_hud`; absent when the HUD was not spawned.
#[derive(Resource)]
pub struct VehicleHudEntities {
    pub root: Entity,
    pub speed_fill: Entity,
    pub gt_speed_text: Entity,
    pub est_speed_text: Entity,
    pub desired_speed_text: Entity,
    pub throttle_fill: Entity,
    pub throttle_text: Entity,
    pub steering_spoke: Entity,
    pub steering_text: Entity,
}

// ─────────────────────────────────────────────────────────────────────────────
// Spawn
// ─────────────────────────────────────────────────────────────────────────────

/// Builds the full HUD node tree and inserts `VehicleHudEntities`.
/// Called once on `OnEnter(Running)`; no-ops if neither control nor estimation is active.
pub fn spawn_vehicle_hud(
    mut commands: Commands,
    capabilities: Res<CapabilitySet>,
    config: Res<DebugVisualizationConfig>,
) {
    if !capabilities.control() && !capabilities.estimation() {
        return;
    }

    let initial_vis = if config.show_vehicle_hud {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };

    // ── Speed section ────────────────────────────────────────────────────────

    let speed_fill = commands
        .spawn((
            Node {
                width: Val::Percent(0.0),
                height: Val::Percent(100.0),
                ..default()
            },
            BackgroundColor(Color::srgb(0.2, 0.75, 0.2)),
        ))
        .id();

    let speed_bar = commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Px(10.0),
                ..default()
            },
            BackgroundColor(Color::srgb(0.2, 0.2, 0.2)),
        ))
        .id();
    commands.entity(speed_bar).add_child(speed_fill);

    let desired_speed_text = commands
        .spawn((
            Node::default(),
            Text::new("Des: --"),
            TextFont {
                font_size: 12.0,
                ..default()
            },
            TextColor(Color::srgb(0.8, 0.8, 0.8)),
        ))
        .id();

    let gt_speed_text = commands
        .spawn((
            Node::default(),
            Text::new("GT:  -- m/s"),
            TextFont {
                font_size: 12.0,
                ..default()
            },
            TextColor(Color::WHITE),
        ))
        .id();

    let est_speed_text = commands
        .spawn((
            Node::default(),
            Text::new("Est: --"),
            TextFont {
                font_size: 12.0,
                ..default()
            },
            TextColor(Color::srgb(0.6, 0.8, 1.0)),
        ))
        .id();

    let speed_section = commands
        .spawn((Node {
            flex_direction: FlexDirection::Column,
            width: Val::Percent(100.0),
            row_gap: Val::Px(3.0),
            ..default()
        },))
        .id();
    commands
        .entity(speed_section)
        .add_child(desired_speed_text)
        .add_child(gt_speed_text)
        .add_child(est_speed_text)
        .add_child(speed_bar);

    // ── Steering wheel ───────────────────────────────────────────────────────

    let steering_spoke = commands
        .spawn((
            Node {
                width: Val::Px(4.0),
                height: Val::Px(28.0),
                ..default()
            },
            BackgroundColor(Color::WHITE),
            Transform::IDENTITY,
        ))
        .id();

    let steering_circle = commands
        .spawn((
            Node {
                width: Val::Px(60.0),
                height: Val::Px(60.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                ..default()
            },
            BackgroundColor(Color::srgb(0.25, 0.25, 0.25)),
            BorderRadius::all(Val::Px(30.0)),
        ))
        .id();
    commands.entity(steering_circle).add_child(steering_spoke);

    let steering_text = commands
        .spawn((
            Node::default(),
            Text::new("--"),
            TextFont {
                font_size: 11.0,
                ..default()
            },
            TextColor(Color::WHITE),
        ))
        .id();

    let steering_label = commands
        .spawn((
            Node::default(),
            Text::new("Steering"),
            TextFont {
                font_size: 11.0,
                ..default()
            },
            TextColor(Color::srgb(0.6, 0.6, 0.6)),
        ))
        .id();

    let steering_col = commands
        .spawn((Node {
            flex_direction: FlexDirection::Column,
            align_items: AlignItems::Center,
            row_gap: Val::Px(4.0),
            flex_grow: 1.0,
            ..default()
        },))
        .id();
    commands
        .entity(steering_col)
        .add_child(steering_label)
        .add_child(steering_circle)
        .add_child(steering_text);

    // ── Throttle gauge ───────────────────────────────────────────────────────

    let throttle_fill = commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(0.0),
                ..default()
            },
            BackgroundColor(Color::srgb(0.0, 0.8, 0.0)),
        ))
        .id();

    let throttle_container = commands
        .spawn((
            Node {
                width: Val::Px(24.0),
                height: Val::Px(60.0),
                flex_direction: FlexDirection::Column,
                justify_content: JustifyContent::FlexEnd,
                ..default()
            },
            BackgroundColor(Color::srgb(0.2, 0.2, 0.2)),
        ))
        .id();
    commands.entity(throttle_container).add_child(throttle_fill);

    let throttle_text = commands
        .spawn((
            Node::default(),
            Text::new("--"),
            TextFont {
                font_size: 11.0,
                ..default()
            },
            TextColor(Color::WHITE),
        ))
        .id();

    let throttle_label = commands
        .spawn((
            Node::default(),
            Text::new("Throttle"),
            TextFont {
                font_size: 11.0,
                ..default()
            },
            TextColor(Color::srgb(0.6, 0.6, 0.6)),
        ))
        .id();

    let throttle_col = commands
        .spawn((Node {
            flex_direction: FlexDirection::Column,
            align_items: AlignItems::Center,
            row_gap: Val::Px(4.0),
            flex_grow: 1.0,
            ..default()
        },))
        .id();
    commands
        .entity(throttle_col)
        .add_child(throttle_label)
        .add_child(throttle_container)
        .add_child(throttle_text);

    // ── Gauges row ───────────────────────────────────────────────────────────

    let gauges_row = commands
        .spawn((Node {
            flex_direction: FlexDirection::Row,
            width: Val::Percent(100.0),
            column_gap: Val::Px(8.0),
            justify_content: JustifyContent::Center,
            ..default()
        },))
        .id();
    commands
        .entity(gauges_row)
        .add_child(steering_col)
        .add_child(throttle_col);

    // ── Header ───────────────────────────────────────────────────────────────

    let header = commands
        .spawn((
            Node::default(),
            Text::new("Vehicle Control [C]"),
            TextFont {
                font_size: 13.0,
                ..default()
            },
            TextColor(Color::srgb(0.9, 0.9, 0.9)),
        ))
        .id();

    let speed_header = commands
        .spawn((
            Node::default(),
            Text::new("─ Speed (m/s) ─"),
            TextFont {
                font_size: 11.0,
                ..default()
            },
            TextColor(Color::srgb(0.5, 0.5, 0.5)),
        ))
        .id();

    // ── Root ─────────────────────────────────────────────────────────────────

    let root = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(10.0),
                bottom: Val::Px(10.0),
                width: Val::Px(200.0),
                flex_direction: FlexDirection::Column,
                padding: UiRect::all(Val::Px(8.0)),
                row_gap: Val::Px(5.0),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
            initial_vis,
        ))
        .id();

    commands
        .entity(root)
        .add_child(header)
        .add_child(speed_header)
        .add_child(speed_section)
        .add_child(gauges_row);

    commands.insert_resource(VehicleHudEntities {
        root,
        speed_fill,
        gt_speed_text,
        est_speed_text,
        desired_speed_text,
        throttle_fill,
        throttle_text,
        steering_spoke,
        steering_text,
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Update
// ─────────────────────────────────────────────────────────────────────────────

/// Reads agent components each frame and updates HUD text/gauges.
/// No-ops if `VehicleHudEntities` resource is absent (HUD was not spawned).
#[allow(clippy::too_many_arguments)]
pub fn update_vehicle_hud(
    entities: Option<Res<VehicleHudEntities>>,
    config: Res<DebugVisualizationConfig>,
    mut node_q: Query<&mut Node>,
    mut transform_q: Query<&mut Transform>,
    mut text_q: Query<&mut Text>,
    mut bg_q: Query<&mut BackgroundColor>,
    mut vis_q: Query<&mut Visibility>,
    agent_q: Query<(
        &GroundTruthState,
        &AckermannActuator,
        &ControllerStateSource,
        Option<&EstimatorComponent>,
        Option<&ControlOutputComponent>,
        Option<&AckermannCommand>,
    )>,
) {
    let Some(entities) = entities else {
        return;
    };

    // Toggle root visibility.
    if let Ok(mut vis) = vis_q.get_mut(entities.root) {
        *vis = if config.show_vehicle_hud {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }

    if !config.show_vehicle_hud {
        return;
    }

    // Get data from the first matching agent.
    let Some((gt_state, actuator, state_source, est_opt, ctrl_opt, cmd_opt)) =
        agent_q.iter().next()
    else {
        return;
    };

    let max_speed = actuator.max_speed;

    // GT speed is always available (queried as required).
    let gt_speed = gt_state.linear_velocity.norm() as f32;

    // Estimated speed: only present when an EstimatorComponent exists.
    let est_speed: Option<f32> = est_opt.and_then(|est| {
        est.0
            .get_state()
            .and_then(|s| s.get_vector3(&StateVariable::Vx(FrameId::World)))
            .map(|v| v.norm() as f32)
    });

    // Speed used for the bar fill tracks the active controller source.
    let active_speed = match state_source {
        ControllerStateSource::GroundTruth => gt_speed,
        ControllerStateSource::Estimated => est_speed.unwrap_or(gt_speed),
    };

    // Desired speed from the last ControlOutput.
    let desired_speed: Option<f32> = ctrl_opt.map(|ctrl| match &ctrl.0 {
        ControlOutput::BodyVelocity { linear, .. } => linear.x as f32,
        ControlOutput::Raw(u) if !u.is_empty() => u[0] as f32 * max_speed,
        _ => 0.0,
    });

    // Throttle and steering from the persisted AckermannCommand component.
    let throttle_norm: Option<f32> = cmd_opt.map(|c| c.throttle_norm);
    let steering_norm: Option<f32> = cmd_opt.map(|c| c.steering_torque_norm);

    // Whether to display [CTRL] markers (only meaningful when both sources exist).
    let show_ctrl_marker = est_speed.is_some();

    // ── Text updates ─────────────────────────────────────────────────────────

    if let Ok(mut text) = text_q.get_mut(entities.desired_speed_text) {
        text.0 = match desired_speed {
            Some(s) => format!("Des: {:.1} m/s", s),
            None => "Des: --".to_string(),
        };
    }

    if let Ok(mut text) = text_q.get_mut(entities.gt_speed_text) {
        let marker = if show_ctrl_marker && *state_source == ControllerStateSource::GroundTruth {
            " [CTRL]"
        } else {
            ""
        };
        text.0 = format!("GT:  {:.1} m/s{}", gt_speed, marker);
    }

    if let Ok(mut text) = text_q.get_mut(entities.est_speed_text) {
        text.0 = match est_speed {
            Some(spd) => {
                let marker = if *state_source == ControllerStateSource::Estimated {
                    " [CTRL]"
                } else {
                    ""
                };
                format!("Est: {:.1} m/s{}", spd, marker)
            }
            None => "Est: --".to_string(),
        };
    }

    if let Ok(mut text) = text_q.get_mut(entities.throttle_text) {
        text.0 = match throttle_norm {
            Some(t) => format!("{:+.0}%", t * 100.0),
            None => "--".to_string(),
        };
    }

    if let Ok(mut text) = text_q.get_mut(entities.steering_text) {
        text.0 = match steering_norm {
            Some(s) => format!("{:+.0}%", s * 100.0),
            None => "--".to_string(),
        };
    }

    // ── Speed bar ────────────────────────────────────────────────────────────

    if let Ok(mut node) = node_q.get_mut(entities.speed_fill) {
        let pct = if max_speed > 0.0 {
            (active_speed / max_speed * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };
        node.width = Val::Percent(pct);
    }

    // ── Throttle gauge ───────────────────────────────────────────────────────

    if let Ok(mut node) = node_q.get_mut(entities.throttle_fill) {
        let pct = throttle_norm.map(|t| t.abs() * 100.0).unwrap_or(0.0);
        node.height = Val::Percent(pct);
    }

    if let Ok(mut bg) = bg_q.get_mut(entities.throttle_fill) {
        let color = match throttle_norm {
            Some(t) if t < 0.0 => Color::srgb(0.8, 0.1, 0.1),
            _ => Color::srgb(0.0, 0.8, 0.0),
        };
        *bg = BackgroundColor(color);
    }

    // ── Steering spoke rotation ───────────────────────────────────────────────

    if let Ok(mut transform) = transform_q.get_mut(entities.steering_spoke) {
        let angle = steering_norm
            .map(|s| s * 135.0_f32.to_radians())
            .unwrap_or(0.0);
        transform.rotation = Quat::from_rotation_z(angle);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// State source toggle
// ─────────────────────────────────────────────────────────────────────────────

/// Flips `ControllerStateSource` on all agent entities when T is pressed.
/// Runs every frame in `Running` state; has no effect if no agents have the component.
pub fn toggle_state_source(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut source_q: Query<&mut ControllerStateSource>,
) {
    if !keyboard.just_pressed(KeyCode::KeyT) {
        return;
    }
    for mut source in &mut source_q {
        *source = match *source {
            ControllerStateSource::Estimated => {
                info!("[Debug] Controller state source → GroundTruth");
                ControllerStateSource::GroundTruth
            }
            ControllerStateSource::GroundTruth => {
                info!("[Debug] Controller state source → Estimated");
                ControllerStateSource::Estimated
            }
        };
    }
}
