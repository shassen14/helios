// helios_sim/src/simulation/plugins/debugging/ui/vehicle_hud.rs
//
// Vehicle control HUD: spawn logic + entity handle resource.
// Runtime update systems live in vehicle_hud_update.rs.
//
// Toggle with C; T switches ControllerStateSource when both control + estimation are active.

use bevy::prelude::*;

use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;
use crate::simulation::profile::CapabilitySet;

pub use super::vehicle_hud_update::{toggle_state_source, update_vehicle_hud};

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
            TextFont { font_size: 12.0, ..default() },
            TextColor(Color::srgb(0.8, 0.8, 0.8)),
        ))
        .id();

    let gt_speed_text = commands
        .spawn((
            Node::default(),
            Text::new("GT:  -- m/s"),
            TextFont { font_size: 12.0, ..default() },
            TextColor(Color::WHITE),
        ))
        .id();

    let est_speed_text = commands
        .spawn((
            Node::default(),
            Text::new("Est: --"),
            TextFont { font_size: 12.0, ..default() },
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
            Node { width: Val::Px(4.0), height: Val::Px(28.0), ..default() },
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
            TextFont { font_size: 11.0, ..default() },
            TextColor(Color::WHITE),
        ))
        .id();

    let steering_label = commands
        .spawn((
            Node::default(),
            Text::new("Steering"),
            TextFont { font_size: 11.0, ..default() },
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
            Node { width: Val::Percent(100.0), height: Val::Percent(0.0), ..default() },
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
            TextFont { font_size: 11.0, ..default() },
            TextColor(Color::WHITE),
        ))
        .id();

    let throttle_label = commands
        .spawn((
            Node::default(),
            Text::new("Throttle"),
            TextFont { font_size: 11.0, ..default() },
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
            TextFont { font_size: 13.0, ..default() },
            TextColor(Color::srgb(0.9, 0.9, 0.9)),
        ))
        .id();

    let speed_header = commands
        .spawn((
            Node::default(),
            Text::new("─ Speed (m/s) ─"),
            TextFont { font_size: 11.0, ..default() },
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
