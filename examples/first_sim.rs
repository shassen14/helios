// examples/first_sim.rs

use bevy::prelude::*;
use bevy::time::Time;
use std::f32::consts::PI;

// Import local modules
use rust_robotics::input_systems::keyboard_control_system; // Keep system import
use rust_robotics::rendering::systems::*;
use rust_robotics::simulation::components::{
    ControlInput, DynamicsModel, FollowCamera, FollowCameraTarget, KeyboardControlled,
    SensorOutputEvent, TrueState,
};
use rust_robotics::simulation::systems::{self as sim_systems, autonomous_controller_system};
// Import the spawner module and its types/functions
use rust_robotics::controllers::simple_go_to::SimpleGoToController; // Controller used by spawner
use rust_robotics::simulation::components::{AutonomousAgent, GoalComponent}; // Import new components
use rust_robotics::vehicles::car_setup::{
    CarConfig,
    RollingWheel, // Import markers from here
    SteeringWheel,
    spawn_car,
}; // Import new system

// Constants for scheduling (Hz)
// const DYNAMICS_HZ: f64 = 100.0; // Not explicitly used for scheduling in this version

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Robotics Simulator Example".into(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            close_when_requested: true,
            exit_condition: bevy::window::ExitCondition::OnPrimaryClosed,
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.7, 0.85, 1.0)))
        .add_event::<SensorOutputEvent>()
        .add_systems(Startup, (setup_scene, setup_ui))
        .add_systems(FixedUpdate, sim_systems::dynamics_system) // Simulation
        .add_systems(
            Update,
            (
                keyboard_control_system,      // Input for player
                autonomous_controller_system, // Control for autonomous cars
                update_control_display_text,
                wheel_steering_system, // Only affects player car currently
                wheel_rolling_system,  // Affects all cars
                update_car_transform_from_state // Update transforms for *all* cars now
                    .after(sim_systems::dynamics_system), // Run after physics
                camera_follow_system.after(update_car_transform_from_state),
                // Rendering Gizmo Systems
                draw_true_state_system,
                draw_estimated_state_system,
                draw_path_system,
                draw_goal_system,
                draw_obstacle_system,
            )
                .chain(),
        )
        .add_systems(PostUpdate, draw_sensor_data_system)
        .run();
}

// --- UI Components ---
#[derive(Component)]
struct ControlDisplayUIRoot; // Marker for the root UI Node

#[derive(Component)]
struct SteerTextDisplay; // Marker for steer text entity

#[derive(Component)]
struct AccelTextDisplay; // Marker for accel text entity

#[derive(Component)]
struct VelocityTextDisplay; // Marker for velocity text

// --- UI Setup System (Corrected for 0.16: Node + child Text components) ---
fn setup_ui(mut commands: Commands) {
    // Spawn the root UI Node for positioning
    commands
        .spawn((
            Node {
                // Spawn Node component directly
                // Configure layout fields directly on Node
                position_type: PositionType::Absolute,
                top: Val::Px(10.0),
                left: Val::Px(10.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(5.0),

                // Optional background for visibility
                // background_color: Color::srgba(1.0, 1.0, 1.0, 0.5).into(),
                ..default()
            },
            ControlDisplayUIRoot,
            Name::new("UI_ControlDisplay"),
            Transform::default(),
            Visibility::default(),
        ))
        .with_children(|parent| {
            // Spawn child entities with Text components for each line
            parent.spawn((
                Text::new("Steer: "), // Spawn Text component
                // Add required layout/styling components
                TextLayout {
                    justify: JustifyText::Left,
                    linebreak: LineBreak::AnyCharacter, // Use correct field name and enum
                    ..default()
                },
                // TextColor::from(Color::srgb(0.1, 0.1, 0.1)), // Optional: Add TextColor if not default black
                // TextFont is usually added by Bevy with default font
                SteerTextDisplay, // Marker
                Name::new("UI_SteerText"),
            ));
            parent.spawn((
                Text::new("Accel: "),
                TextLayout {
                    justify: JustifyText::Left,
                    linebreak: LineBreak::AnyCharacter, // Correct field name
                    ..default()
                },
                // TextColor::from(Color::srgb(0.1, 0.1, 0.1)),
                AccelTextDisplay, // Marker
                Name::new("UI_AccelText"),
            ));
            parent.spawn((
                Text::new("Velocity: 0.00 m/s"),
                TextFont {
                    font_size: 20.0,
                    ..default()
                },
                TextColor(Color::srgb(0.1, 0.1, 0.1)),
                VelocityTextDisplay, // Add marker
                Name::new("UI_VelocityText"),
            ));
        });
}

fn update_control_display_text(
    car_query: Query<(&ControlInput, &TrueState), With<KeyboardControlled>>,
    // Query the Entity ID of the root UI node
    ui_steer_query: Query<Entity, With<SteerTextDisplay>>,
    ui_accel_query: Query<Entity, With<AccelTextDisplay>>,
    ui_velocity_query: Query<Entity, With<VelocityTextDisplay>>,
    mut writer: TextUiWriter,
) {
    // Use .single() which returns Result
    if let Ok((control_input, true_state)) = car_query.single() {
        let steer_deg = control_input.0.get(0).cloned().unwrap_or(0.0).to_degrees();
        let accel = control_input.0.get(1).cloned().unwrap_or(0.0);
        let velocity = true_state.0.get(3).cloned().unwrap_or(0.0);

        // Use .single() which returns Result
        if let Ok(ui_steer_entity) = ui_steer_query.single() {
            let text = format!("Steer: {:.2}", steer_deg);
            *writer.text(ui_steer_entity, 0) = text;
        }
        if let Ok(ui_accel_entity) = ui_accel_query.single() {
            let text = format!("Accel: {:.2}", accel);
            *writer.text(ui_accel_entity, 0) = text;
        }
        if let Ok(ui_velocity_entity) = ui_velocity_query.single() {
            let text = format!("Velocity: {:.2}", velocity);
            *writer.text(ui_velocity_entity, 0) = text;
        }
    } else {
        // Handle no car found
        if let Ok(ui_steer_entity) = ui_steer_query.single() {
            let text = "Steer: ---".to_string();
            *writer.text(ui_steer_entity, 0) = text;
        }
        if let Ok(ui_accel_entity) = ui_accel_query.single() {
            let text = "Accel: ---".to_string();
            *writer.text(ui_accel_entity, 0) = text;
        }
        if let Ok(ui_accel_entity) = ui_accel_query.single() {
            let text = "Velocity: ---".to_string();
            *writer.text(ui_accel_entity, 0) = text;
        }
    }
}

/// System to set up the initial simulation scene (Bevy 0.16 Corrected)
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // --- Camera ---
    commands.spawn((
        Camera3d::default(),
        // Start with initial transform, the follow system will update it
        Transform::from_xyz(-15.0, 10.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
        FollowCamera {
            // Add the follow component to the camera
            distance: 15.0, // Distance behind target
            height: 8.0,    // Height above target's base
            lag_speed: 3.0, // Adjust for desired smoothness
        },
        Name::new("MainCamera"),
    ));

    // --- Lighting ---
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 15000.0,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.8, -0.5, 0.0)),
        Name::new("SunLight"),
    ));

    // --- Ground Plane ---
    let ground_mesh_handle = meshes.add(Plane3d::default().mesh().size(50.0, 50.0));
    let ground_material_handle = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.5, 0.3),
        ..default()
    });
    commands.spawn((
        Mesh3d(ground_mesh_handle),
        MeshMaterial3d(ground_material_handle),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Name::new("Ground"),
    ));

    // --- Spawn the Player Car using the Spawner ---
    let player_car_config = CarConfig {
        name: "PlayerCar".to_string(),
        initial_transform: Transform::from_xyz(0.0, 0.35, 0.0), // Start Y=wheel_radius
        ..Default::default()
    };

    let _player_car_entity = spawn_car(
        &mut commands,
        &mut meshes,
        &mut materials,
        &player_car_config,
        true,
        true,
        false,
        None,
    );

    for i in 0..5 {
        let start_pos = Vec3::new(-10.0 + i as f32 * 4.0, 0.35, 5.0);
        let goal_pos = Vec2::new(start_pos.x, -15.0); // Goal towards bottom edge
        let config = CarConfig {
            name: format!("AutonomousCar_{}", i),
            body_color: Color::srgb(0.2, 0.3, 0.9),
            initial_transform: Transform::from_translation(start_pos)
                .with_rotation(Quat::from_rotation_y(PI)), // Start facing down
            initial_velocity: 3.0, // Start moving
            ..Default::default()
        };
        let _car_entity = spawn_car(
            &mut commands,
            &mut meshes,
            &mut materials,
            &config,
            false,          // Not keyboard controlled
            false,          // Not camera target
            true,           // IS autonomous
            Some(goal_pos), // Assign initial goal
        );
    }
}

fn camera_follow_system(
    time: Res<Time>,
    target_query: Query<&Transform, (With<FollowCameraTarget>, Without<FollowCamera>)>, // Target transform
    mut camera_query: Query<(&mut Transform, &FollowCamera), With<Camera>>, // Camera transform & settings
) {
    // Use .single() for target and camera
    if let Ok(target_transform) = target_query.single() {
        if let Ok((mut camera_transform, follow_settings)) = camera_query.single_mut() {
            let target_pos = target_transform.translation;
            // Target's forward direction (assuming Y is up, Z is forward locally after Y rotation)
            // This depends on how dynamics_system updates the transform rotation
            let target_forward = target_transform.forward(); // Bevy's forward is -Z

            // Calculate desired camera position: offset behind and above the target
            // Use target's forward direction (negated) to position behind
            let desired_offset =
                (-target_forward * follow_settings.distance) + (Vec3::Y * follow_settings.height);
            let desired_camera_pos = target_pos + desired_offset;

            // Smoothly interpolate (lerp) the camera's position towards the desired position
            let lerp_factor = (time.delta().as_secs_f32() * follow_settings.lag_speed).min(1.0); // Clamp factor to avoid overshooting
            camera_transform.translation = camera_transform
                .translation
                .lerp(desired_camera_pos, lerp_factor);

            // Make the camera look at the target's position
            // Use look_at for smooth orientation change
            camera_transform.look_at(target_pos, Vec3::Y); // Look at target, keep Y axis up
        }
    }
}

fn update_car_transform_from_state(
    mut query: Query<(&TrueState, &mut Transform), With<DynamicsModel>>,
) {
    for (true_state, mut transform) in query.iter_mut() {
        // Iterate over all cars
        let state = &true_state.0;
        if state.len() >= 3 {
            transform.translation.x = state[0] as f32;
            transform.translation.z = -state[1] as f32; // State y maps to world Z
            transform.rotation = Quat::from_rotation_y(state[2] as f32);
        }
    }
}

fn wheel_steering_system(
    parent_query: Query<&ControlInput, With<KeyboardControlled>>,
    mut wheel_query: Query<&mut Transform, With<SteeringWheel>>,
) {
    // Use .single()
    if let Ok(parent_control_input) = parent_query.single() {
        let steering_angle = parent_control_input.0.get(0).cloned().unwrap_or(0.0) as f32;
        let base_axle_rotation = Quat::from_rotation_x(PI / 2.0);
        let steer_rotation = Quat::from_rotation_y(steering_angle);
        for mut wheel_transform in wheel_query.iter_mut() {
            wheel_transform.rotation = steer_rotation * base_axle_rotation;
        }
    }
}

fn wheel_rolling_system(
    parent_query: Query<&TrueState, With<KeyboardControlled>>,
    mut wheel_query: Query<&mut Transform, With<RollingWheel>>,
    time: Res<Time>,
    mut wheel_radius_opt: Local<Option<f32>>,
) {
    let wheel_radius = *wheel_radius_opt.get_or_insert(0.35);
    // Use .single()
    if let Ok(true_state) = parent_query.single() {
        let velocity = true_state.0.get(3).cloned().unwrap_or(0.0) as f32;
        if wheel_radius.abs() < 1e-6 {
            return;
        }
        let angular_velocity = velocity / wheel_radius;
        // Use correct delta time method -> as_secs_f32()
        let rotation_change = angular_velocity * time.delta().as_secs_f32(); // Corrected time delta access

        if rotation_change.abs() < 1e-6 {
            return;
        }
        for mut wheel_transform in wheel_query.iter_mut() {
            wheel_transform.rotate_local_y(rotation_change);
        }
    }
}
