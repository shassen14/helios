// examples/first_sim.rs

use bevy::prelude::*;
use bevy::time::Time;
use nalgebra::{
    Isometry3, Rotation, Rotation3, Translation, Translation3, UnitQuaternion, Vector3,
};
use rust_robotics::simulation::traits::{Obstacle, Shape};
use rust_robotics::simulation::utils::*;
use std::f32::consts::PI;

// Import local modules
use rust_robotics::input_systems::keyboard_control_system; // Keep system import
use rust_robotics::rendering::systems::*;
use rust_robotics::simulation::components::{
    ControlInput, DynamicsModel, FollowCamera, FollowCameraTarget, KeyboardControlled,
    ObstacleComponent, SensorOutputEvent, TrueState,
};
use rust_robotics::simulation::systems::{self as sim_systems, autonomous_controller_system};
// Import the spawner module and its types/functions
use rust_robotics::controllers::simple_go_to::SimpleGoToController; // Controller used by spawner
use rust_robotics::path_planning::grid_utils::{GridConfig, GridCoord, ObstacleGrid};
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
        .insert_resource(GridConfig {
            // Configure your grid
            resolution: 0.5,                       // e.g., 0.5 meters per cell
            width: 100,                            // Corresponds to 50m world width
            height: 100,                           // Corresponds to 50m world height
            world_origin: Vec2::new(-25.0, -25.0), // Center grid at world origin
        })
        .insert_resource(ObstacleGrid::new(100, 100)) // Create empty grid matching config
        .add_event::<SensorOutputEvent>()
        .add_systems(Startup, (setup_scene, setup_ui))
        .add_systems(
            FixedUpdate,
            (
                sim_systems::dynamics_system,
                // Note: Controllers might ideally run at a slightly lower rate
                // than dynamics, but running them here ensures they use the
                // state *before* the dynamics step that uses their output.
                // Alternatively, run controllers in Update *before* FixedUpdate stage flushes.
            )
                .chain(),
        )
        .add_systems(
            Update,
            (
                // 1. Update environment representation
                update_obstacle_grid_system,
                // 2. Input & Planning (can run in parallel with grid update if careful)
                keyboard_control_system, // Handles player input directly
                sim_systems::planner_system // Plans for autonomous agents
                    .after(update_obstacle_grid_system), // Ensure grid is updated first
                // 3. Autonomous Control (depends on planner output)
                autonomous_controller_system // Calculates control for autonomous agents
                    .after(sim_systems::planner_system), // Ensure path is available
                // 4. Update simulation state visuals based on TrueState
                //    (Dynamics runs in FixedUpdate, so this uses the latest integrated state)
                update_car_transform_from_state,
                // 5. Visual Updates & Camera (depend on Transform)
                wheel_steering_system, // Uses ControlInput
                wheel_rolling_system,  // Uses TrueState
                camera_follow_system.after(update_car_transform_from_state),
                update_control_display_text, // Reads ControlInput/TrueState
                // 6. Rendering Gizmos (can run fairly late)
                draw_true_state_system,
                draw_estimated_state_system,
                draw_path_system,
                draw_goal_system,
                draw_obstacle_system,
                draw_world_axes_system,
            )
                .chain(), // Apply ordering constraints within Update
        )
        .add_systems(PostUpdate, draw_sensor_data_system)
        .run();
}

fn update_obstacle_grid_system(
    mut obstacle_grid: ResMut<ObstacleGrid>,
    grid_config: Res<GridConfig>,
    // Query ObstacleComponent directly - it now holds the Simulation pose
    obstacle_query: Query<&ObstacleComponent>,
) {
    obstacle_grid.clear();

    for obs_comp in obstacle_query.iter() {
        // Pose is already in Simulation Frame (X=Right, Y=Up, Z=Forward)
        let obs_pose_sim = &obs_comp.0.pose;
        let obs_pos_sim_2d = Vec2::new(
            obs_pose_sim.translation.x as f32,
            obs_pose_sim.translation.z as f32,
        ); // Use Sim X/Z

        // Extract shape dimensions (already in Sim Frame)
        let half_extents_sim_2d = match &obs_comp.0.shape {
            Shape::Sphere { radius } => Vec2::splat(*radius as f32),
            Shape::Box { half_extents } => Vec2::new(half_extents.x as f32, half_extents.z as f32), // Sim X/Z extents
            Shape::Cylinder { radius, .. } => Vec2::splat(*radius as f32),
            Shape::Capsule { radius, .. } => Vec2::splat(*radius as f32),
            Shape::Mesh { .. } => Vec2::splat(1.0),
        };

        // Basic AABB rasterization in Simulation Frame grid
        // TODO: Account for obstacle rotation using obs_pose_sim.rotation
        let min_sim = obs_pos_sim_2d - half_extents_sim_2d;
        let max_sim = obs_pos_sim_2d + half_extents_sim_2d;

        if let (Some(min_grid), Some(max_grid)) = (
            grid_config.world_to_grid(min_sim),
            grid_config.world_to_grid(max_sim),
        ) {
            for x in min_grid.x..=max_grid.x {
                for y in min_grid.y..=max_grid.y {
                    // Grid Y maps to Sim Z
                    obstacle_grid.set_occupied(GridCoord { x, y }, true);
                }
            }
        }
    }
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

    // Define a translation vector
    let player_translation = Vec3::new(5.0, 0.0, 0.0);

    // Define a rotation quaternion (e.g., 90 degrees around the Y axis)
    let player_rotation = Quat::from_rotation_y(1.0 * std::f32::consts::FRAC_PI_2);

    // Create a Transform with the specified translation and rotation
    let player_transform = Transform {
        translation: player_translation,
        rotation: player_rotation,
        ..Default::default() // Keep the scale as default (Vec3::ONE)
    };

    // --- Spawn the Player Car using the Spawner ---
    let player_car_config = CarConfig {
        name: "PlayerCar".to_string(),
        initial_transform: player_transform,
        body_color: Color::srgb(0.8, 0.1, 0.1), // Ensure player is red
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
        false, // keyboard=T, follow=T, autonomous=F, planner=F
    );

    // Autonomous Cars
    // Make sure initial transform and goal pos use Bevy coordinates,
    // but the underlying components (TrueState, GoalComponent) will use Sim coords.
    for i in 0..1 {
        // Define start/goal in Bevy coordinates for clarity in scene setup
        let start_pos_bevy = Vec3::new(-10.0 + i as f32 * 7.0, 0.35, -10.0); // Start 10m forward (-Z)
        let goal_pos_bevy = Vec2::new(start_pos_bevy.x, 10.0); // Goal 15m backward (+Z)

        let config = CarConfig {
            name: format!("AutonomousCar_{}", i),
            body_color: Color::srgb(0.2, 0.3, 0.9),
            initial_transform: Transform::from_translation(start_pos_bevy)
                .with_rotation(Quat::from_rotation_y(-PI)), // Rotate visual to face +Z (Backward) initially if needed
            initial_velocity: 0.0,
            // *** IMPORTANT: car_setup needs to be updated to set TrueState and GoalComponent
            // *** using SIMULATION coordinates derived from these Bevy inputs.
            // *** We'll modify spawn_car next.
            ..Default::default()
        };

        // Call spawn_car, passing the Bevy coordinate goal
        spawn_car(
            &mut commands,
            &mut meshes,
            &mut materials,
            &config,
            false,
            false,
            true,
            Some(goal_pos_bevy), // Pass Bevy goal coordinates
            true,
        );
    }

    // Spawn obstacles using Bevy coordinates for visuals,
    // but store Simulation pose in ObstacleComponent.
    let obstacle_bevy_transform = Transform::from_xyz(5.0, 1.0, -10.0); // Bevy pose: Z=-10 means 10m forward
    // Convert Bevy transform to Simulation pose
    let obstacle_sim_pose = bevy_transform_to_enu_iso(&obstacle_bevy_transform);
    // Define shape dimensions in Simulation frame
    let obstacle_shape_sim = Shape::Box {
        half_extents: nalgebra::Vector3::new(0.5, 1.0, 2.5),
    }; // (Right, Up, Forward)

    commands.spawn((
        // Visuals use Bevy Transform
        Mesh3d(meshes.add(Cuboid::from_size(Vec3::new(1.0, 2.0, 5.0)))), // Size matches shape (X=1, Y=2, Z=5)
        MeshMaterial3d(materials.add(Color::srgb(0.4, 0.4, 0.5))),
        obstacle_bevy_transform, // Use Bevy transform for positioning mesh
        // Simulation uses Sim Pose & Shape
        ObstacleComponent(Obstacle {
            id: 1,
            pose: obstacle_sim_pose,   // Store Simulation pose
            shape: obstacle_shape_sim, // Store Simulation shape
            is_static: true,
        }),
        Name::new("ObstacleBox"),
    ));

    // Example Sphere Obstacle
    let sphere_bevy_transform = Transform::from_xyz(-5.0, 1.5, -8.0); // Bevy pose
    let sphere_sim_pose = bevy_transform_to_enu_iso(&sphere_bevy_transform);
    let sphere_shape_sim = Shape::Sphere { radius: 1.5 };

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(1.5))),
        MeshMaterial3d(materials.add(Color::srgb(0.4, 0.4, 0.5))),
        sphere_bevy_transform,
        ObstacleComponent(Obstacle {
            id: 2,
            pose: sphere_sim_pose,
            shape: sphere_shape_sim,
            is_static: true,
        }),
        Name::new("ObstacleSphere"),
    ));
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
    // Query entities that have both simulation state and a visual transform
    mut query: Query<(&TrueState, &mut Transform), With<DynamicsModel>>, // Assuming cars have DynamicsModel
                                                                         // Optional: Query wheel radius if needed for ground offset
                                                                         // car_config_query: Query<&CarConfig>, // If wheel radius is stored there
) {
    // Default ground level assumption for visuals
    let wheel_radius = 0.35; // TODO: Get from component

    for (true_state, mut transform) in query.iter_mut() {
        let state = &true_state.0; // State is now [sim_x, sim_z, sim_yaw, sim_v]
        if state.len() >= 3 {
            // Construct the pose in the ENU frame
            // Assuming ground vehicle, so Z_up (ENU) is 0.0 for translation
            // If your vehicle can move vertically in ENU, state should include enu_z.
            let enu_translation_vector = Vector3::new(state[0], state[1], wheel_radius);
            let enu_rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), state[2]); // Yaw around ENU Z-axis
            let enu_pose =
                Isometry3::from_parts(Translation3::from(enu_translation_vector), enu_rotation);

            // Convert the full ENU pose to Bevy Transform
            let new_bevy_transform = enu_iso_to_bevy_transform(&enu_pose);

            // Update Bevy component (only pose, keep scale etc.)
            transform.translation = new_bevy_transform.translation;
            transform.rotation = new_bevy_transform.rotation;
            // Note: bevy_transform.translation.y will be set by enu_iso_to_bevy_transform
            // based on enu_translation_vector.z (which is 0.0 here).
            // If you need the car to be at a specific Bevy Y height (e.g. wheel_radius),
            // you either:
            // 1. Make enu_translation_vector.z be wheel_radius_in_enu_units.
            // 2. OR, after calling enu_iso_to_bevy_transform, explicitly set:
            //    bevy_transform.translation.y = desired_bevy_y_height;}
        }
    }
}

fn wheel_rolling_system(
    parent_query: Query<(Entity, &TrueState, &Children), With<DynamicsModel>>,
    mut wheel_query: Query<&mut Transform, With<RollingWheel>>,
    time: Res<Time>,
    mut wheel_radius_opt: Local<Option<f32>>,
) {
    let wheel_radius = *wheel_radius_opt.get_or_insert(0.35);
    if wheel_radius.abs() < 1e-6 {
        return;
    }
    let dt = time.delta().as_secs_f32();
    if dt <= 0.0 {
        return;
    }

    for (_parent_entity, true_state, children) in parent_query.iter() {
        let velocity = true_state.0.get(3).cloned().unwrap_or(0.0) as f32; // Forward velocity
        let angular_velocity = velocity / wheel_radius; // rad/s
        let rotation_change = angular_velocity * dt; // radians

        if rotation_change.abs() > 1e-6 {
            for child_entity in children.iter() {
                if let Ok(mut wheel_transform) = wheel_query.get_mut(child_entity) {
                    // Assuming wheels are oriented with their cylinder axis along Bevy Y initially,
                    // and X points "forward" relative to the wheel axle.
                    // Rolling forward should be rotation around the wheel's *local* right axis (Bevy Z axis if not steered?)
                    // Let's try rotating around local X first as that's common for cylinder meshes.
                    // Sign depends on coordinate system and mesh orientation.
                    wheel_transform.rotate_local_y(-rotation_change); // Adjust sign if rolling backward visually
                }
            }
        }
    }
}

// --- Update Wheel Steering System (Ensure correct rotation axis) ---
fn wheel_steering_system(
    parent_query: Query<&ControlInput, With<KeyboardControlled>>, // Could also query AutonomousAgent eventually
    mut wheel_query: Query<&mut Transform, With<SteeringWheel>>,
) {
    if let Ok(parent_control_input) = parent_query.single() {
        let steering_angle = parent_control_input.0.get(0).cloned().unwrap_or(0.0) as f32; // Sim delta angle

        // Assuming wheels are children of the main car body.
        // Base rotation aligns wheel axle with Bevy X/Z plane.
        // Steering rotation should be around the local UP axis (Bevy Y).
        // Need initial wheel orientation from car_setup. Let's assume it was Quat::from_rotation_x(PI/2.0)
        let base_axle_rotation = Quat::from_rotation_x(PI / 2.0); // Initial orientation in setup
        let steer_rotation = Quat::from_rotation_y(steering_angle); // Yaw around Bevy Y

        // Apply steering ON TOP of base orientation
        for mut wheel_transform in wheel_query.iter_mut() {
            // Reset rotation first? Or combine? Combine: Steer first, then align axle.
            // wheel_transform.rotation = base_axle_rotation * steer_rotation; // Order matters! Try swapping if wrong.
            wheel_transform.rotation = steer_rotation * base_axle_rotation; // Apply steer yaw, then tilt axle. Check visually.
        }
    }
}

fn draw_world_axes_system(mut gizmos: Gizmos) {
    // Define the length of the axis lines
    let axis_length = 3.0; // Adjust length as needed for visibility

    // --- Draw X-Axis (Red) ---
    // From origin (0,0,0) along positive X
    //    gizmos.line(
    //        Vec3::ZERO,                 // Start point
    //        Vec3::X * axis_length,      // End point (+X direction)
    //        Color::srgb(1.0, 0.2, 0.2), // Red color (slightly desaturated)
    //    );
    //
    //    // --- Draw Y-Axis (Green) ---
    //    // From origin (0,0,0) along positive Y (UP in Bevy)
    //    gizmos.line(
    //        Vec3::ZERO,                 // Start point
    //        Vec3::Y * axis_length,      // End point (+Y direction)
    //        Color::srgb(0.2, 1.0, 0.2), // Green color
    //    );
    //
    //    // --- Draw Z-Axis (Blue) ---
    //    // From origin (0,0,0) along positive Z (BACKWARDS/Towards Camera in Bevy)
    //    gizmos.line(
    //        Vec3::ZERO,                 // Start point
    //        Vec3::Z * axis_length,      // End point (+Z direction)
    //        Color::srgb(0.2, 0.2, 1.0), // Blue color
    //    );

    // --- Optional: Draw Negative Axes (e.g., with darker colors or dashed lines) ---
    // gizmos.line(Vec3::ZERO, Vec3::NEG_X * axis_length, Color::srgb(0.5, 0.1, 0.1));
    // gizmos.line(Vec3::ZERO, Vec3::NEG_Y * axis_length, Color::srgb(0.1, 0.5, 0.1));
    // gizmos.line(Vec3::ZERO, Vec3::NEG_Z * axis_length, Color::srgb(0.1, 0.1, 0.5));

    // --- Optional: Make axes thicker using cuboids (lines can be thin) ---

    let thickness = 0.05;
    gizmos.cuboid(
        Transform::from_xyz(axis_length / 2.0, 0.0, 0.0) // Centered on axis
            .with_scale(Vec3::new(axis_length, thickness, thickness)),
        Color::srgb(1.0, 0.2, 0.2),
    );
    gizmos.cuboid(
        Transform::from_xyz(0.0, axis_length / 2.0, 0.0).with_scale(Vec3::new(
            thickness,
            axis_length,
            thickness,
        )),
        Color::srgb(0.2, 1.0, 0.2),
    );
    gizmos.cuboid(
        Transform::from_xyz(0.0, 0.0, axis_length / 2.0).with_scale(Vec3::new(
            thickness,
            thickness,
            axis_length,
        )),
        Color::srgb(0.2, 0.2, 1.0),
    );
}
