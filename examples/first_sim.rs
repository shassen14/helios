// examples/first_sim.rs

use avian3d::prelude::RigidBody;
use bevy::prelude::*;
use bevy::time::Time;
use nalgebra::{
    Isometry3, Rotation, Rotation3, Translation, Translation3, UnitQuaternion, Vector3,
};
use rust_robotics::models::ackermann_vehicle_model::AckermannVehicleModel;
use rust_robotics::models::bicycle_kinematic::BicycleKinematicModel;
use rust_robotics::simulation::collision_groups::Layer;
use rust_robotics::simulation::traits::{Dynamics, Obstacle, Shape};
use rust_robotics::simulation::utils::*;
use std::any::Any;
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

use rust_robotics::vehicles::car_setup::{RollingWheel, SteeringWheel, spawn_car}; // Import new system
use rust_robotics::vehicles::vehicle_params::{CarConfig, VehiclePhysicalParams};

use avian3d::prelude::*;
use bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;

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
        // .add_plugins(EguiPlugin {
        //     enable_multipass_for_primary_context: true,
        // })
        // .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(PhysicsPlugins::default()) // Adds required physics systems and resources
        // Optional: For debugging colliders, rigid bodies, etc.
        // .add_plugins(PhysicsDebugPlugin::default())
        .insert_resource(ClearColor(Color::srgb(0.7, 0.85, 1.0)))
        .insert_resource(GridConfig {
            // Configure your grid
            resolution: 0.5,                         // e.g., 0.5 meters per cell
            width: 1000,                             // Corresponds to 50m world width
            height: 1000,                            // Corresponds to 50m world height
            world_origin: Vec2::new(-250.0, -250.0), // Center grid at world origin
        })
        .insert_resource(ObstacleGrid::new(1000, 1000)) // Create empty grid matching config
        .insert_resource(Gravity(Vec3::NEG_Y * 9.81)) // Explicitly set gravity if needed
        .add_event::<SensorOutputEvent>()
        .add_systems(Startup, (setup_scene, setup_ui))
        // --- Systems that determine control intent (can run in Update) ---
        .add_systems(
            Update,
            (
                keyboard_control_system,
                (
                    // Planning and autonomous control can be grouped
                    update_obstacle_grid_system,
                    sim_systems::planner_system.after(update_obstacle_grid_system),
                    autonomous_controller_system.after(sim_systems::planner_system),
                )
                    .chain(), // Ensure this chain completes before apply_vehicle_forces
            )
                .chain(), // Ensure keyboard and auto controllers run before force application
        )
        // --- Systems that run in FixedUpdate, interacting with Physics ---
        // By default, systems added to FixedUpdate run before PhysicsSteps::Simulate in Avian
        .add_systems(
            FixedUpdate,
            (
                // 1. Apply forces/torques based on ControlInput.
                //    This MUST run before Avian's main simulation step.
                //    Avian's PreUpdate stage or just before PhysicsSteps::Simulate.
                apply_vehicle_forces_system
                    .after(keyboard_control_system) // Ensure ControlInput is up-to-date from player
                    .after(autonomous_controller_system), // And from AI

                                                          // 2. Update our custom TrueState from physics results.
                                                          //    This MUST run after Avian has updated Bevy components (Transform, Velocity).
                                                          //    Avian does this by PhysicsSteps::Writeback which is late in FixedUpdate.
                                                          //    So, this system should be ordered after that if possible, or run late in FixedUpdate.
                                                          //    For simplicity, if ordering sets aren't exposed as easily in Avian 0.1's main plugin,
                                                          //    we can run it in a stage known to be after physics or just ensure it's late.
                                                          //    Let's assume default FixedUpdate ordering places it after Avian's writeback
                                                          //    if Avian's systems are also in FixedUpdate. If not, PostUpdate is safer for this.
                                                          // update_true_state_from_physics, // Moved to PostUpdate for safety
            )
                .chain(), // Apply ordering within FixedUpdate
        )
        // PostUpdate is guaranteed to run after all FixedUpdates for the frame have completed.
        // This is a safe place to read the final physics state for the frame.
        .add_systems(PostUpdate, update_true_state_from_physics)
        // --- Visual/Rendering Systems (typically in Update, after physics has potentially updated Transforms) ---
        .add_systems(
            Update,
            (
                // Visual wheel animations - depend on ControlInput (for steering) or TrueState/Velocity (for rolling)
                // Run these after the systems that populate ControlInput and TrueState.
                wheel_steering_system_for_player.after(keyboard_control_system),
                autonomous_wheel_steering_system.after(autonomous_controller_system),
                wheel_rolling_system.after(update_true_state_from_physics), // Depends on TrueState (which reflects physics velocity)
                camera_follow_system, // Uses Bevy Transform directly updated by physics
                update_control_display_text, // Reads ControlInput & TrueState
                // Gizmo drawing
                draw_true_state_system,
                draw_estimated_state_system,
                draw_path_system,
                draw_goal_system,
                draw_obstacle_system,
                draw_world_axes_system,
                draw_sensor_data_system, // draw_sensor_data_system was in PostUpdate, can be here.
            )
                .chain(), // Order visual updates
        )
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
    let (ground_length, ground_width) = (500.0, 500.0);
    let ground_mesh_handle =
        meshes.add(Plane3d::default().mesh().size(ground_length, ground_width));
    let ground_material_handle = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.5, 0.3),
        ..default()
    });
    commands.spawn((
        Mesh3d(ground_mesh_handle),
        MeshMaterial3d(ground_material_handle),
        Transform::from_xyz(0.0, 0.0, 0.0),
        RigidBody::Static,
        CollisionLayers::new(
            Layer::WorldStatic,
            Layer::Character.to_bits() | Layer::GroundVehicle.to_bits(),
        ),
        Collider::cuboid(ground_length, 0.01, ground_width),
        // CollisionLayers::new(Group::ENVIRONMENT, GROUP::ALL), // Define collision groups (optional but good practice)
        Name::new("Ground"),
    ));

    // Define a translation vector
    let player_translation = Vec3::new(-10.0, 0.0, -10.0);

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
    for i in 0..0 {
        // Define start/goal in Bevy coordinates for clarity in scene setup
        let start_pos_bevy = Vec3::new(-10.0 + i as f32 * 7.0, 10.35, -10.0); // Start 10m forward (-Z)
        let goal_pos_bevy = Vec2::new(start_pos_bevy.x + 50.0, 50.0); // Goal 15m backward (+Z)

        let config = CarConfig {
            name: format!("AutonomousCar_{}", i),
            body_color: Color::srgb(0.2, 0.3, 0.9),
            initial_transform: Transform::from_translation(start_pos_bevy)
                .with_rotation(Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2)), // Rotate visual to face +Z (Backward) initially if needed
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
    let obstacle_enu_pose = bevy_transform_to_enu_iso(&obstacle_bevy_transform);
    // Define shape dimensions in Simulation frame
    let obstacle_shape_enu = Shape::Box {
        half_extents: nalgebra::Vector3::new(0.5, 1.0, 2.5),
    }; // (Right, Up, Forward)
    // --- Correct way to access data within the Shape enum ---
    let (bevy_collider_width_x, bevy_collider_height_y, bevy_collider_depth_z, avian_collider) =
        match obstacle_shape_enu {
            Shape::Box { half_extents } => {
                // half_extents is nalgebra::Vector3<f64> (dx_e, dy_n, dz_u)
                // Avian Collider::cuboid(full_x_bevy, full_y_bevy, full_z_bevy)
                let width_x = (half_extents.x * 2.0) as f32; // ENU East -> Bevy X
                let height_y = (half_extents.z * 2.0) as f32; // ENU Up -> Bevy Y
                let depth_z = (half_extents.y * 2.0) as f32; // ENU North -> Bevy Z
                (
                    width_x,
                    height_y,
                    depth_z,
                    Collider::cuboid(width_x, height_y, depth_z),
                )
            }
            Shape::Sphere { radius } => {
                // Avian Collider::ball(radius_bevy)
                // Radius is scalar, no coordinate conversion needed for the value itself.
                let r = radius as f32;
                // For the dimensions tuple, we can represent it as a bounding box for consistency if needed,
                // or just use radius for all if that's how you want to treat spheres elsewhere.
                (r * 2.0, r * 2.0, r * 2.0, Collider::sphere(r))
            }
            Shape::Cylinder {
                half_height,
                radius,
            } => {
                // Avian Collider::cylinder(height_bevy_y, radius_bevy_xz)
                // Your Shape::Cylinder is ENU: half_height along ENU Z (Up), radius in ENU XY plane.
                // Bevy Cylinder collider: height along Bevy Y, radius in Bevy XZ plane.
                let height_bevy_y = (half_height * 2.0) as f32; // ENU Up -> Bevy Y
                let radius_bevy_xz = radius as f32;
                (
                    radius_bevy_xz * 2.0,
                    height_bevy_y,
                    radius_bevy_xz * 2.0,
                    Collider::cylinder(height_bevy_y, radius_bevy_xz),
                )
            }
            Shape::Capsule {
                half_height,
                radius,
            } => {
                // Avian Collider::capsule(height_of_cylinder_part_bevy_y, radius_bevy_xz)
                // Your Shape::Capsule is ENU: half_height of cylinder part along ENU Z (Up), radius in ENU XY.
                let segment_height_bevy_y = (half_height * 2.0) as f32; // ENU Up -> Bevy Y
                let radius_bevy_xz = radius as f32;
                (
                    radius_bevy_xz * 2.0,
                    segment_height_bevy_y + 2.0 * radius_bevy_xz,
                    radius_bevy_xz * 2.0,
                    Collider::capsule(segment_height_bevy_y, radius_bevy_xz),
                )
            }
            Shape::Mesh { placeholder: _ } => {
                // For meshes, you'd ideally use Collider::trimesh_from_mesh(&bevy_mesh_asset)
                // or Collider::convex_hull_from_mesh(&bevy_mesh_asset)
                // For now, a default placeholder (e.g., a small box)
                eprintln!(
                    "WARN: Mesh collider not fully implemented for Avian, using default box."
                );
                let s = 1.0f32; // Default size
                (s, s, s, Collider::cuboid(s, s, s))
            }
        };

    commands.spawn((
        // Visuals use Bevy Transform
        Mesh3d(meshes.add(Cuboid::from_size(Vec3::new(1.0, 2.0, 5.0)))), // Size matches shape (X=1, Y=2, Z=5)
        MeshMaterial3d(materials.add(Color::srgb(0.4, 0.4, 0.5))),
        obstacle_bevy_transform, // Use Bevy transform for positioning mesh
        RigidBody::Static,
        avian_collider,
        CollisionLayers::new(
            Layer::WorldStatic,
            Layer::Character.to_bits() | Layer::GroundVehicle.to_bits(),
        ),
        // Simulation uses Sim Pose & Shape
        ObstacleComponent(Obstacle {
            id: 1,
            pose: obstacle_enu_pose,   // Store Simulation pose
            shape: obstacle_shape_enu, // Store Simulation shape
            is_static: true,
        }),
        Name::new("ObstacleBox"),
    ));

    // Example Sphere Obstacle
    // let sphere_bevy_transform = Transform::from_xyz(-5.0, 1.5, -8.0); // Bevy pose
    // let sphere_sim_pose = bevy_transform_to_enu_iso(&sphere_bevy_transform);
    //
    // let sphere_shape_sim = Shape::Sphere { radius: 1.5 };
    //
    // commands.spawn((
    //     Mesh3d(meshes.add(Sphere::new(1.5))),
    //     MeshMaterial3d(materials.add(Color::srgb(0.4, 0.4, 0.5))),
    //     sphere_bevy_transform,
    //     ObstacleComponent(Obstacle {
    //         id: 2,
    //         pose: sphere_sim_pose,
    //         shape: sphere_shape_sim,
    //         is_static: true,
    //     }),
    //     Name::new("ObstacleSphere"),
    // ));
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
            // Target's forward direction (assuming Y is up, +X is forward locally after Y rotation)
            // This depends on how dynamics_system updates the transform rotation
            let target_forward = target_transform.local_x(); // Bevy's forward is +X

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

fn apply_vehicle_forces_system(
    mut car_query: Query<
        (
            &GlobalTransform,
            &LinearVelocity,
            &AngularVelocity,
            &ControlInput,
            &DynamicsModel, // This is the component holding Box<dyn Dynamics>
            &Mass,
            &VehiclePhysicalParams,
            Entity,
        ),
        With<RigidBody>,
    >,
    mut commands: Commands,
) {
    for (
        global_transform,
        linear_velocity,
        angular_velocity,
        control_input,
        dynamics_model_comp, // This is &DynamicsModel
        mass,
        vehicle_params,
        entity,
    ) in car_query.iter_mut()
    {
        // 1. Get a reference to the trait object (&dyn Dynamics) from the Box:
        let dynamics_trait_object: &dyn Dynamics = &*dynamics_model_comp.0;
        // Or, if you needed to modify the model (not in this case for calculate_forces):
        // let dynamics_trait_object_mut: &mut dyn Dynamics = &mut *dynamics_model_comp.0;

        // 2. Call as_any() on the trait object reference:
        let any_ref: &dyn Any = dynamics_trait_object.as_any();

        // 3. Attempt the downcast:
        if let Some(bicycle_model) = any_ref.downcast_ref::<BicycleKinematicModel>() {
            // Get parameters. These should ideally be part of CarConfig or accessible via Dynamics trait
            let wheelbase_f32 = vehicle_params.wheelbase as f32; // Example
            let steering_torque_gain = 15000.0; // Needs tuning!
            let max_yaw_torque = 21000.0; // Needs tuning!

            let (world_force, world_torque) = bicycle_model.calculate_world_forces_and_torques(
                global_transform,
                linear_velocity,
                angular_velocity,
                control_input,
                mass.0, // mass.0 is the f32 value
                wheelbase_f32,
                steering_torque_gain,
                max_yaw_torque,
            );

            // ExternalForce and ExternalTorque take WORLD SPACE vectors
            commands.entity(entity).insert((
                ExternalForce::new(world_force).with_persistence(false),
                ExternalTorque::new(world_torque).with_persistence(false),
            ));
        }
        if let Some(ackermann_model) = any_ref.downcast_ref::<AckermannVehicleModel>() {
            let (world_force, world_torque) = ackermann_model
                .calculate_world_forces_and_torques_from_wheels(
                    vehicle_params, // Pass the VehiclePhysicalParams component
                    global_transform,
                    linear_velocity,
                    angular_velocity,
                    control_input,
                    // Note: car_mass is inside vehicle_params now.
                    // The method signature for calculate_world_forces_and_torques_from_wheels
                    // might not need car_mass separately if it can get it from vehicle_params.
                    // For now, let's assume the method uses vehicle_params.mass
                );
            commands.entity(entity).insert((
                ExternalForce::new(world_force).with_persistence(false),
                ExternalTorque::new(world_torque).with_persistence(false),
            ));
        } else {
            // Handle cases where it's not a BicycleKinematicModel, if necessary
            // e.g., if you have other types of DynamicsModels for other vehicles.
            println!("WARN: DynamicsModel is unkown for entity {:?}", entity);
        }
    }
}

fn update_true_state_from_physics(
    // Query entities with physics results and our TrueState component
    mut query: Query<(
        &GlobalTransform, // Updated by Avian
        &LinearVelocity,  // Updated by Avian
        &AngularVelocity, // Updated by Avian
        &mut TrueState,
    )>,
) {
    for (global_transform_bevy, linear_velocity_bevy, angular_velocity_bevy, mut true_state_enu) in
        query.iter_mut()
    {
        // 1. Convert Bevy Pose to ENU Pose
        let current_enu_pose =
            bevy_transform_to_enu_iso(&global_transform_bevy.compute_transform());

        // 2. Convert Bevy Linear Velocity to ENU Linear Velocity
        // Linear velocity vector needs to be rotated by the frame rotation
        let enu_linear_velocity_vec = bevy_vector_to_enu_vector(&linear_velocity_bevy);

        // For a 2D bicycle model, we care about forward speed in its ENU heading.
        // Vehicle's ENU forward direction:
        let enu_vehicle_forward = current_enu_pose.rotation * Vector3::x();
        let enu_forward_speed = enu_linear_velocity_vec.dot(&enu_vehicle_forward);

        // 3. Extract ENU Yaw from ENU Pose
        let enu_vehicle_x_axis = current_enu_pose.rotation * Vector3::x();
        let enu_yaw = enu_vehicle_x_axis.y.atan2(enu_vehicle_x_axis.x);

        // 4. Update TrueState (ENU)
        // Assuming TrueState is [x_e, y_n, yaw_enu, forward_speed_enu]
        true_state_enu.0[0] = current_enu_pose.translation.vector.x;
        true_state_enu.0[1] = current_enu_pose.translation.vector.y;
        true_state_enu.0[2] = enu_yaw;
        true_state_enu.0[3] = enu_forward_speed;

        // If TrueState needs ENU Z_up (it's currently 0 in state vector):
        // true_state_enu.0[some_idx_for_z_up] = current_enu_pose.translation.vector.z;
        // If TrueState needs ENU angular velocity around Z_up:
        // angular_velocity_bevy.0 is Vec3. Bevy Y is Up. ENU Z is Up.
        // So, angular_velocity_bevy.0.y is roughly ENU yaw rate.
        // (More accurately, rotate the angular velocity vector too)
        // let enu_angular_velocity_vec = ROT_ENU_FROM_BEVY.with(|r| r * angular_velocity_bevy.0.as_dvec3());
        // true_state_enu.0[some_idx_for_yaw_rate] = enu_angular_velocity_vec.z;
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
fn wheel_steering_system_for_player(
    // Renamed for clarity
    // Query for the parent entity that is keyboard controlled and has children
    player_car_query: Query<(&ControlInput, &Children), With<KeyboardControlled>>,
    // Query to get the Transform of entities that are SteeringWheels
    // This query will be filtered down using the parent's children list
    mut steering_wheel_transforms_query: Query<&mut Transform, With<SteeringWheel>>,
) {
    // Iterate over player-controlled cars (should usually be one, but handles multiple if ever needed)
    for (control_input, children) in player_car_query.iter() {
        let player_steering_angle = control_input.0.get(0).cloned().unwrap_or(0.0) as f32;

        // Base rotation to make cylinder wheel lie flat, axle along its local Y
        // (assuming cylinder created with height along Y, radius in XZ)
        // After this, wheel's local Y IS its axle. Steering rotates around parent's Y.
        // Rolling rotates around wheel's local Y.
        let base_wheel_orientation = Quat::from_rotation_x(PI / 2.0); // Align cylinder to be a wheel

        // Additional rotation for steering (around parent's Y-axis, which is wheel's X after base_orientation)
        // No, steering is rotation of the wheel around its steering pivot, which is effectively an axis aligned with parent's Y.
        // The wheel itself, after base_orientation, has its faces in parent's YZ plane, axle along parent X.
        // This isn't right.

        // Let's re-think wheel alignment in spawn_car and here:
        // In spawn_car, wheel_align_rotation = Quat::from_rotation_x(PI_F32 / 2.0);
        // This makes the Cylinder's original Y-axis (its axle) point along the parent's LOCAL Z-axis (sideways).
        // The wheel's "face" is now in the parent's local XY plane.
        // Steering means rotating this wheel around an axis parallel to the parent's LOCAL Y-axis (up).

        let steering_rotation = Quat::from_rotation_y(player_steering_angle);

        // Combine: first align the cylinder to be a wheel, then apply steering rotation
        let final_wheel_rotation = steering_rotation * base_wheel_orientation;

        // Iterate through the children of the player-controlled car
        for child_entity in children.iter() {
            // Try to get the Transform of the child if it's also a SteeringWheel
            if let Ok(mut wheel_transform) = steering_wheel_transforms_query.get_mut(child_entity) {
                wheel_transform.rotation = final_wheel_rotation;
            }
        }
    }
}

fn autonomous_wheel_steering_system(
    // Query for autonomous parent entities that have ControlInput and Children
    // Exclude KeyboardControlled to avoid conflict with the player system
    autonomous_car_query: Query<
        (&ControlInput, &Children),
        (With<AutonomousAgent>, Without<KeyboardControlled>),
    >,
    mut steering_wheel_transforms_query: Query<&mut Transform, With<SteeringWheel>>,
) {
    for (control_input, children) in autonomous_car_query.iter() {
        // This is the autonomous car's own calculated steering angle
        let autonomous_steering_angle = control_input.0.get(0).cloned().unwrap_or(0.0) as f32;

        let base_wheel_orientation = Quat::from_rotation_x(PI / 2.0);
        let steering_rotation_around_parent_y = Quat::from_rotation_y(autonomous_steering_angle);
        let final_wheel_rotation = steering_rotation_around_parent_y * base_wheel_orientation;

        for child_entity in children.iter() {
            if let Ok(mut wheel_transform) = steering_wheel_transforms_query.get_mut(child_entity) {
                wheel_transform.rotation = final_wheel_rotation;
            }
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
