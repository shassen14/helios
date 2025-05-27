// src/vehicles/car_setup.rs

use crate::controllers::path_follower::PathFollowerController;
use crate::controllers::simple_go_to::SimpleGoToController;
use crate::models::bicycle_kinematic::BicycleKinematicModel;
use crate::path_planning::grid_planner::GridPlanner;
// Assuming model is public
use crate::simulation::components::*;
use crate::simulation::traits::Goal;
use crate::simulation::utils::{bevy_transform_to_enu_iso, enu_iso_to_bevy_transform}; // Import simulation components
use bevy::prelude::*;
use std::f32::consts::PI;

// Marker components defined here for clarity, even if used elsewhere
#[derive(Component)]
pub struct SteeringWheel;
#[derive(Component)]
pub struct RollingWheel;

/// Configuration struct holding parameters to define and spawn a car.
#[derive(Clone, Debug)] // Allow cloning for multiple spawns
pub struct CarConfig {
    // --- Visuals ---
    pub body_color: Color,
    pub wheel_color: Color,
    pub body_size: Vec3, // length, height, width
    pub wheel_radius: f32,
    pub wheel_width: f32,

    // --- Dynamics / Kinematics ---
    pub wheelbase: f64,
    pub max_steer_angle_deg: f64, // Max steering angle in degrees

    // --- Control Limits/Parameters (Optional - could be separate component) ---
    // These are currently hardcoded in input_system, but could be moved here
    // pub max_acceleration: f64,
    // pub max_braking: f64,
    // pub steering_rate_rad_per_sec: f64,
    // pub accel_rate_m_per_s3: f64,

    // --- Initial State ---
    pub initial_transform: Transform, // Initial position and orientation of the root
    pub initial_velocity: f64,        // Initial longitudinal velocity

    // --- Entity Info ---
    pub name: String,
}

impl Default for CarConfig {
    /// Default car configuration matching the original example.
    fn default() -> Self {
        let wheelbase = 2.7;
        let wheel_radius = 0.35;
        let car_body_height = 0.8;
        let initial_state_vec = vec![0.0, 0.0, 0.0, 0.0]; // x, y, theta, v
        let initial_transform =
            Transform::from_xyz(0.0, wheel_radius, 0.0).with_rotation(Quat::IDENTITY); // No initial rotation in Bevy frame
        Self {
            body_color: Color::srgb(0.8, 0.1, 0.1),  // Red
            wheel_color: Color::srgb(0.1, 0.1, 0.1), // Dark grey
            body_size: Vec3::new(3.0, 0.8, 1.5),     // length, height, width
            wheel_radius,
            wheel_width: 0.2,
            wheelbase,
            max_steer_angle_deg: 35.0,
            initial_transform,
            initial_velocity: 0.0,
            name: "Car".to_string(),
        }
    }
}

/// Spawns a complete car entity hierarchy based on the provided configuration.
pub fn spawn_car(
    commands: &mut Commands,           // Use &mut Commands
    meshes: &mut ResMut<Assets<Mesh>>, // Pass resources needed
    materials: &mut ResMut<Assets<StandardMaterial>>,
    config: &CarConfig,
    // --- Optional flags for components to add ---
    add_keyboard_controller: bool,
    add_follow_target: bool,
    add_autonomous_agent: bool,
    initial_goal_bevy_pos: Option<Vec2>,
    add_planner: bool, // NEW flag for planner
) -> Entity {
    // Return the Entity ID of the spawned root

    // --- Calculate derived values ---
    let front_axle_offset = config.wheelbase / 2.0;
    let rear_axle_offset = -config.wheelbase / 2.0;
    let wheel_track_width = config.body_size.z * 0.8; // Use configured body width
    let car_body_y_offset = config.wheel_radius + config.body_size.y / 2.0; // Center body vertically

    // --- Create Assets ---
    let car_body_mat = materials.add(config.body_color);
    let wheel_mat = materials.add(config.wheel_color);
    let wheel_mesh = meshes.add(Cylinder::new(config.wheel_radius, config.wheel_width * 2.0));
    let car_body_mesh = meshes.add(Cuboid::new(
        config.body_size.x,
        config.body_size.y,
        config.body_size.z,
    ));

    // --- Dynamics Model ---
    let bicycle_model = BicycleKinematicModel::new(config.wheelbase, config.max_steer_angle_deg);

    // --- Initial State ---
    // Use transform from config, but ensure initial state vector matches
    let initial_sim_pose = bevy_transform_to_enu_iso(&config.initial_transform);
    let initial_state_sim = StateVector::from_vec(vec![
        initial_sim_pose.translation.x,             // enu_x
        initial_sim_pose.translation.y,             // enu_y
        initial_sim_pose.rotation.euler_angles().2, // enu_yaw (around Z) - Extract from Isometry rotation
        config.initial_velocity,                    // sim_v
    ]);

    // --- Spawn Parent Entity ---
    let mut parent_entity_commands = commands.spawn((
        // Add essential non-optional components first
        config.initial_transform, // Use the Bevy transform for visual placement
        Name::new(config.name.clone()),
        // Simulation Components use Simulation Frame data
        TrueState(initial_state_sim.clone()), // Store initial Sim state
        DynamicsModel(Box::new(bicycle_model)),
        ControlInput::default(), // Control input [delta, a] doesn't need conversion itself
        DrawTrueStateViz(true),  // Controls gizmo visibility
        Visibility::default(),   // Controls mesh visibility
                                 // InheritedVisibility::default(),
                                 // ViewVisibility::default(),
    ));

    // --- Conditionally Insert Optional Components ---
    if add_keyboard_controller {
        parent_entity_commands.insert(KeyboardControlled);
    }
    if add_follow_target {
        parent_entity_commands.insert(FollowCameraTarget);
    }

    if add_autonomous_agent {
        parent_entity_commands.insert(AutonomousAgent);

        // Default controller
        let controller = Box::new(PathFollowerController {
            target_velocity: 4.0,
            lookahead_distance: 5.0,
            kp_velocity: 1.0,
            kp_steering: 1.0,
            waypoint_reached_threshold: 2.0, // Reached if within 1m
        });
        parent_entity_commands.insert(ControllerLogic(controller));

        // Create Goal struct - need full state dimension even if only pos used
        // Pad with zeros or use current state for non-position elements
        let goal_state_sim = if let Some(bevy_goal) = initial_goal_bevy_pos {
            let sim_x = bevy_goal.x as f64;
            let sim_z = -bevy_goal.y as f64; // Bevy Vec2 Z component maps to -Sim Z
            // Create Sim goal state [sim_x, sim_z, target_yaw=0, target_v=0]
            // Could use current yaw/v if needed, but 0 is often fine for planner goal point
            StateVector::from_vec(vec![sim_x, sim_z, 0.0, 0.0])
        } else {
            // Default goal if none provided (in Sim coords)
            let mut default_goal = initial_state_sim.clone();
            default_goal[1] += 20.0; // Move 20m forward (increase sim_z)
            default_goal
        };

        parent_entity_commands.insert(GoalComponent(Goal {
            state: goal_state_sim, // Store Simulation goal state
            derivative: None,
        }));

        // --- Conditionally Insert PLANNER Components ---
        if add_planner {
            let planner = Box::new(GridPlanner {});
            parent_entity_commands.insert(PathPlannerLogic(planner));
            parent_entity_commands.insert(CurrentPath::default());
            parent_entity_commands.insert(DrawPathViz(true));
        } else {
            parent_entity_commands.insert(CurrentPath::default());
            parent_entity_commands.insert(DrawPathViz(false));
            // Maybe add DrawPathViz(false) or omit if default is false
        }
    }

    // --- Add Children ---
    parent_entity_commands.with_children(|parent| {
        // --- Car Body ---
        parent.spawn((
            Mesh3d(car_body_mesh.clone()),
            MeshMaterial3d(car_body_mat.clone()),
            Transform::from_xyz(0.0, car_body_y_offset, 0.0),
            Name::new(format!("{}_Body", config.name)),
        ));

        // --- Wheels ---
        let wheel_positions = [
            Vec3::new(
                front_axle_offset as f32,
                config.wheel_radius,
                wheel_track_width / 2.0,
            ), // FL
            Vec3::new(
                front_axle_offset as f32,
                config.wheel_radius,
                -wheel_track_width / 2.0,
            ), // FR
            Vec3::new(
                rear_axle_offset as f32,
                config.wheel_radius,
                wheel_track_width / 2.0,
            ), // RL
            Vec3::new(
                rear_axle_offset as f32,
                config.wheel_radius,
                -wheel_track_width / 2.0,
            ), // RR
        ];
        let wheel_align_rotation = Quat::from_rotation_x(PI / 2.0);

        parent.spawn((
            Mesh3d(wheel_mesh.clone()),
            MeshMaterial3d(wheel_mat.clone()),
            Transform::from_translation(wheel_positions[0]).with_rotation(wheel_align_rotation),
            SteeringWheel,
            RollingWheel,
            Name::new(format!("{}_FL", config.name)),
        ));
        parent.spawn((
            Mesh3d(wheel_mesh.clone()),
            MeshMaterial3d(wheel_mat.clone()),
            Transform::from_translation(wheel_positions[1]).with_rotation(wheel_align_rotation),
            SteeringWheel,
            RollingWheel,
            Name::new(format!("{}_FR", config.name)),
        ));
        parent.spawn((
            Mesh3d(wheel_mesh.clone()),
            MeshMaterial3d(wheel_mat.clone()),
            Transform::from_translation(wheel_positions[2]).with_rotation(wheel_align_rotation),
            RollingWheel,
            Name::new(format!("{}_RL", config.name)),
        ));
        parent.spawn((
            Mesh3d(wheel_mesh.clone()),
            MeshMaterial3d(wheel_mat.clone()),
            Transform::from_translation(wheel_positions[3]).with_rotation(wheel_align_rotation),
            RollingWheel,
            Name::new(format!("{}_RR", config.name)),
        ));
    });

    // Return the Entity ID
    parent_entity_commands.id()
}

// --- Remember to declare the vehicles module in src/lib.rs or src/main.rs ---
// pub mod vehicles { pub mod car_setup; }
