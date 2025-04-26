// src/rendering/systems.rs

use crate::simulation::components::*;
use crate::simulation::traits::{SensorOutputData, Shape};
use crate::simulation::utils::*;
use bevy::prelude::*;
use std::f32::consts::PI; // For cuboid rotation if needed

// =========================================================================
// == True State Visualization ==
// =========================================================================
pub fn draw_true_state_system(
    mut query: Query<(&TrueState, &DrawTrueStateViz, &mut Visibility, &Transform)>, // Added Transform
    mut gizmos: Gizmos,
) {
    for (_true_state, viz_toggle, mut visibility, _transform) in query.iter_mut() {
        *visibility = if viz_toggle.0 {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };

        // Optional: Default gizmo if no mesh (Example: Red Sphere)
        // if viz_toggle.0 /* && !entity_has_mesh */ {
        //     if _true_state.0.len() >= 3 {
        //         gizmos.sphere(transform.translation, 0.2, Color::srgb(1.0, 0.1, 0.1)); // Pass position, radius, color
        //     }
        // }
        // NOTE: The Gizmos API seems to have changed again. Let's adapt to the builder pattern if sphere takes Isometry.
        // Let's assume for now that the entity HAS a mesh and this system just controls visibility.
        // If we need default drawing, we need to adapt to the exact gizmo signature.
    }
}

// =========================================================================
// == Estimated State Visualization ==
// =========================================================================
pub fn draw_estimated_state_system(
    query: Query<(&EstimatedState, &Transform, &DrawEstimatedStateViz)>,
    mut gizmos: Gizmos,
) {
    for (estimated_state, transform, viz_toggle) in query.iter() {
        if !viz_toggle.0 {
            continue;
        }

        let state_vec = &estimated_state.state;
        let ghost_pos: Vec3;
        let ghost_rot: Quat;

        // --- Determine Ghost Pose --- (logic unchanged)
        // TODO: Check for state_vec and what are the items inside / ordering
        if state_vec.len() >= 7 {
            ghost_pos = Vec3::new(
                state_vec[0] as f32,
                state_vec[1] as f32,
                state_vec[2] as f32,
            );
            ghost_rot = Quat::from_xyzw(
                state_vec[3] as f32,
                state_vec[4] as f32,
                state_vec[5] as f32,
                state_vec[6] as f32,
            );
        } else if state_vec.len() >= 3 {
            ghost_pos = Vec3::new(
                state_vec[0] as f32,
                state_vec[1] as f32,
                transform.translation.z,
            );
            ghost_rot = Quat::from_rotation_z(state_vec[2] as f32);
        } else {
            ghost_pos = transform.translation;
            ghost_rot = transform.rotation;
        }

        // --- Create Isometry for Gizmo ---
        let ghost_isometry: Isometry3d = Isometry3d::new(ghost_pos, ghost_rot);

        // --- Draw Ghost Pose Gizmo ---
        gizmos.sphere(ghost_isometry, 0.3, Color::srgba(0.1, 1.0, 0.1, 0.5)); // Pass Isometry3d

        // --- Draw Coordinate Axes --- (logic unchanged)
        let axis_length = 0.5;
        gizmos.line(
            ghost_pos,
            ghost_pos + ghost_rot * Vec3::X * axis_length,
            Color::srgba(1.0, 0.1, 0.1, 0.8),
        );
        gizmos.line(
            ghost_pos,
            ghost_pos + ghost_rot * Vec3::Y * axis_length,
            Color::srgba(0.1, 1.0, 0.1, 0.8),
        );
        gizmos.line(
            ghost_pos,
            ghost_pos + ghost_rot * Vec3::Z * axis_length,
            Color::srgba(0.1, 0.1, 1.0, 0.8),
        );

        // --- Draw Covariance Ellipsoid ---
        if let Some(covariance) = &estimated_state.covariance {
            if covariance.nrows() >= 3 && covariance.ncols() >= 3 {
                // ... eigenvalue calculation ...

                // Simplified sphere scaled by trace
                let pos_cov = covariance.fixed_view::<3, 3>(0, 0);
                let trace = pos_cov.trace();
                let uncertainty_radius = (trace / 3.0).sqrt().max(0.01) as f32;
                let scale_factor = 2.0;

                // Create Isometry for the sphere (position only)
                let uncertainty_isometry: Isometry3d =
                    transform_to_isometry3d(&Transform::from_translation(ghost_pos));

                gizmos.sphere(
                    uncertainty_isometry, // Pass Isometry3d
                    scale_factor * uncertainty_radius,
                    Color::srgba(1.0, 1.0, 0.0, 0.15),
                );
            }
        }
    }
}

// =========================================================================
// == Path Visualization ==
// =========================================================================
pub fn draw_path_system(
    query: Query<(&CurrentPath, &Transform, &DrawPathViz)>,
    mut gizmos: Gizmos,
) {
    for (path, transform, viz_toggle) in query.iter() {
        if !viz_toggle.0 || path.0.len() < 2 {
            continue;
        }

        let points: Vec<Vec3> = path
            .0
            .iter()
            .filter_map(|state_vec| {
                if state_vec.len() >= 2 {
                    let z_coord = if state_vec.len() >= 3 {
                        state_vec[2] as f32
                    } else {
                        transform.translation.z
                    };
                    Some(Vec3::new(state_vec[0] as f32, state_vec[1] as f32, z_coord))
                } else {
                    None
                }
            })
            .collect();

        if points.len() >= 2 {
            gizmos.linestrip(points, Color::srgb(0.0, 1.0, 1.0)); // Cyan path
        }
    }
}

// =========================================================================
// == Sensor Data Visualization ==
// =========================================================================
pub fn draw_sensor_data_system(
    mut events: EventReader<SensorOutputEvent>,
    viz_query: Query<&DrawSensorViz>,
    // Query for Transform needed if drawing relative to entity
    // transform_query: Query<&Transform>, // Example if needed for GPS viz
    mut gizmos: Gizmos,
) {
    for event in events.read() {
        if let Ok(viz_settings) = viz_query.get(event.entity) {
            match &event.data {
                SensorOutputData::LidarScan {
                    sensor_id: _,
                    origin: _,
                    points,
                } => {
                    if viz_settings.lidar {
                        for point in points {
                            let pos = Vec3::new(point.x as f32, point.y as f32, point.z as f32);
                            gizmos.sphere(
                                Isometry3d::from_translation(pos),
                                0.05,                       // Radius
                                Color::srgb(1.0, 1.0, 0.0), // Yellow points
                            );
                        }
                    }
                }
                SensorOutputData::Gps {
                    sensor_id: _,
                    latitude: _,
                    longitude: _,
                    altitude,
                    covariance,
                } => {
                    if viz_settings.gps_uncertainty {
                        // Visualization for GPS uncertainty often better tied to EstimatedState viz
                        // as the raw GPS data doesn't inherently have the filtered position.
                        // We can draw the covariance sphere from the event if available.
                        if covariance.nrows() >= 3 && covariance.ncols() >= 3 {
                            // Example: Draw uncertainty sphere at the **reported GPS position** (if available/sensible)
                            // This needs the actual reported position (e.g. from a Position variant or calculated from lat/lon)
                            // Placeholder: Assume 'altitude' roughly maps to Z.
                            // let gps_reported_pos = Vec3::new(0.0, 0.0, *altitude as f32); // Need actual X, Y!

                            // let pos_cov = covariance.fixed_slice::<3, 3>(0, 0);
                            // let trace = pos_cov.trace();
                            // let uncertainty_radius = (trace / 3.0).sqrt().max(0.01) as f32;
                            // gizmos.sphere(Transform::from_translation(gps_reported_pos), 2.0 * uncertainty_radius, Color::srgba(0.5, 0.0, 0.5, 0.1));
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

// =========================================================================
// == Obstacle Visualization ==
// =========================================================================
pub fn draw_obstacle_system(
    query: Query<(&ObstacleComponent, &GlobalTransform)>,
    mut gizmos: Gizmos,
) {
    for (obs_comp, global_transform) in query.iter() {
        let transform = global_transform.compute_transform();
        // Use SRGB colors
        let color = if obs_comp.0.is_static {
            Color::srgb(0.5, 0.5, 0.5)
        } else {
            Color::srgb(0.3, 0.3, 0.3)
        };

        let obstacle_isometry: Isometry3d = transform_to_isometry3d(&transform);

        match &obs_comp.0.shape {
            Shape::Sphere { radius } => {
                // Pass Transform for position/rotation (though rotation doesn't affect sphere visual)
                gizmos.sphere(transform_to_isometry3d(&transform), *radius as f32, color);
            }
            Shape::Box { half_extents } => {
                let size = Vec3::new(
                    (half_extents.x * 2.0) as f32,
                    (half_extents.y * 2.0) as f32,
                    (half_extents.z * 2.0) as f32,
                );
                let cuboid_transform = Transform {
                    translation: transform.translation,
                    rotation: transform.rotation,
                    scale: size, // Apply size via scale
                };
                // Gizmos::cuboid now only takes transform and color in 0.15.3
                gizmos.cuboid(cuboid_transform, color);
            }
            Shape::Cylinder {
                half_height,
                radius,
            } => {
                // Still needs approximation or mesh spawning
                let half_height_f32 = *half_height as f32;
                let radius_f32 = *radius as f32;
                let pos = transform.translation;
                let rot = transform.rotation;
                let p1 = pos + rot * Vec3::Y * half_height_f32;
                let p2 = pos - rot * Vec3::Y * half_height_f32;
                gizmos.line(p1, p2, color);

                // Create Isometry for circles (aligned with Y axis, rotated by transform rotation)
                let circle_rot_offset = Quat::from_rotation_x(PI / 2.0); // Rotate circle to face XY plane
                let circle1_isometry = transform_to_isometry3d(
                    &Transform::from_translation(p1).with_rotation(rot * circle_rot_offset),
                );
                let circle2_isometry = transform_to_isometry3d(
                    &Transform::from_translation(p2).with_rotation(rot * circle_rot_offset),
                );

                gizmos.circle(circle1_isometry, radius_f32, color);
                gizmos.circle(circle2_isometry, radius_f32, color);
            }
            Shape::Capsule {
                half_height,
                radius,
            } => {
                let half_height_f32 = *half_height as f32;
                let radius_f32 = *radius as f32;
                let pos = transform.translation;
                let rot = transform.rotation;
                let p1 = pos + rot * Vec3::Y * half_height_f32;
                let p2 = pos - rot * Vec3::Y * half_height_f32;
                gizmos.line(p1, p2, color);
                // Create Isometry for end caps (position only)
                let cap1_isometry = transform_to_isometry3d(&Transform::from_translation(p1));
                let cap2_isometry = transform_to_isometry3d(&Transform::from_translation(p2));
                gizmos.sphere(cap1_isometry, radius_f32, color);
                gizmos.sphere(cap2_isometry, radius_f32, color);
            }
            Shape::Mesh { placeholder: _ } => {
                // Placeholder: Draw oriented bounding box? Or just axes?
                // Cuboid needs size. Let's draw axes instead.
                let axis_length = 0.5; // Smaller axes for meshes maybe
                gizmos.line(
                    transform.translation,
                    transform.translation + transform.rotation * Vec3::X * axis_length,
                    color,
                );
                gizmos.line(
                    transform.translation,
                    transform.translation + transform.rotation * Vec3::Y * axis_length,
                    color,
                );
                gizmos.line(
                    transform.translation,
                    transform.translation + transform.rotation * Vec3::Z * axis_length,
                    color,
                );
            }
        }
    }
}

// =========================================================================
// == Goal Visualization ==
// =========================================================================
pub fn draw_goal_system(
    query: Query<(&GoalComponent, &Transform, &DrawGoalViz)>,
    mut gizmos: Gizmos,
) {
    for (goal_comp, transform, viz_toggle) in query.iter() {
        if !viz_toggle.0 {
            continue;
        }
        let goal_state = &goal_comp.0.state;

        if goal_state.len() >= 2 {
            let goal_pos = Vec3::new(
                goal_state[0] as f32,
                goal_state[1] as f32,
                if goal_state.len() >= 3 {
                    goal_state[2] as f32
                } else {
                    transform.translation.z
                },
            );
            // Create Isometry for sphere (position only)
            let goal_isometry = transform_to_isometry3d(&Transform::from_translation(goal_pos));
            gizmos.sphere(goal_isometry, 0.4, Color::srgb(1.0, 0.4, 0.0)); // OrangeRed
            gizmos.line(
                goal_pos + Vec3::Y * 0.5,
                goal_pos - Vec3::Y * 0.5,
                Color::srgb(1.0, 0.4, 0.0),
            );
        }
    }
}

// --- Example System Setup Function (Reminder for main.rs) ---
/*
fn setup_rendering_systems(app: &mut App) {
    app
        // ... Add systems to Update or PostUpdate ...
}
*/
