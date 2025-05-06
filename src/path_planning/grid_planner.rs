// src/path_planning/grid_planner.rs
use super::dijkstra;
use super::error::PlanningError;
// Use the dijkstra module
use super::grid_utils::{GridConfig, GridCoord, ObstacleGrid};
use crate::simulation::components::StateVector;
use crate::simulation::traits::{Dynamics, Goal, Obstacle, Planner, Shape, State}; // Import traits and Obstacle/Shape
use bevy::prelude::*; // Use Vec2 from prelude

#[derive(Debug)] // Add Debug derive
pub struct GridPlanner {
    // Configuration is now expected as a Resource
    // No internal map stored here - generated on the fly or read from resource
}

impl GridPlanner {
    /// Generates neighbors for a grid cell, checking bounds and obstacles.
    fn get_grid_neighbors(
        &self,
        node: &GridCoord,
        grid_config: &GridConfig,
        obstacle_grid: &ObstacleGrid,
    ) -> Vec<(GridCoord, f64)> {
        // Return type matches Dijkstra's IT
        let mut neighbors = Vec::with_capacity(8); // Max 8 neighbors
        // Define potential moves (4-way or 8-way)
        let directions = [
            (0, 1),
            (0, -1),
            (1, 0),
            (-1, 0), // Cardinal
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1), // Diagonal
        ];

        for (dx, dy) in directions {
            let next_coord = GridCoord {
                x: node.x + dx,
                y: node.y + dy,
            };

            // Check bounds and occupation
            if grid_config.is_in_bounds(next_coord) && !obstacle_grid.is_occupied(next_coord) {
                let move_cost = if dx.abs() + dy.abs() == 2 { 1.414 } else { 1.0 }; // sqrt(2) for diagonal
                neighbors.push((next_coord, move_cost));
            }
        }
        neighbors
    }

    /// Updates the obstacle grid based on world obstacles.
    /// Inefficient: Should ideally only update changed areas or use spatial partitioning.
    fn update_obstacle_grid(
        &self,
        obstacle_grid: &mut ObstacleGrid,
        grid_config: &GridConfig,
        obstacles: &[Obstacle], // From trait method input
                                // Optionally pass Rapier context for more accurate shape checks
    ) {
        obstacle_grid.clear(); // Clear previous obstacles

        for obstacle in obstacles {
            // Simple AABB rasterization for now
            // TODO: Implement more accurate shape rasterization (sphere, box, cylinder)
            // Needs conversion from Isometry3 to 2D pose and shape projection.
            let obs_pos_2d = Vec2::new(
                obstacle.pose.translation.x as f32,
                obstacle.pose.translation.z as f32,
            );

            // Approximate bounds based on shape type (very rough)
            let half_extents = match &obstacle.shape {
                Shape::Sphere { radius } => Vec2::splat(*radius as f32),
                Shape::Box { half_extents } => {
                    Vec2::new(half_extents.x as f32, half_extents.z as f32)
                } // Use X/Z
                Shape::Cylinder { radius, .. } => Vec2::splat(*radius as f32),
                Shape::Capsule { radius, .. } => Vec2::splat(*radius as f32),
                Shape::Mesh { .. } => Vec2::splat(1.0), // Placeholder extent for mesh
            };

            let min_world = obs_pos_2d - half_extents;
            let max_world = obs_pos_2d + half_extents;

            if let (Some(min_grid), Some(max_grid)) = (
                grid_config.world_to_grid(min_world),
                grid_config.world_to_grid(max_world),
            ) {
                for x in min_grid.x..=max_grid.x {
                    for y in min_grid.y..=max_grid.y {
                        obstacle_grid.set_occupied(GridCoord { x, y }, true);
                    }
                }
            }
        }
    }
}

impl Planner for GridPlanner {
    fn plan_path(
        &self,
        start_state: &State,
        goal: &Goal,
        obstacles: &[Obstacle], // <-- Add parameter back
        _dynamics: Option<&dyn Dynamics>,
        grid_config: &GridConfig,
        obstacle_grid: &ObstacleGrid,
        _t: f64,
    ) -> Result<Vec<State>, PlanningError> {
        // Ensure return type is Result

        // Comment explaining why `obstacles` might be ignored here:
        // Note: This grid-based planner primarily relies on the pre-computed
        // `obstacle_grid`. The `obstacles: &[Obstacle]` argument is part of
        // the generic Planner trait signature but might not be directly used
        // in this specific implementation, assuming the grid is up-to-date.
        // Other planners (like RRT*) *will* use the `obstacles` list directly.

        if obstacle_grid.width == 0 || obstacle_grid.height == 0 {
            return Err(PlanningError::GridNotInitialized);
        }

        // ... rest of the GridPlanner logic using grid_config and obstacle_grid ...
        // ... ensure all return paths yield Ok(path) or Err(PlanningError::...) ...

        let start_pos_sim = Vec2::new(start_state[0] as f32, start_state[1] as f32);
        let goal_pos_sim = Vec2::new(goal.state[0] as f32, goal.state[1] as f32);

        let start_node = grid_config
            .world_to_grid(start_pos_sim)
            .ok_or(PlanningError::StartOutsideGrid)?;
        let goal_node = grid_config
            .world_to_grid(goal_pos_sim)
            .ok_or(PlanningError::GoalOutsideGrid)?;

        if obstacle_grid.is_occupied(start_node) {
            return Err(PlanningError::StartOccupied);
        }
        if obstacle_grid.is_occupied(goal_node) {
            return Err(PlanningError::GoalOccupied);
        }

        let mut neighbor_fn =
            |node: &GridCoord| self.get_grid_neighbors(node, grid_config, obstacle_grid);
        let is_goal_fn = |node: &GridCoord| -> bool { *node == goal_node };

        dijkstra::plan(&start_node, is_goal_fn, &mut neighbor_fn)
            .map(|grid_path| {
                grid_path
                    .into_iter()
                    .map(|grid_coord| {
                        let world_pos_sim = grid_config.grid_to_world_center(grid_coord); // (sim_x, sim_z)
                        // Create state vector [sim_x, sim_z, yaw=0, v=0] for path points
                        StateVector::from_vec(vec![
                            world_pos_sim.x as f64,
                            world_pos_sim.y as f64,
                            0.0,
                            0.0,
                        ])
                    })
                    .collect()
            })
            .ok_or(PlanningError::NoPathFound)
    }
}
