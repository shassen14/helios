// src/path_planning/grid_utils.rs
use bevy::prelude::*;
use nalgebra::Vector2;
use std::{
    collections::HashMap,
    hash::{Hash, Hasher},
};

/// Represents a 2D index in the grid map.
#[derive(Debug, Copy, Clone, Eq)]
pub struct GridCoord {
    pub x: isize,
    pub y: isize, // Corresponds to world Z usually
}

// Implement Hash and PartialEq for use in HashMap keys
impl Hash for GridCoord {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
    }
}

impl PartialEq for GridCoord {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

/// Configuration for the planning grid.
#[derive(Resource, Clone, Debug)] // Make it a Resource for easy access
pub struct GridConfig {
    pub resolution: f32,    // World units per grid cell (e.g., 0.5 meters)
    pub width: usize,       // Number of cells in world X direction
    pub height: usize,      // Number of cells in world Z direction
    pub world_origin: Vec2, // World coordinate (X, Z) of grid cell (0, 0)
}

impl GridConfig {
    /// Converts a world position (X, Z) to grid coordinates.
    pub fn world_to_grid(&self, world_pos: Vec2) -> Option<GridCoord> {
        let relative_pos = world_pos - self.world_origin;
        let grid_x = (relative_pos.x / self.resolution).floor() as isize;
        let grid_y = (relative_pos.y / self.resolution).floor() as isize; // World Z is grid Y

        if grid_x >= 0
            && grid_x < self.width as isize
            && grid_y >= 0
            && grid_y < self.height as isize
        {
            Some(GridCoord {
                x: grid_x,
                y: grid_y,
            })
        } else {
            None // Position is outside the grid bounds
        }
    }

    /// Converts grid coordinates to the center world position (X, Z) of the cell.
    pub fn grid_to_world_center(&self, grid_coord: GridCoord) -> Vec2 {
        self.world_origin
            + Vec2::new(
                (grid_coord.x as f32 + 0.5) * self.resolution,
                (grid_coord.y as f32 + 0.5) * self.resolution, // Grid Y is world Z
            )
    }

    /// Checks if grid coordinates are within the defined bounds.
    pub fn is_in_bounds(&self, coord: GridCoord) -> bool {
        coord.x >= 0
            && coord.x < self.width as isize
            && coord.y >= 0
            && coord.y < self.height as isize
    }
}

/// Represents the grid map for path planning.
/// 0 = free, 1 = occupied.
#[derive(Resource, Clone, Debug)]
pub struct ObstacleGrid {
    pub cells: Vec<Vec<u8>>,
    pub width: usize,
    pub height: usize,
}

impl ObstacleGrid {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            cells: vec![vec![0; height]; width],
            width,
            height,
        }
    }

    pub fn is_occupied(&self, coord: GridCoord) -> bool {
        if coord.x < 0
            || coord.x >= self.width as isize
            || coord.y < 0
            || coord.y >= self.height as isize
        {
            true // Out of bounds is considered occupied
        } else {
            self.cells[coord.x as usize][coord.y as usize] == 1
        }
    }

    pub fn set_occupied(&mut self, coord: GridCoord, occupied: bool) {
        if coord.x >= 0
            && coord.x < self.width as isize
            && coord.y >= 0
            && coord.y < self.height as isize
        {
            self.cells[coord.x as usize][coord.y as usize] = if occupied { 1 } else { 0 };
        }
    }

    pub fn clear(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                self.cells[x][y] = 0;
            }
        }
    }
}
