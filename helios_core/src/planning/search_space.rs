// helios_core/src/planning/search_space.rs
//
// SearchSpace trait: abstracts grid vs. continuous environments for planners.

use nalgebra::Vector2;

/// An environment that a search-based planner can query.
pub trait SearchSpace: Send + Sync {
    /// Returns true if the given ENU world 2D position is unoccupied / traversable.
    fn is_free(&self, pos: Vector2<f64>) -> bool;

    /// Returns true if the position is within the representable bounds of this space.
    fn is_in_bounds(&self, pos: Vector2<f64>) -> bool;

    /// Project a position that is outside the bounds to the nearest in-bounds position.
    /// Returns `None` only if the space is degenerate (zero size).
    fn project_to_bounds(&self, pos: Vector2<f64>) -> Option<Vector2<f64>>;

    /// Grid resolution in meters, if this space has a fixed cell size. `None` for continuous.
    fn resolution(&self) -> Option<f64>;
}
