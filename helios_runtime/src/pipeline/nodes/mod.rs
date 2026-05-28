//! Concrete [`PipelineNode`](crate::pipeline::PipelineNode) implementations.
//!
//! Naming convention: one node type per family, the node is generic over the
//! family trait object. `GaussianEstimatorNode` wraps any
//! `Box<dyn GaussianStateEstimator>` — EKF, UKF, ESKF, IF.

pub mod controller;
pub mod gaussian_estimator;
pub mod mock_oracle_estimator;
pub mod occupancy_grid;
pub mod path_follower;
pub mod search_planner;

pub use controller::ControllerNode;
pub use gaussian_estimator::{AidingHandler, GaussianEstimatorNode, TypedAidingHandler};
pub use occupancy_grid::OccupancyGridNode;
pub use path_follower::PathFollowerNode;
pub use search_planner::SearchPlannerNode;
