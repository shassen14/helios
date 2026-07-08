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
