//! Concrete [`PipelineNode`](crate::pipeline::PipelineNode) implementations.
//!
//! Naming convention: one node type per family, the node is generic over the
//! family trait object. `GaussianEstimatorNode` wraps any
//! `Box<dyn GaussianStateEstimator>` — EKF, UKF, ESKF, IF.

pub mod gaussian_estimator;

pub use gaussian_estimator::{AidingHandler, GaussianEstimatorNode, TypedAidingHandler};
