//! Concrete [`PipelineNode`](crate::pipeline::PipelineNode) implementations,
//! one folder per algorithm family.
//!
//! Each family folder holds `node` (the adapter, generic over its `helios_core`
//! trait), optionally `input` (bus-input assembly), and `register` (factories
//! keyed by config `kind`), behind a `mod.rs` front-door that keeps the
//! submodules private and re-exports only what crosses the boundary — usually
//! just the `register` fn the [`AutonomyRegistry`](crate::registry) calls.
//!
//! One node type per family: the node is generic over the family trait object,
//! so `GaussianEstimatorNode` wraps any `Box<dyn GaussianStateEstimator>` (EKF,
//! UKF, ESKF, IF). `mocks` holds runtime-native test doubles that wrap no core
//! trait.

pub mod allocator;
pub mod combinators;
pub mod controller;
pub mod gaussian_estimator;
pub mod mocks;
pub mod occupancy_grid;
pub mod path_follower;
pub mod planner;
