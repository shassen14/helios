//! Concrete [`PipelineNode`](crate::pipeline::PipelineNode) implementations,
//! one folder per algorithm family.
//!
//! Each family folder holds `node` (the adapter, generic over its `helios_core`
//! trait), optionally `input` (bus-input assembly), and `register` (factories
//! keyed by config `kind`), behind a `mod.rs` front-door that keeps the
//! submodules private and re-exports only what crosses the boundary — usually
//! just the `register` fn the [`AutonomyRegistry`](crate::assembly::AutonomyRegistry) calls.
//!
//! One node type per family: the node is generic over the family trait object,
//! so `RecursiveEstimatorNode` wraps any `Box<dyn GaussianStateEstimator>` (EKF,
//! UKF, …). `mocks` holds runtime-native test doubles that wrap no core
//! trait. `estimation` is not a family: it holds the components (filter,
//! dynamics, measurement) every estimator family assembles its node from;
//! `recursive_estimator` is the family built from them today. `estimate_relay`
//! is the node the estimate seam adds, forwarding the authoritative
//! estimator's state.

pub mod allocator;
pub mod combinators;
pub mod controller;
pub mod deproject;
pub mod estimate_relay;
pub mod estimation;
pub mod mocks;
pub mod occupancy_grid;
pub mod path_follower;
pub mod planner;
pub mod recursive_estimator;
pub mod teleop;
