// helios_sim/src/simulation/core/components.rs

use bevy::prelude::*;
use helios_core::prelude::{Controller, ControlOutput, EstimationDynamics, Measurement};
use nalgebra::{Isometry3, Vector3};
use serde::Serialize;

// --- Wrapper Components for Core Traits ---

/// A Bevy component that wraps a pure `Dynamics` trait object.
#[derive(Component)]
pub struct EstimationDynamicsModel(pub Box<dyn EstimationDynamics>);

/// A Bevy component that wraps a pure `Measurement` trait object.
#[derive(Component)]
pub struct MeasurementModel(pub Box<dyn Measurement>);

// --- Controller Components ---

/// Wraps a `Controller` trait object. Mirrors `EstimationDynamicsModel` pattern.
/// Written once during `SceneBuildSet::ProcessControllers`; read every tick by `ControlPlugin`.
#[derive(Component)]
pub struct ControllerComponent(pub Box<dyn Controller>);

/// The output of the last `Controller::compute()` call.
/// Written by `SimulationSet::Control`; read by `SimulationSet::Actuation`.
#[derive(Component)]
pub struct ControlOutputComponent(pub ControlOutput);

// --- Agent State Components ---

/// The perfect, physics-driven ground truth state of an agent.
/// Written by the StateSync system; read by sensors and debugging.
#[derive(Component, Clone, Debug, Serialize)]
pub struct GroundTruthState {
    pub pose: Isometry3<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub linear_acceleration: Vector3<f64>,
    pub last_linear_velocity: Vector3<f64>,
}

impl Default for GroundTruthState {
    fn default() -> Self {
        Self {
            pose: Isometry3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            linear_acceleration: Vector3::zeros(),
            last_linear_velocity: Vector3::zeros(),
        }
    }
}
