// src/simulation/traits.rs

use crate::integrators::Integrator;
use crate::path_planning::error::PlanningError;
use crate::path_planning::grid_utils::{GridConfig, ObstacleGrid}; // Assuming integrators.rs is in src or crate root
use nalgebra::{DMatrix, DVector, Isometry3, Point3, Vector3};
use std::any::Any; // We might need this for flexible sensor data handling
use std::fmt::Debug;
use std::sync::Arc;

use super::components::CurrentPath;

// --- Core Type Aliases ---
// Using f64 for simulation precision, but could be generic if needed later.
pub type State = DVector<f64>;
pub type Control = DVector<f64>;

// --- Helper Structs/Enums ---

// Define IDs for clarity
pub type ObstacleId = u32;

/// Represents a geometric shape primitive.
#[derive(Clone, Debug)]
pub enum Shape {
    Sphere {
        radius: f64,
    },
    // Box defined by its center and half-extents (dimensions / 2)
    Box {
        half_extents: Vector3<f64>,
    },
    // Cylinder aligned with the local Y axis by default
    Cylinder {
        half_height: f64,
        radius: f64,
    },
    // Capsule aligned with the local Y axis by default
    Capsule {
        half_height: f64, // Height of the cylindrical part / 2
        radius: f64,
    },
    // For 2.5D scenarios (polygon on XY plane, extruded along Z) or 3D meshes
    // Using a library like `parry` for meshes is much more practical
    Mesh {
        // Placeholder - Real mesh data (vertices, indices) is complex
        // Consider using parry3d::shape::TriMesh or SharedShape here eventually.
        placeholder: String,
    },
    // Add Cone, Plane, Heightfield, etc. as needed
}

/// Represents a physical obstacle in the simulation environment.
#[derive(Clone, Debug)]
pub struct Obstacle {
    /// Unique identifier for this obstacle.
    pub id: ObstacleId,
    /// The position and orientation of the obstacle's shape in the world frame.
    pub pose: Isometry3<f64>,
    /// The geometric shape of the obstacle.
    pub shape: Shape,
    /// Optional: Is the obstacle static or dynamic? (Could influence planning/sensors)
    pub is_static: bool,
    // Optional: Add material properties, visual handles, etc.
}

/// Represents the target for a controller or planner.
/// Can include position, orientation, velocity, etc., depending on the context.
#[derive(Clone, Debug)]
pub struct Goal {
    /// The desired state vector (might be partially specified, e.g., only position).
    pub state: State,
    /// Optional: The desired rate of change of the state (for feedforward/trajectory following).
    pub derivative: Option<State>,
    // Add tolerance, priority, etc. if needed
}

type SensorId = u32; // Or a dedicated enum/struct

#[derive(Clone, Debug)]
pub enum SensorOutputData {
    // Add optional sensor ID
    Position {
        sensor_id: Option<SensorId>,
        value: Point3<f64>,
    },
    Pose {
        sensor_id: Option<SensorId>,
        value: Isometry3<f64>,
    },
    Velocity {
        sensor_id: Option<SensorId>,
        value: Vector3<f64>,
    },
    Imu {
        sensor_id: Option<SensorId>,
        acceleration: Vector3<f64>,
        angular_velocity: Vector3<f64>,
    },
    LidarScan {
        sensor_id: Option<SensorId>,
        origin: Isometry3<f64>,
        points: Vec<Point3<f64>>,
    },
    Gps {
        sensor_id: Option<SensorId>,
        latitude: f64,
        longitude: f64,
        altitude: f64,
        covariance: DMatrix<f64>,
    },
    Empty {
        // Maybe associated with a sensor ID that failed
        sensor_id: Option<SensorId>,
    },
    // Use Arc instead of Box here
    Custom {
        sensor_id: Option<SensorId>,
        data: Arc<dyn Any + Send + Sync>,
    },
}

// --- Core Simulation Traits ---

/// # Dynamics Trait
///
/// Represents the physics or kinematic model of an entity (robot, obstacle, etc.).
/// Defines how the entity's state evolves over time based on control inputs.
/// Implementations should be `Send + Sync` to be safely used across threads by Bevy.
pub trait Dynamics: Debug + Send + Sync {
    /// Returns the number of dimensions in the state vector `x`.
    fn get_state_dim(&self) -> usize;

    /// Returns the number of dimensions in the control input vector `u`.
    fn get_control_dim(&self) -> usize;

    /// Computes the time derivative of the state vector: `x_dot = f(x, u, t)`.
    /// This is the core function describing the system's behavior.
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`, which is `DVector<f64>`).
    /// * `u`: Current control input vector (`Control`, which is `DVector<f64>`).
    /// * `t`: Current simulation time (`Time`, which is `f64`).
    ///
    /// # Returns
    /// The time derivative of the state vector (`State`).
    fn get_derivatives(&self, x: &State, u: &Control, t: f64) -> State;

    /// Propagates the state forward in time using a numerical integrator.
    /// This method provides a default implementation using the `Integrator` trait.
    /// Specific dynamics models *could* override this if they have an analytical solution
    /// or a specialized integration scheme.
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`).
    /// * `u`: Current control input vector (`Control`). Assumed constant over `dt`.
    /// * `t`: Current simulation time (`Time`).
    /// * `dt`: Time step duration (`Time`). Must be non-negative.
    /// * `integrator`: A reference to an object implementing the `Integrator` trait (e.g., `RK4`).
    ///
    /// # Returns
    /// The estimated state vector at time `t + dt` (`State`).
    fn propagate(
        &self,
        x: &State,
        u: &Control,
        t: f64,
        dt: f64,
        integrator: &dyn Integrator<f64>,
    ) -> State {
        assert!(dt >= 0.0, "Dynamics::propagate: dt cannot be negative");

        // Ensure control input matches expected dimensions, providing zeros if not.
        // This prevents panics if the controller provides an incorrectly sized vector.
        let u_actual = if u.nrows() == self.get_control_dim() {
            u
        } else {
            // Log a warning or error here in a real application
            // eprintln!("Warning: Control input dimension mismatch for Dynamics propagation. Expected {}, got {}. Using zeros.", self.get_control_dim(), u.nrows());
            // Consider thread-safe logging if needed.
            thread_local! {
                static ZERO_CONTROL: std::cell::RefCell<Control> = std::cell::RefCell::new(Control::zeros(0));
            }
            &ZERO_CONTROL.with(|zc| {
                let mut zc_mut = zc.borrow_mut();
                if zc_mut.nrows() != self.get_control_dim() {
                    *zc_mut = Control::zeros(self.get_control_dim());
                }
                zc_mut.clone() // Borrow the correctly sized zero vector
            })
        };

        // Define the closure f(x, t) for the integrator, capturing the current control input 'u'.
        let func = |func_x: &State, func_t: f64| -> State {
            self.get_derivatives(func_x, u_actual, func_t)
        };

        // Perform the integration step.
        integrator.step(&func, x, t, t + dt)
    }

    /// (Optional) Calculates the Jacobian matrices of the dynamics function `f(x, u, t)`.
    /// Jacobian A = ∂f/∂x (how state derivatives change with state)
    /// Jacobian B = ∂f/∂u (how state derivatives change with control input)
    /// Useful for linear controllers (LQR), Kalman Filters (EKF), and stability analysis.
    /// Provides a default implementation that panics, forcing implementers to override if needed.
    ///
    /// # Arguments
    /// * `x`: State vector (`State`) at which to linearize.
    /// * `u`: Control input vector (`Control`) at which to linearize.
    /// * `t`: Simulation time (`Time`).
    ///
    /// # Returns
    /// A tuple `(A, B)` where `A` is an NxN matrix and `B` is an NxM matrix (N=state dim, M=control dim).
    fn calculate_jacobian(&self, x: &State, u: &Control, t: f64) -> (DMatrix<f64>, DMatrix<f64>) {
        // Default implementation: Indicate that Jacobians are not implemented for this model.
        // Consider returning Option<(...)> or Result<(...), Error> in a production setting.
        unimplemented!("Jacobian calculation not implemented for this dynamics model.");
    }

    /// (Optional) Calculates the feedforward control input required to achieve a desired state derivative.
    /// This attempts to solve `x_dot_desired = f(x, u_ff, t)` for `u_ff`.
    /// This is useful for feedforward control components. Not all models may have a
    /// straightforward or unique inverse.
    ///
    /// # Arguments
    /// * `x`: Current state vector (`State`).
    /// * `x_dot_desired`: The desired time derivative of the state vector (`State`).
    /// * `t`: Current simulation time (`Time`).
    ///
    /// # Returns
    /// * `Some(Control)`: The calculated feedforward control input `u_ff` if solvable.
    /// * `None`: If the feedforward input cannot be determined (e.g., model is not invertible,
    ///   no solution exists, or the feature is not implemented).
    fn calculate_feedforward_input(
        &self,
        x: &State,
        x_dot_desired: &State,
        t: f64,
    ) -> Option<Control> {
        // Default implementation: Feedforward calculation is not supported by default.
        None
    }
}

/// # Controller Trait
///
/// Represents the logic that determines the control input `u` for an entity.
/// Controllers typically aim to drive the entity's state towards a desired goal or follow a path.
/// They can use feedback (comparing current vs. desired state) and/or feedforward.
pub trait Controller: Debug + Send + Sync {
    /// Calculates the control input `u` based on the current situation.
    ///
    /// # Arguments
    /// * `current_state`: The current estimated or true state of the system (`State`).
    ///   (Usually the estimated state from an `Estimator` is used).
    /// * `goal`: The current desired state or target (`Goal`). This might represent a
    ///   single waypoint, a final destination, or a segment of a trajectory.
    /// * `dynamics`: (Optional) A reference to the entity's `Dynamics` model. This allows
    ///   the controller to potentially use model-based control, including feedforward
    ///   calculations via `calculate_feedforward_input` or using Jacobians.
    /// * `t`: Current simulation time (`Time`).
    ///
    /// # Returns
    /// The calculated control input vector (`Control`). It should match the dimension expected
    /// by the `Dynamics` model (`get_control_dim`).
    fn calculate_control(
        &mut self, // Allow controllers to have internal state (e.g., integral term in PID)
        current_state: &State,
        goal: &Goal,
        dynamics: Option<&dyn Dynamics>, // Provide access to the model
        path: Option<&mut CurrentPath>,
        t: f64,
    ) -> Control;
}

/// # PathPlanner Trait
///
/// Represents the logic for finding a sequence of states (a path) from a start to a goal,
/// usually avoiding obstacles.
pub trait Planner: Debug + Send + Sync {
    fn plan_path(
        &self,
        start: &State,
        goal: &Goal,
        obstacles: &[Obstacle],
        dynamics: Option<&dyn Dynamics>,
        grid_config: &GridConfig,
        obstacle_grid: &ObstacleGrid,
        t: f64,
    ) -> Result<Vec<State>, PlanningError>; // <-- Changed to Result
}

/// # Sensor Trait
///
/// Represents a sensor that perceives the environment or the entity's own state.
/// Simulates the measurement process, potentially adding noise, bias, or occlusions.
pub trait Sensor: Debug + Send + Sync {
    /// Simulates taking a sensor measurement.
    ///
    /// # Arguments
    /// * `true_state`: The ground truth state (`State`) of the entity the sensor is attached to.
    /// * `true_pose`: The ground truth pose (`Isometry3<f64>`) of the sensor itself in the world frame.
    /// * `obstacles`: A slice of `Obstacle` objects in the environment, needed for sensors
    ///   like Lidars that interact with the surroundings.
    /// * `t`: Current simulation time (`Time`).
    ///
    /// # Returns
    /// The simulated sensor measurement encapsulated in the `SensorOutputData` enum.
    fn sense(
        &self,
        true_state: &State,         // State of the *robot*
        true_pose: &Isometry3<f64>, // Pose of the *sensor*
        obstacles: &[Obstacle],
        t: f64,
    ) -> SensorOutputData;
}

/// # Estimator Trait
///
/// Represents state estimation algorithms (like EKF, Particle Filter, etc.).
/// Fuses sensor measurements and control inputs with a prediction model (often derived
/// from the `Dynamics` model) to estimate the entity's state.
pub trait Estimator: Debug + Send + Sync {
    /// Performs the prediction step based on control input and dynamics.
    /// Updates the internal predicted state and covariance.
    ///
    /// # Arguments
    /// * `control`: The control input `u` applied during the interval `dt` (`Control`).
    /// * `dynamics`: A reference to the entity's `Dynamics` model.
    /// * `dt`: The time elapsed (`f64`) since the last prediction.
    fn predict(&mut self, control: &Control, dynamics: &dyn Dynamics, dt: f64);

    /// Performs the update step based on a single sensor measurement.
    /// Corrects the predicted state using the measurement.
    ///
    /// # Arguments
    /// * `measurement`: The sensor measurement received (`SensorOutputData`). The estimator
    ///   needs to handle the relevant variants and sensor_id within this enum.
    fn update(&mut self, measurement: &SensorOutputData);

    /// Returns the current best estimate of the state after predict/update cycles.
    fn get_current_estimate(&self) -> State;

    /// Optional: Returns the current uncertainty estimate (e.g., covariance matrix).
    fn get_covariance(&self) -> Option<DMatrix<f64>> {
        None
    }
}
