// src/simulation/components.rs

use crate::simulation::traits::{
    Control, Controller, Dynamics, Estimator, Goal, Obstacle, Planner, Sensor, SensorOutputData,
    State,
};
use bevy::prelude::*;
use nalgebra::{DMatrix, Isometry3}; // Add DMatrix for covariance
use std::fmt;

// --- Type Aliases from Traits (for convenience within this module/project) ---
pub type StateVector = State;
pub type ControlVector = Control;

// =========================================================================
// == Core State Components ==
// Describe the physical state and intentions of an entity.
// =========================================================================

/// # TrueState
/// Holds the ground truth state vector of the entity (e.g., position, orientation, velocity).
/// This is the "real" state within the simulation's physics.
///
/// **Data:** `pub StateVector` (nalgebra::DVector<f64>) - The state vector itself.
/// **Simulation Systems:**
///    - *Write:* `dynamics_system` calculates and updates this based on physics/kinematics.
///    - *Read:* `sensor_system` reads this to generate simulated sensor readings.
///    - *Read:* Can be used by planners/controllers if perfect state info is assumed (debugging).
/// **Rendering Systems:**
///    - *Read:* `draw_true_state_system` reads this (along with `Transform`) to render the entity's primary visual representation (e.g., car model, drone mesh). Requires `DrawTrueStateViz(true)`.
/// **Input Systems:** Generally not directly modified by input.
#[derive(Component, Debug, Clone)]
pub struct TrueState(pub StateVector);

/// # EstimatedState
/// Holds the estimated state vector and potentially its uncertainty (covariance).
/// This represents the entity's *belief* about its state, derived from sensor fusion.
///
/// **Data:**
///    - `pub state: StateVector` - The current best estimate of the state.
///    - `pub covariance: Option<DMatrix<f64>>` - The uncertainty associated with the estimate.
/// **Simulation Systems:**
///    - *Write:* `estimator_system` calculates and updates this based on sensor events, control inputs, and the estimator logic.
///    - *Read:* `controller_system` typically reads this estimated state to calculate control commands.
///    - *Read:* `planner_system` might use this as the starting point for planning.
/// **Rendering Systems:**
///    - *Read:* `draw_estimated_state_system` reads this (along with `Transform`) to render a visual representation of the estimate (e.g., a ghost model, uncertainty ellipsoid). Requires `DrawEstimatedStateViz(true)`.
/// **Input Systems:** Not directly modified by input.
#[derive(Component, Debug, Clone)]
pub struct EstimatedState {
    pub state: StateVector,
    pub covariance: Option<DMatrix<f64>>,
}

/// # ControlInput
/// Holds the latest calculated control input vector (`u`) to be applied to the entity's dynamics.
///
/// **Data:** `pub ControlVector` - The control input vector.
/// **Simulation Systems:**
///    - *Write:* `controller_system` calculates and writes the desired control input here.
///    - *Read:* `dynamics_system` reads this input to calculate state derivatives (`f(x, u, t)`).
///    - *Read:* `estimator_system` reads this as the input `u` for the prediction step.
/// **Rendering Systems:** Could potentially visualize control inputs (e.g., steering angle, throttle bar).
/// **Input Systems:** Manual control input systems (e.g., keyboard driving) could *write* to this component, overriding the autonomous controller.
#[derive(Component, Debug, Clone, Default)]
pub struct ControlInput(pub ControlVector);

#[derive(Component, Debug, Default, Clone)]
pub struct CommandedForces {
    // Forces and torques are typically applied in the body's local frame for vehicles
    // or in the world frame (Avian supports both with different components/methods)
    // Let's assume world frame forces/torques for simplicity first, or local if easier for vehicle model
    pub force_world: Vec3,  // Total force to apply in Bevy world coordinates
    pub torque_world: Vec3, // Total torque to apply in Bevy world coordinates (around world axes)
}

/// # GoalComponent
/// Represents the current objective or target for an autonomous agent.
/// Wraps the `Goal` struct from `traits.rs`.
///
/// **Data:** `pub Goal` - Contains target state, optional target derivatives, etc.
/// **Simulation Systems:**
///    - *Write:* `input_system` (e.g., mouse click) or higher-level mission logic systems can write/update the goal.
///    - *Read:* `planner_system` reads this as the target destination for path planning.
///    - *Read:* `controller_system` reads this to determine the desired state/trajectory to follow.
/// **Rendering Systems:**
///    - *Read:* `draw_goal_system` reads this to visualize the target location/state. Requires `DrawGoalViz(true)`.
/// **Input Systems:**
///    - *Write:* Typically written by input (e.g., mouse click sets `goal.state`).
#[derive(Component, Debug, Clone)]
pub struct GoalComponent(pub Goal);

/// # CurrentPath
/// Holds the sequence of states representing the path the entity is currently trying to follow.
///
/// **Data:** `pub Vec<StateVector>` - A list of waypoints or states defining the path.
/// **Simulation Systems:**
///    - *Write:* `planner_system` calculates and writes the path here after successful planning.
///    - *Read:* `controller_system` reads this path to determine intermediate desired states/velocities for path following.
/// **Rendering Systems:**
///    - *Read:* `draw_path_system` reads this to visualize the planned path as a line strip. Requires `DrawPathViz(true)`.
/// **Input Systems:** Not typically modified by direct input.
#[derive(Component, Debug, Clone, Default)]
pub struct CurrentPath(pub Vec<StateVector>);

// =========================================================================
// == Logic Components (Holding Trait Objects) ==
// Contain the behavioural logic for different aspects of simulation.
// =========================================================================

/// # Addressing Multiple Dynamics Models per Entity
///
/// A common robotics scenario involves combining multiple models (e.g., longitudinal dynamics
/// + lateral kinematics for a car). There are several ways to handle this in ECS:
///
/// 1.  **Composite Dynamics Struct (Recommended for Tightly Coupled Models):**
///     - Create a specific struct (e.g., `struct AckermannVehicleDynamics`) that *itself*
///       implements the `Dynamics` trait.
///     - Internally, this struct holds instances of the sub-models (e.g., longitudinal, lateral).
///     - Its `get_derivatives` method orchestrates calls to the sub-models and combines
///       their results appropriately, managing the interactions and the full state vector.
///     - **Bevy Component:** You still only need *one* `DynamicsModel` component holding
///       this composite `Box<dyn Dynamics>`.
///     - **Pros:** Encapsulates complex interactions cleanly. Predictable behavior.
///     - **Cons:** Requires writing specific composite structs for each desired combination.
///
/// 2.  **Multiple Specific Components:**
///     - Define components like `LongitudinalDynamics(Box<dyn Dynamics>)`,
///       `LateralKinematics(Box<dyn Dynamics>)`.
///     - **Bevy Component:** Attach multiple such components to the entity.
///     - **Pros:** More flexible "mix-and-match" composition.
///     - **Cons:** Requires a complex `DynamicsSystem` that knows how to find these specific
///       components, partition the state/control vectors, call the correct models for the
///       correct state variables, and merge the results for integration. Error-prone.
///       Less clear how `propagate` or `calculate_jacobian` would work cohesively.
///
/// 3.  **Tagged Components Collection:**
///     - `DynamicsAspects(Vec<(String, Box<dyn Dynamics>)>)` or similar.
///     - **Bevy Component:** One component holding a collection of tagged models.
///     - **Pros:** Very flexible.
///     - **Cons:** Similar complexity issues as option 2 for the system implementation.
///
/// **Conclusion:** For tightly coupled models like car dynamics, the **Composite Dynamics Struct (1)**
/// is generally the most robust and recommended approach. We will assume this pattern
/// and use a single `DynamicsModel` component per entity that requires physics simulation.
/// The *implementation* inside the `Box<dyn Dynamics>` handles the complexity.

/// # DynamicsModel
/// Holds the entity's dynamics implementation (the `f(x, u, t)` function and related methods).
/// Contains a `Box<dyn Dynamics>` allowing for different physics/kinematic models.
/// Assumes the "Composite Dynamics Struct" pattern if multiple coupled models are needed.
///
/// **Data:** `pub Box<dyn Dynamics>` - The trait object holding the dynamics logic.
/// **Simulation Systems:**
///    - *Read:* `dynamics_system` calls `.get_derivatives()`, `.propagate()`.
///    - *Read:* `estimator_system` often needs access (via query) to call `.get_derivatives()` or `.calculate_jacobian()` for prediction.
///    - *Read:* `controller_system` might query this to use `.calculate_feedforward_input()` or `.calculate_jacobian()` for model-based control.
///    - *Read:* `planner_system` might query this for kinodynamic planning.
/// **Rendering/Input Systems:** Not directly used.
#[derive(Component)]
pub struct DynamicsModel(pub Box<dyn Dynamics>);

/// # ControllerLogic
/// Holds the entity's control law implementation.
/// Contains a `Box<dyn Controller>`. Responsible for calculating `ControlInput`.
///
/// **Data:** `pub Box<dyn Controller>` - The trait object holding the controller logic.
/// **Simulation Systems:**
///    - *Read:* `controller_system` calls `.calculate_control()` on this object, potentially providing
///              `EstimatedState`, `GoalComponent`, and `DynamicsModel`. Needs `&mut` access if the
///               controller has internal state (e.g., PID integral term).
/// **Rendering/Input Systems:** Not directly used.
#[derive(Component)]
pub struct ControllerLogic(pub Box<dyn Controller>);

/// # PathPlannerLogic
/// Holds the entity's path planning algorithm implementation.
/// Contains a `Box<dyn Planner>`. Responsible for calculating `CurrentPath`.
///
/// **Data:** `pub Box<dyn Planner>` - The trait object holding the planner logic.
/// **Simulation Systems:**
///    - *Read:* `planner_system` calls `.plan_path()` on this object, providing start (`EstimatedState` or `TrueState`),
///              `GoalComponent`, obstacles, and maybe `DynamicsModel`.
/// **Rendering/Input Systems:** Not directly used.
#[derive(Component)]
pub struct PathPlannerLogic(pub Box<dyn Planner>);

/// # EstimatorLogic
/// Holds the entity's state estimation algorithm implementation and its internal state (e.g., filter state, covariance).
/// Contains a `Box<dyn Estimator>`. Responsible for calculating `EstimatedState`.
///
/// **Data:** `pub Box<dyn Estimator>` - The trait object holding the estimator logic and state.
/// **Simulation Systems:**
///    - *Read:* `estimator_system` calls `.estimate()` (or predict/update) on this object when triggered
///               by its timer or `SensorOutputEvent`. Needs `&mut` access to update internal state.
///               It also calls `.get_current_estimate()` and `.get_covariance()` to update the `EstimatedState` component.
/// **Rendering/Input Systems:** Not directly used.
#[derive(Component)]
pub struct EstimatorLogic(pub Box<dyn Estimator>);

/// # SensorInstance
/// Represents a single physical sensor attached to an entity, including its logic,
/// relative pose, and update rate control. Stored within `SensorSuite`.
///
/// **Data:**
///    - `pose_offset: Isometry3<f64>` - Transform from entity origin to sensor origin.
///    - `model: Box<dyn Sensor>` - The sensor simulation logic.
///    - `update_rate_hz: Option<f32>` - How often the sensor *tries* to update.
///    - `timer: Timer` - Bevy timer used by `sensor_system` to trigger `.sense()`.
/// **Simulation Systems:**
///    - *Read/Write:* `sensor_system` iterates through these in `SensorSuite`, ticks the `timer`, and if finished, calls `.model.sense()`, using the entity's `TrueState`, calculated sensor world pose (`Entity::Transform * pose_offset`), and obstacles. The system then fires a `SensorOutputEvent`.
/// **Rendering/Input Systems:** Not directly used, but data visualized via `SensorOutputEvent`.
pub struct SensorInstance {
    pub pose_offset: Isometry3<f64>,
    pub model: Box<dyn Sensor>,
    pub update_rate_hz: Option<f32>,
    pub(crate) timer: Timer,
}

// Manual Debug implementation for SensorInstance
impl fmt::Debug for SensorInstance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SensorInstance")
            .field("pose_offset", &self.pose_offset)
            .field("model", &self.model) // Relies on Sensor trait having : Debug
            .field("update_rate_hz", &self.update_rate_hz)
            // Timer is intentionally skipped
            .finish()
    }
}

// Constructor for SensorInstance
impl SensorInstance {
    pub fn new(
        pose_offset: Isometry3<f64>,
        model: Box<dyn Sensor>,
        update_rate_hz: Option<f32>,
    ) -> Self {
        let timer_duration = update_rate_hz.map_or(0.0, |hz| 1.0 / hz.max(0.001)); // Avoid division by zero or negative
        let mut timer = Timer::from_seconds(timer_duration, TimerMode::Repeating);
        // Mark timer as finished only if it has a positive duration, ensuring first run.
        if timer_duration > 0.0 {
            timer.tick(std::time::Duration::from_secs_f32(timer_duration));
        }
        Self {
            pose_offset,
            model,
            update_rate_hz,
            timer,
        }
    }
}

/// # SensorSuite
/// Component holding all `SensorInstance`s attached to a single entity.
/// Allows an entity to have multiple sensors of different types.
///
/// **Data:** `pub sensors: Vec<SensorInstance>` - The collection of sensors.
/// **Simulation Systems:**
///    - *Read/Write:* `sensor_system` queries for entities with `SensorSuite` and `TrueState`,
///                     iterates through the `sensors` Vec, and manages their individual timers and calls to `.sense()`.
/// **Rendering/Input Systems:** Not directly used.
#[derive(Component, Default, Debug)]
pub struct SensorSuite {
    pub sensors: Vec<SensorInstance>,
}

/// # SensorOutputEvent (Bevy Event)
/// Fired by the `sensor_system` whenever a `SensorInstance` successfully generates data.
/// This decouples sensor data generation from consumption (e.g., by the estimator).
///
/// **Data:**
///    - `pub entity: Entity` - The entity that owns the sensor generating the data.
///    - `pub data: SensorOutputData` - The actual measurement data (includes sensor_id).
/// **Simulation Systems:**
///    - *Write:* `sensor_system` creates and sends (`EventWriter`) these events.
///    - *Read:* `estimator_system` reads (`EventReader`) these events to trigger its update step using the `data` for the corresponding `entity`.
/// **Rendering Systems:**
///    - *Read:* Sensor visualization systems (`draw_lidar_system`, etc.) can read these events to draw transient sensor data (like lidar points) each frame they are generated. Requires `DrawSensorViz` flags.
/// **Input Systems:** Not directly used.
#[derive(Event, Debug, Clone)] // Use Bevy's #[derive(Event)]
pub struct SensorOutputEvent {
    pub entity: Entity,
    pub data: SensorOutputData,
}

// --- Configuration and Marker Components ---

// =========================================================================
// == Configuration and Marker Components ==
// Define properties of entities or mark them for specific systems.
// =========================================================================

/// # ObstacleComponent
/// Attaches static or dynamic obstacle properties (shape, pose) to an entity.
/// Used for collision detection, sensor simulation (occlusion), and path planning.
///
/// **Data:** `pub Obstacle` - The obstacle data structure from `traits.rs`.
/// **Simulation Systems:**
///    - *Read:* `planner_system` queries for entities with `ObstacleComponent` (and `Transform`) to get the list of obstacles.
///    - *Read:* `sensor_system` queries for these to simulate occlusion/detection (e.g., lidar raycasting).
///    - *Read:* Collision detection systems would use this.
/// **Rendering Systems:**
///    - *Read:* `draw_obstacle_system` reads this (along with `Transform`) to render the obstacle's shape.
/// **Input Systems:** Could potentially allow selecting/modifying obstacles.
#[derive(Component, Debug, Clone)]
pub struct ObstacleComponent(pub Obstacle);

/// # Selectable (Marker Component)
/// Marks an entity as being interactable by the user via UI/mouse clicks.
///
/// **Data:** None (it's a marker).
/// **Simulation Systems:**
///    - *Read:* Highlighting or selection systems might use this.
/// **Rendering Systems:** Could change appearance if selected (e.g., outline).
/// **Input Systems:**
///    - *Read:* `mouse_goal_system` or selection systems query for entities `With<Selectable>` to determine what the user can interact with.
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct Selectable;

/// # AutonomousAgent (Marker Component)
/// Marks an entity that runs its own planning, control, estimation loop autonomously.
/// Distinguishes robots from passive objects or externally controlled ones.
///
/// **Data:** None.
/// **Simulation Systems:**
///    - *Read:* Useful for high-level logic or systems that only operate on autonomous robots
///             (e.g., mission assignment). Helps systems efficiently query only relevant entities.
/// **Rendering/Input Systems:** Not typically used directly.
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct AutonomousAgent;

/// Marker component to mark an entity that can be controlled via keyboard input.
#[derive(Component, Debug, Clone, Copy, Default)] // Added derives
pub struct KeyboardControlled;

/// Marker component for the entity the camera should follow.
#[derive(Component, Debug, Clone, Copy, Default)] // Added derives
pub struct FollowCameraTarget;

/// Component holding settings for a camera that follows a target.
#[derive(Component, Debug, Clone)] // Added derives
pub struct FollowCamera {
    pub distance: f32,
    pub height: f32,
    pub lag_speed: f32,
}

// Add default values if desired
impl Default for FollowCamera {
    fn default() -> Self {
        Self {
            distance: 15.0,
            height: 8.0,
            lag_speed: 3.0,
        }
    }
}

// =========================================================================
// == Simulation Rate Control Components ==
// Optionally provide per-entity control over system update rates.
// Often superseded by global FixedTimestep settings in main.rs for simplicity.
// =========================================================================

#[derive(Component, Debug, Clone, Copy)]
pub struct PlannerRateHz(pub f32); // Use with Timer or common_conditions::on_timer

#[derive(Component, Debug, Clone, Copy)]
pub struct ControllerRateHz(pub f32);

#[derive(Component, Debug, Clone, Copy)]
pub struct EstimatorRateHz(pub f32);

// Note: Dynamics rate is often tied to a fixed simulation step (`FixedTimestep`)
// rather than a per-entity component, but could be added if needed:
// #[derive(Component, Debug, Clone, Copy)]
// pub struct DynamicsRateHz(pub f32); // e.g., 100.0 or higher

// =========================================================================
// == Visualization Components ==
// Control rendering aspects for debugging and visualization.
// =========================================================================

/// # DrawTrueStateViz
/// Toggles rendering of the ground truth state visualization.
#[derive(Component, Debug, Clone, Copy)]
pub struct DrawTrueStateViz(pub bool);

/// # DrawEstimatedStateViz
/// Toggles rendering of the estimated state visualization (e.g., ghost, covariance).
#[derive(Component, Debug, Clone, Copy)]
pub struct DrawEstimatedStateViz(pub bool);

/// # DrawPathViz
/// Toggles rendering of the entity's current planned path.
#[derive(Component, Debug, Clone, Copy)]
pub struct DrawPathViz(pub bool);

/// # DrawSensorViz
/// Toggles rendering of specific sensor data visualizations.
#[derive(Component, Debug, Clone, Copy)]
pub struct DrawSensorViz {
    pub lidar: bool,
    pub gps_uncertainty: bool,
    // Add flags for other sensors as needed
}
impl Default for DrawSensorViz {
    // Provide default visualization settings
    fn default() -> Self {
        Self {
            lidar: true,
            gps_uncertainty: true,
        }
    }
}

/// # DrawGoalViz
/// Toggles rendering of the entity's current goal.
#[derive(Component, Debug, Clone, Copy)]
pub struct DrawGoalViz(pub bool);

// =========================================================================
// == Bevy Built-in Components (Implicitly Used) ==
// Remember that Bevy provides essential components we rely on.
// =========================================================================
// - `Transform`: Defines world position, rotation, scale. Crucial for rendering,
//   spatial queries (sensors, planning), and calculating world poses.
// - `GlobalTransform`: Automatically calculated world transform (handles hierarchy).
// - `Visibility`, `ComputedVisibility`: Control rendering visibility.
// - `Name`: Optional component for identifying entities in logs/editor.
// - `Parent`, `Children`: For scene hierarchy.
