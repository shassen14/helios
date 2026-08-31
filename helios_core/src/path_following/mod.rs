//! Path following layer — advances a geometric or temporal cursor along a planned
//! path and emits one [`BodyTwistRef`](crate::control::BodyTwistRef) reference per controller tick.
//!
//! This layer sits between planning and control in the autonomy pipeline:
//!
//! ```text
//! Planner  ──set_path()──►  PathFollower  ──compute()──►  Controller
//! (1–5 Hz)                  (controller rate)
//! ```
//!
//! Each algorithm owns its internal progress state (lookahead index, signed
//! distance cursor, or temporal window). The [`PathFollower::compute`] method
//! advances that state and returns a single reference point.
//!
//! The reference is a body-frame velocity setpoint
//! ([`BodyTwistRef`](crate::control::BodyTwistRef)). A follower that drives only
//! some DOF (a car: surge + yaw-rate) leaves the rest zero. Feedforward terms
//! (curvature, acceleration) are not carried here — they belong to a separate
//! summed node, so this layer stays pure geometry-to-velocity.
//!
//! This layer is bypassed entirely when a trajectory optimizer (Architecture B)
//! or MPC (Architecture C) is active — see `pipeline_vision.md`.

pub mod pure_pursuit;
pub mod steering_pid;

use crate::control::ControlReference;
use crate::frames::conventions::Enu;
use crate::frames::quantities::Point;
use crate::frames::FrameAwareState;
use crate::planning::types::Path;

/// Bus-sourced inputs for one [`PathFollower::compute`] call.
pub struct PathFollowerInputs {
    pub state: FrameAwareState,
}

/// The outcome of one [`PathFollower::compute`] call.
pub enum PathFollowerResult<R: ControlReference> {
    /// Normal operation. The follower advanced its internal cursor and produced
    /// a reference point for the controller this tick.
    Active(R),

    /// The robot is within `goal_radius` of the final waypoint. The variant
    /// carries the terminal reference the controller should now track — a
    /// "park here" setpoint the follower builds from the goal and its own
    /// reference semantics (a zero body twist for a velocity follower). Unlike
    /// [`NoPath`](Self::NoPath) / [`Error`](Self::Error), which hold
    /// last-known-good, this reference is published, so the vehicle stops at the
    /// goal instead of coasting on its last command. Call
    /// [`PathFollower::set_path`] to begin a new path.
    GoalReached(R),

    /// No path has been set yet. Expected at startup and after goal reached
    /// while waiting for the next plan. The controller should hold its last
    /// output or stop.
    NoPath,

    /// A recoverable failure this tick — for example, the state vector is
    /// missing required fields (Px, Py). Logged by `PathFollowingCore`;
    /// the controller holds its last output.
    Error(String),
}

/// A stateful path follower that advances a cursor along a [`Path`] and
/// emits one reference point per controller tick.
///
/// Implementations include Pure Pursuit (geometric lookahead), Stanley
/// (front-axle tracking), and SteeringPid (bearing + PID on heading error).
/// Each owns whatever internal progress state its algorithm requires.
///
/// The trait is intentionally minimal — all tuning parameters (lookahead
/// distance, speed bounds, goal radius) live in the concrete struct and
/// are populated from config at construction time.
pub trait PathFollower: Send + Sync {
    type Reference: ControlReference;

    /// Advance the internal cursor and return a reference for this tick.
    ///
    /// Called at controller rate. `inputs` carries bus-sourced data; `dt` is
    /// the tick timestep from the pipeline clock. The follower decides
    /// internally whether to step the cursor forward based on proximity,
    /// signed distance, or elapsed time — the caller does not control advancement.
    fn compute(
        &mut self,
        dt: f64,
        inputs: &PathFollowerInputs,
    ) -> PathFollowerResult<Self::Reference>;

    /// Replace the active path and reset all internal progress state.
    ///
    /// Called at planner rate when the planner emits a new [`PlannerResult::Path`].
    /// After this call the next [`compute`] begins tracking from the start of
    /// the new path.
    ///
    /// [`compute`]: PathFollower::compute
    fn set_path(&mut self, path: Path);

    /// Return the path waypoint currently targeted by the internal lookahead cursor.
    ///
    /// This is the raw path waypoint at `lookahead_index`, not the computed reference
    /// output from [`compute`]. Its state holds a world-frame position
    /// (`Px(World)`, `Py(World)`, `Pz(World)`) that can be used for visualization
    /// and debugging. Returns `None` when no path has been set.
    ///
    /// [`compute`]: PathFollower::compute
    fn get_lookahead_waypoint(&self) -> Option<&Point<Enu>>;

    /// Reset all internal state, including the stored path and progress cursor.
    ///
    /// Analogous to [`Controller::reset`]. Call when the agent is re-initialized
    /// or when a hard stop is required.
    fn reset(&mut self);
}
