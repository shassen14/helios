//! Path following layer — advances a geometric or temporal cursor along a planned
//! path and emits one [`TrajectoryPoint`] reference per controller tick.
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
//! `state_dot` in the returned [`TrajectoryPoint`] is the feedforward carrier:
//! a downstream FF+FB controller extracts curvature or acceleration from it
//! without needing access to the path itself. Simple followers leave it `None`;
//! controllers degrade to pure feedback gracefully.
//!
//! This layer is bypassed entirely when a trajectory optimizer (Architecture B)
//! or MPC (Architecture C) is active — see `pipeline_vision.md`.

pub mod pure_pursuit;
pub mod steering_pid;

use crate::planning::types::Path;
use crate::{frames::FrameAwareState, types::TrajectoryPoint};

/// The outcome of one [`PathFollower::compute`] call.
pub enum PathFollowerResult {
    /// Normal operation. The follower advanced its internal cursor and produced
    /// a reference point for the controller this tick.
    Active(TrajectoryPoint),

    /// The robot is within `goal_radius` of the final waypoint. The controller
    /// should stop issuing commands. Call [`PathFollower::set_path`] to begin
    /// a new path.
    GoalReached,

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
    /// Advance the internal cursor and return a reference for this tick.
    ///
    /// Called at controller rate. The follower decides internally whether to
    /// step the cursor forward based on proximity, signed distance, or elapsed
    /// time — the caller does not control advancement.
    fn compute(&mut self, state: &FrameAwareState, dt: f64) -> PathFollowerResult;

    /// Replace the active path and reset all internal progress state.
    ///
    /// Called at planner rate when the planner emits a new [`PlannerResult::Path`].
    /// After this call the next [`compute`] begins tracking from the start of
    /// the new path.
    ///
    /// [`compute`]: PathFollower::compute
    fn set_path(&mut self, path: Path);

    /// Reset all internal state, including the stored path and progress cursor.
    ///
    /// Analogous to [`Controller::reset`]. Call when the agent is re-initialized
    /// or when a hard stop is required.
    fn reset(&mut self);
}
