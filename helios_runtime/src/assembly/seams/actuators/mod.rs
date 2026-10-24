//! The actuator seam: turns the `[actuators]` section into the `Merge` that
//! writes the one actuator command the body applies.
//!
//! Each member writes a partial `ActuatorCommand` on a channel of its own,
//! over the actuators its output port declares it drives. The seam finds each
//! member's command output among the built nodes, checks the declared drives
//! against each other and against the body, and adds one `Merge` from those
//! channels onto the fixed [`control::actuators`] channel. A lone member flows
//! through a one-input `Merge`, so the shape is the same for any number.
//!
//! The output keeps one fixed name, unlike a command fold: a body takes one
//! actuator command, so the name is the contract at the body boundary.
//!
//! - `config` — the section's config.
//! - `pass` — the pass that builds the seam's node.
//!
//! [`control::actuators`]: crate::channels::control::actuators

mod config;
mod pass;

pub use self::config::ActuatorSeamConfig;
pub(in crate::assembly) use self::pass::actuator_merge;
