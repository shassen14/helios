//! The command type an allocator consumes.
//!
//! It is not parsed as a value: an allocator kind implies its own. The
//! assembler uses the tag to pick the concrete `T` of the allocator's input
//! channel, `T @ <input>`. That is why it deliberately does **not** derive
//! `Deserialize`. A `[command]` fold names its type through the registry's
//! command-type table instead, which is open to new types; this tag is closed,
//! and serves only the allocators until each allocator factory knows its input
//! type itself.
//!
//! Each variant names, one-to-one, a command type in
//! `helios_core::control::commands`; the variant name *is* the mapping.

/// A command type an allocator can consume.
///
/// Each variant corresponds to a `helios_core::control::commands` type of the
/// same name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum CommandSpace {
    /// `commands::BodyTwist` — a body-frame velocity (linear + angular). The
    /// command space of the kinematic Ackermann allocator.
    BodyTwist,
    /// `commands::DriveForce` — a scalar longitudinal drive force. The command
    /// space of the wheel-torque allocator.
    DriveForce,

    SteerAngle,
}
