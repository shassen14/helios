//! The command-seam contract type: which command a controller emits and an
//! allocator consumes.
//!
//! This is a *derived dispatch tag*, not a parsed config field — nothing in TOML
//! ever names a command space. It is computed from the allocator kind (which
//! defines the seam) and from each controller kind, so the assembler can pick the
//! one concrete `T` to instantiate the command channel, the fold, and the
//! allocator input as `command::<T>()`. That is why it deliberately does **not**
//! derive `Deserialize`.
//!
//! It is the seam *between* [`AllocatorConfig`](super::AllocatorConfig) and
//! [`ControllerConfig`](super::ControllerConfig) — owned by neither — so it lives
//! in its own file rather than inside one of them.
//!
//! Each variant names, one-to-one, a command type in
//! `helios_core::control::commands`; the variant name *is* the mapping. The
//! reverse binding (tag → concrete type) is realized in exactly one place, the
//! assembler's command-space dispatch, whose exhaustive match is what guarantees
//! every variant denotes a real core type.

/// A command type that can flow through the `command` channel.
///
/// Each variant corresponds to a `helios_core::control::commands` type of the
/// same name. Compared for equality by validation to enforce that every
/// controller feeding an allocator speaks the allocator's command space.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandSpace {
    /// `commands::BodyTwist` — a body-frame velocity (linear + angular). The
    /// command space of the kinematic Ackermann allocator.
    BodyTwist,
    /// `commands::DriveForce` — a scalar longitudinal drive force. The command
    /// space of the wheel-torque allocator.
    DriveForce,

    SteerAngle,
}
