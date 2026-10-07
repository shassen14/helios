//! Teleop family: the operator-intent mapper, its config section, and
//! registration.
//!
//! - `node` — `TwistTeleopNode`, which scales a `TwistIntent` into a
//!   `BodyTwistRef`.
//! - `config` — the `[nodes.<name>]` section of the `TwistTeleop` kind.
//! - `register` — registers that kind.
//!
//! Only the `register` fn crosses the family boundary.

mod config;
mod node;
mod register;

pub(crate) use register::register;
