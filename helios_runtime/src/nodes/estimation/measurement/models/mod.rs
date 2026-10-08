//! The built-in measurement kinds, each registered with the payload type it
//! reads.
//!
//! Every model resolves its sensor's geometry from the TF tree at tick time,
//! so each is built for the sensor frame its channel names. Physical constants
//! the model believes (gravity, the local magnetic field) come from its own
//! config.
//!
//! - `config` — the configs of the kinds that take keys.
//! - `model` — each kind's factory and the registration.

mod config;
mod model;

pub(super) use self::model::register;
#[cfg(test)]
pub(super) use self::model::GPS_POSITION_KIND;
