//! Controller abstraction layer. All types are framework-agnostic.
//!
//! Defines the [`Controller`] trait and its inputs. A controller's output is a
//! concrete typed command from [`commands`] (e.g. [`BodyTwist`](commands::BodyTwist),
//! [`BodyWrench`](commands::BodyWrench)) chosen via the `Out` associated type —
//! there is no output enum. [`ControlDynamics`] is re-exported from
//! `models::controls` for caller convenience. Bevy/ECS wrappers and actuator
//! dispatch live in `helios_sim`.

pub mod actuation_model;
pub mod actuators;
pub mod allocation;
pub mod commands;
pub mod controllers;
pub mod dynamics;
pub mod kernels;
pub mod measurement;
pub mod plant;
pub mod reference;

pub use crate::control::dynamics::ControlDynamics;
pub use controllers::{ControlInputs, Controller};
pub use reference::{BodyTwistRef, ControlReference};
