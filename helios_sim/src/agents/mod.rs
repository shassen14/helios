//! The embodied half of each simulated agent: its physical body and the
//! sensors mounted on it.
//!
//! An "agent" here is a vehicle plus a sensor suite. This module owns the host
//! side of that pairing — [`vehicles`] turns the brain's control output into
//! physical motion, [`sensors`] samples ground-truth world state into typed
//! readings. The autonomy that decides what the body does lives behind the
//! `brain_bridge`; nothing in this module runs a pipeline. It only moves data
//! across the host↔world boundary, calling into `helios_core` for any math.

pub mod sensors;
pub mod vehicles;
