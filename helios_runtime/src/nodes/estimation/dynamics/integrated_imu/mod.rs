//! The `IntegratedImu` dynamics kind: a strapdown INS driven by the IMU's
//! specific force and angular rate, read from the bus each predict.
//!
//! - `config` — the kind's `[dynamics]` sub-table and its prior defaults.
//! - `model` — the input builder and the factory that builds the model.

mod config;
mod model;

pub(super) use self::model::register;
#[cfg(test)]
pub(super) use self::model::INTEGRATED_IMU_KIND;
