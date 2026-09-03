//! Controller measurement models: the estimate re-expressed as the quantity a
//! control law consumes. The dual of `sensors/` — belief in, controlled quantity
//! out — and, like every crossing between the estimator's frame and the body
//! frame, a frame projection rather than a layout read.

mod body_velocity;

pub use body_velocity::{body_forward_speed, body_velocity};
