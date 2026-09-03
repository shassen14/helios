//! Allocators for driven-wheel vehicles.
//!
//! Everything here assumes actuation through a wheel: [`drive_torque`] converts a
//! longitudinal drive force into a wheel torque via the wheel radius, and
//! [`steer_position`] passes a steer angle through to the steering actuator. A
//! car, ATV, motorcycle, or holonomic wheeled platform composes its actuation
//! from members of this module; non-wheeled morphologies (multirotor, thruster)
//! live in sibling modules.

pub mod drive_torque;
pub mod steer_position;
