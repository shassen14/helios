// helios_sim/src/simulation/core/transforms/constants.rs
//
// Static rotation constants for ENU↔Bevy and FLU↔Bevy frame conversions.
// Kept in a separate file to make the mathematical basis explicit.

use nalgebra::{UnitQuaternion, Vector3};
use std::f64::consts::FRAC_PI_2;

thread_local! {
    /// Quaternion from ENU frame to Bevy world frame.
    ///
    /// ENU X (East) → Bevy +X, ENU Y (North) → Bevy −Z, ENU Z (Up) → Bevy +Y.
    /// Equivalent to a −90° rotation around the X-axis.
    pub(super) static Q_ENU_FRAME_TO_BEVY_FRAME: UnitQuaternion<f64> =
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2);
}

thread_local! {
    /// Quaternion from FLU body frame to Bevy local frame.
    ///
    /// FLU: Forward=+X, Left=+Y, Up=+Z → Bevy local: Forward=−Z, Left=−X, Up=+Y.
    /// Composed as: Q_ENU_TO_BEVY * Q_FLU_TO_ENU (rotate FLU→ENU, then ENU→Bevy).
    pub(super) static Q_FLU_BODY_TO_BEVY_LOCAL: UnitQuaternion<f64> = {
        let q_flu_to_enu = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        let q_enu_to_bevy = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2);
        q_enu_to_bevy * q_flu_to_enu
    };
}
