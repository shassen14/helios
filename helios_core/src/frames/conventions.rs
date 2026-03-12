// helios_core/src/frames/conventions.rs
//
// Pure, stateless helpers for the two coordinate conventions used throughout
// the project:
//
//   ENU  — East=+X, North=+Y, Up=+Z  (world frame & sensor-local axes in TF)
//   FLU  — Forward=+X, Left=+Y, Up=+Z (body/sensor measurement frame)
//
// The FLU→ENU relationship is a static 90° CCW rotation around Z and is
// independent of the sensor's orientation in the world.

use std::f64::consts::FRAC_PI_2;

use nalgebra::{Isometry3, UnitQuaternion, Vector3};

/// Returns the rotation that maps FLU axes to ENU axes: 90° CCW around Z.
///
/// Use this whenever sensor-local FLU measurements need to be expressed in
/// ENU-aligned coordinates (e.g. before applying a world pose isometry).
#[inline]
pub fn flu_to_enu_rotation() -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2)
}

/// Returns the rotation that maps ENU axes to FLU axes: 90° CW around Z.
///
/// This is the inverse of [`flu_to_enu_rotation`].
#[inline]
pub fn enu_to_flu_rotation() -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -FRAC_PI_2)
}

/// Composes the FLU→ENU frame correction into a sensor world-pose isometry.
///
/// `sensor_world_pose` transforms ENU-sensor-local → ENU world (as produced
/// by the TF tree). Points stored in FLU (e.g. lidar hit positions) can then
/// be projected directly into the ENU world frame with the returned isometry.
///
/// ```text
/// p_world = flu_corrected_pose * p_flu
/// ```
#[inline]
pub fn sensor_pose_flu_to_world(sensor_world_pose: Isometry3<f64>) -> Isometry3<f64> {
    Isometry3::from_parts(
        sensor_world_pose.translation,
        sensor_world_pose.rotation * flu_to_enu_rotation(),
    )
}
