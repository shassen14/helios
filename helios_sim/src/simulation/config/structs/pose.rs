use bevy::prelude::Transform;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serde::Deserialize;

use crate::simulation::utils::serde_helpers;

#[derive(Deserialize, Debug, Clone, Copy, Default)]
pub struct Pose {
    #[serde(with = "serde_helpers::vec3_f64_from_f32_array", default)]
    pub translation: Vector3<f64>,

    #[serde(with = "serde_helpers::quat_f64_from_euler_deg_f32", default)]
    pub rotation: UnitQuaternion<f64>,
}

impl Pose {
    pub fn to_isometry(&self) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::from(self.translation), self.rotation)
    }

    pub fn to_bevy_transform(&self) -> Transform {
        crate::simulation::core::transforms::enu_body_iso_to_bevy_transform(&self.to_isometry())
    }

    /// For body-relative sensor placement (FLU frame).
    pub fn to_bevy_local_transform(&self) -> Transform {
        crate::simulation::core::transforms::flu_iso_to_bevy_local_transform(&self.to_isometry())
    }
}
