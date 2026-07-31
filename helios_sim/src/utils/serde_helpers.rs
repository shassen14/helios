//! Custom (de)serialization for config types whose TOML shape differs from
//! their runtime shape. TOML numbers are f64-native, so these helpers stay in
//! f64 end to end — narrowing through f32 would discard file precision for
//! nothing.

pub mod vec3_from_array {
    use nalgebra::Vector3;
    use serde::{self, Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(vec: &Vector3<f64>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.collect_seq([vec.x, vec.y, vec.z].iter())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vector3<f64>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let arr: [f64; 3] = Deserialize::deserialize(deserializer)?;
        Ok(Vector3::from(arr))
    }
}

/// TOML expresses rotations as `[roll, pitch, yaw]` in degrees for human
/// readability; runtime holds a quaternion in radians.
pub mod quat_from_euler_deg {
    use nalgebra::UnitQuaternion;
    use serde::{self, Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(quat: &UnitQuaternion<f64>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let (roll, pitch, yaw) = quat.euler_angles();
        serializer.collect_seq([roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()].iter())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<UnitQuaternion<f64>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let [roll, pitch, yaw]: [f64; 3] = Deserialize::deserialize(deserializer)?;
        Ok(UnitQuaternion::from_euler_angles(
            roll.to_radians(),
            pitch.to_radians(),
            yaw.to_radians(),
        ))
    }
}
