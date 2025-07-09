// src/simulation/utils/serde_helpers.rs

pub mod vec3_f64_from_f32_array {
    use nalgebra::Vector3;
    use serde::{self, Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(vec: &Vector3<f64>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let arr = [vec.x as f32, vec.y as f32, vec.z as f32];
        // Use `serialize_fixed_size_array` for arrays
        serializer.collect_seq(arr.iter())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vector3<f64>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let arr: [f32; 3] = Deserialize::deserialize(deserializer)?;
        Ok(Vector3::new(arr[0] as f64, arr[1] as f64, arr[2] as f64))
    }
}

pub mod quat_f64_from_euler_deg_f32 {
    use nalgebra::UnitQuaternion;
    use serde::{self, Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(quat: &UnitQuaternion<f64>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // Get roll, pitch, yaw in radians
        let euler = quat.euler_angles();
        // Convert to degrees for serialization
        let arr = [
            euler.0.to_degrees() as f32, // Roll
            euler.1.to_degrees() as f32, // Pitch
            euler.2.to_degrees() as f32, // Yaw
        ];
        serializer.collect_seq(arr.iter())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<UnitQuaternion<f64>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let arr: [f32; 3] = Deserialize::deserialize(deserializer)?;
        Ok(UnitQuaternion::from_euler_angles(
            (arr[0]).to_radians() as f64, // Roll
            (arr[1]).to_radians() as f64, // Pitch
            (arr[2]).to_radians() as f64, // Yaw
        ))
    }
}
