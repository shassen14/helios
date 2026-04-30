// helios_core/src/sensor_data.rs

use nalgebra::{Point2, Point3, Vector3};
use serde::{Deserialize, Serialize};

/// 3-axis linear acceleration (m/s²) in the sensor's FLU frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LinearAcceleration3D {
    pub value: Vector3<f64>,
}

/// 3-axis angular velocity (rad/s) in the sensor's FLU frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AngularVelocity3D {
    pub value: Vector3<f64>,
}

/// 3-axis magnetic field strength (µT) in the sensor's FLU frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct MagneticField3D {
    pub value: Vector3<f64>,
}

/// 3D position fix (meters) in the ENU world frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct GpsPosition {
    pub position: Vector3<f64>,
}

/// 3D velocity (m/s) in the ENU world frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct GpsVelocity {
    pub velocity: Vector3<f64>,
}

/// 2D point cloud (meters) in the sensor's FLU frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PointCloud2D {
    pub points: Vec<Point2<f64>>,
}

/// A single 3D LiDAR return with optional intensity (meters, sensor FLU frame).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LidarPoint3D {
    pub position: Point3<f64>,
    pub intensity: Option<f32>,
}

/// 3D point cloud (meters) in the sensor's FLU frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PointCloud3D {
    pub points: Vec<LidarPoint3D>,
}

/// Placeholder for RGB camera output. Buffer representation is TBD.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RgbImage;

/// Placeholder for depth/RGBD camera output. Buffer representation is TBD.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DepthImage;

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn linear_acceleration_default_is_zero() {
        let a = LinearAcceleration3D::default();
        assert_eq!(a.value, Vector3::zeros());
    }

    #[test]
    fn gps_velocity_default_is_zero() {
        let v = GpsVelocity::default();
        assert_eq!(v.velocity, Vector3::zeros());
    }

    #[test]
    fn point_cloud_2d_default_is_empty() {
        assert!(PointCloud2D::default().points.is_empty());
    }

    #[test]
    fn lidar_point3d_fields_are_accessible() {
        let p = LidarPoint3D {
            position: Point3::new(1.0, 2.0, 3.0),
            intensity: Some(0.5),
        };
        assert!((p.position.x - 1.0).abs() < 1e-12);
        assert!((p.position.y - 2.0).abs() < 1e-12);
        assert!((p.position.z - 3.0).abs() < 1e-12);
        assert!((p.intensity.unwrap() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn point_cloud_3d_stores_points() {
        let mut cloud = PointCloud3D::default();
        cloud.points.push(LidarPoint3D::default());
        assert_eq!(cloud.points.len(), 1);
    }
}
