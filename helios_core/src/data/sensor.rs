// Raw sensor payload types and the generic `SensorReading<T>` wrapper.
// Sensor payloads (structs like `LinearAcceleration3D`, `GpsPosition`, etc.)
// describe what a physical sensor measures.

use nalgebra::{DVector, Point2, Point3, Vector3};
use serde::{Deserialize, Serialize};

/// Conversion from a typed sensor payload to the flat measurement vector `z` consumed
/// by Gaussian-family filters (EKF, UKF, ESKF, IF).
///
/// One impl per primitive payload that maps cleanly onto `DVector<f64>`. Payloads that
/// don't fit this shape (point clouds, images) are not `SensorPayload` — they flow
/// through perception/mapping paths, not the Kalman update path.
pub trait SensorPayload: 'static + Send + Sync {
    fn to_measurement_vector(&self) -> DVector<f64>;
}

impl SensorPayload for GpsPosition {
    fn to_measurement_vector(&self) -> DVector<f64> {
        DVector::from_row_slice(self.position.as_slice())
    }
}

impl SensorPayload for GpsVelocity {
    fn to_measurement_vector(&self) -> DVector<f64> {
        DVector::from_row_slice(self.velocity.as_slice())
    }
}

impl SensorPayload for LinearAcceleration3D {
    fn to_measurement_vector(&self) -> DVector<f64> {
        DVector::from_row_slice(self.value.as_slice())
    }
}

impl SensorPayload for AngularVelocity3D {
    fn to_measurement_vector(&self) -> DVector<f64> {
        DVector::from_row_slice(self.value.as_slice())
    }
}

impl SensorPayload for MagneticField3D {
    fn to_measurement_vector(&self) -> DVector<f64> {
        DVector::from_row_slice(self.value.as_slice())
    }
}

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
    pub(crate) velocity: Vector3<f64>,
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
    pub(crate) intensity: Option<f32>,
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

    #[test]
    fn sensor_payload_gps_position_yields_xyz() {
        let p = GpsPosition {
            position: Vector3::new(1.0, 2.0, 3.0),
        };
        let z = p.to_measurement_vector();
        assert_eq!(z.nrows(), 3);
        assert!((z[0] - 1.0).abs() < 1e-12);
        assert!((z[1] - 2.0).abs() < 1e-12);
        assert!((z[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn sensor_payload_linear_acceleration_yields_xyz() {
        let a = LinearAcceleration3D {
            value: Vector3::new(0.1, 0.2, 9.8),
        };
        let z = a.to_measurement_vector();
        assert_eq!(z.nrows(), 3);
        assert!((z[2] - 9.8).abs() < 1e-12);
    }
}
