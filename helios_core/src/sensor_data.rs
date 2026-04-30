// helios_core/src/sensor_data.rs

use nalgebra::{Point2, Point3, Vector3};
use serde::Serialize;

// m/s², Sensor Frame
#[derive(Clone, Debug, Default, Serialize)]
pub struct LinearAcceleration3D {
    pub value: Vector3<f64>,
}

// rad/s, Sensor Frame
#[derive(Clone, Debug, Default, Serialize)]
pub struct AngularVelocity3D {
    pub value: Vector3<f64>,
}

// microT, Sensor Frame
#[derive(Clone, Debug, Default, Serialize)]
pub struct MagneticField3D {
    pub value: Vector3<f64>,
}

// meters, ENU
#[derive(Clone, Debug, Default, Serialize)]
pub struct GpsPosition {
    pub position: Vector3<f64>,
}

// m/s, ENU
#[derive(Clone, Debug, Serialize)]
pub struct GpsVelocity {
    pub velocity: Vector3<f64>,
}

// m, sensor frame
#[derive(Clone, Debug, Serialize)]
pub struct PointCloud2D {
    pub points: Vec<Point2<f64>>,
}

// m, sensor frame
#[derive(Clone, Debug, Serialize)]
pub struct LidarPoint3D {
    position: Point3<f64>,
    intensity: Option<f32>,
}

// m, sensor frame
#[derive(Clone, Debug, Serialize)]
pub struct PointCloud3D {
    pub points: Vec<LidarPoint3D>,
}
