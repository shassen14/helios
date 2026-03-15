// heios_core/src/models/perception/lidar_2d.rs

use crate::messages::{MeasurementData, Point, PointCloud};
use crate::models::perception::{RayHit, RaycastingSensorModel, SensorRay};
use crate::types::FrameHandle;
use nalgebra::{Point3, Vector3};
use rand_distr::{Distribution, Normal};

/// A model for a single-plane, 2D LiDAR sensor.
///
/// Noise distributions are pre-computed at construction time and reused on every
/// scan — eliminating `Normal::new()` overhead that would otherwise occur per ray per frame.
#[derive(Debug, Clone)]
pub struct Lidar2DModel {
    pub max_range: f32,
    pub horizontal_fov_deg: f32,
    pub horizontal_beams: u32,
    pub range_noise_stddev: f32,
    pub angular_noise_stddev_deg: f32,
    range_noise_dist: Normal<f64>,
    angular_noise_dist: Normal<f64>,
}

impl Lidar2DModel {
    /// Construct a `Lidar2DModel`, pre-computing noise distributions.
    ///
    /// Returns `None` if either `range_noise_stddev` or `angular_noise_stddev_deg` is not
    /// strictly positive (which would produce an invalid normal distribution).
    pub fn new(
        max_range: f32,
        horizontal_fov_deg: f32,
        horizontal_beams: u32,
        range_noise_stddev: f32,
        angular_noise_stddev_deg: f32,
    ) -> Option<Self> {
        let range_noise_dist = Normal::new(0.0, range_noise_stddev as f64).ok()?;
        let angular_noise_dist =
            Normal::new(0.0, angular_noise_stddev_deg.to_radians() as f64).ok()?;
        Some(Self {
            max_range,
            horizontal_fov_deg,
            horizontal_beams,
            range_noise_stddev,
            angular_noise_stddev_deg,
            range_noise_dist,
            angular_noise_dist,
        })
    }
}

impl RaycastingSensorModel for Lidar2DModel {
    fn generate_rays(&self) -> Vec<SensorRay> {
        let mut rays = Vec::with_capacity(self.horizontal_beams as usize);
        let fov_rad = self.horizontal_fov_deg.to_radians() as f64;
        let start_angle = -fov_rad / 2.0;
        let angle_increment = fov_rad / (self.horizontal_beams - 1) as f64;

        for i in 0..self.horizontal_beams {
            let angle = start_angle + (i as f64) * angle_increment;
            // A 2D LiDAR scans on the XY plane in the standard robotics body frame (+X forward).
            let direction = Vector3::new(angle.cos(), angle.sin(), 0.0);

            rays.push(SensorRay { id: i, direction });
        }
        rays
    }

    fn process_hits(
        &self,
        hits: &[RayHit],
        sensor_handle: FrameHandle,
        timestamp: f64,
    ) -> MeasurementData {
        let mut rng = rand::thread_rng();

        // Pre-compute scan geometry once rather than repeating it inside the closure.
        let fov_rad = self.horizontal_fov_deg.to_radians() as f64;
        let start_angle = -fov_rad / 2.0;
        let angle_increment = fov_rad / (self.horizontal_beams - 1) as f64;

        let points: Vec<Point> = hits
            .iter()
            .map(|hit| {
                // 1. Add noise to the measured distance.
                let noisy_distance = hit.distance as f64 + self.range_noise_dist.sample(&mut rng);

                // 2. Reconstruct the perfect ray angle for this beam id.
                let perfect_angle = start_angle + (hit.ray_id as f64) * angle_increment;

                // 3. Add angular noise.
                let noisy_angle = perfect_angle + self.angular_noise_dist.sample(&mut rng);

                // 4. Create the final point from the noisy angle and distance.
                let direction = Vector3::new(noisy_angle.cos(), noisy_angle.sin(), 0.0);
                Point {
                    position: Point3::from(direction * noisy_distance),
                    intensity: None,
                }
            })
            .collect();

        MeasurementData::PointCloud(PointCloud {
            sensor_handle,
            timestamp,
            points,
        })
    }

    fn get_max_range(&self) -> f32 {
        self.max_range
    }
}
