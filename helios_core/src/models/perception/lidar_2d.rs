// heios_core/src/models/perception/lidar_2d.rs

use crate::messages::{MeasurementData, Point, PointCloud};
use crate::models::perception::{RayHit, RaycastingSensorModel, SensorRay};
use crate::types::FrameHandle;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use rand_distr::{Distribution, Normal};

/// A model for a single-plane, 2D LiDAR sensor.
#[derive(Debug, Clone)]
pub struct Lidar2DModel {
    pub max_range: f32,
    pub horizontal_fov_deg: f32,
    pub horizontal_beams: u32,
    pub range_noise_stddev: f32,
    pub angular_noise_stddev_deg: f32, // New parameter for angular noise
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
        // Use a simple RNG for noise. A real implementation would take a `&mut dyn RngCore`.
        let mut rng = rand::thread_rng();
        let range_noise_dist = Normal::new(0.0, self.range_noise_stddev as f64).unwrap();

        let points: Vec<Point> = hits
            .iter()
            .map(|hit| {
                // 1. Add noise to the measured distance.
                let noisy_distance = hit.distance as f64 + range_noise_dist.sample(&mut rng);

                // 2. Find the original "perfect" direction vector for this ray.
                // (A more optimized version would pass the original rays into this function).
                let fov_rad = self.horizontal_fov_deg.to_radians() as f64;
                let start_angle = -fov_rad / 2.0;
                let angle_increment = fov_rad / (self.horizontal_beams - 1) as f64;
                let perfect_angle = start_angle + (hit.ray_id as f64) * angle_increment;

                // 3. Add angular noise.
                let angular_noise_rad = self.angular_noise_stddev_deg.to_radians() as f64;
                let angular_noise_dist = Normal::new(0.0, angular_noise_rad).unwrap();
                let noisy_angle = perfect_angle + angular_noise_dist.sample(&mut rng);

                // 4. Create the final point from the noisy angle and distance.
                let direction = Vector3::new(noisy_angle.cos(), noisy_angle.sin(), 0.0);
                // Create the new Point struct
                Point {
                    position: Point3::from(direction * noisy_distance),
                    intensity: None, // We don't simulate intensity yet
                }
            })
            .collect();

        // Construct the final PointCloud struct
        let point_cloud = PointCloud {
            sensor_handle,
            timestamp,
            points,
        };

        // Wrap it in the MeasurementData enum variant
        MeasurementData::PointCloud(point_cloud)
    }

    fn get_max_range(&self) -> f32 {
        self.max_range
    }
}
