// helios_core/src/sensors/lidar_2d.rs

use crate::data::sensor;
use crate::sensors::{RayHit, RaycastingOutput, RaycastingSensorModel, SensorRay};
use nalgebra::{Point2, Vector2, Vector3};
use rand::RngCore;
use rand_distr::{Distribution, Normal};

#[derive(Debug, Clone)]
pub struct Lidar2DModel {
    pub(crate) max_range: f32,
    pub(crate) horizontal_fov_deg: f32,
    pub(crate) horizontal_beams: u32,
    pub range_noise_stddev: f32,
    pub angular_noise_stddev_deg: f32,
    range_noise_dist: Normal<f64>,
    angular_noise_dist: Normal<f64>,
}

impl Lidar2DModel {
    /// Returns `None` if either stddev is not strictly positive.
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
            let direction = Vector3::new(angle.cos(), angle.sin(), 0.0);
            rays.push(SensorRay { id: i, direction });
        }
        rays
    }

    fn process_hits(&self, hits: &[RayHit], rng: &mut dyn RngCore) -> RaycastingOutput {
        let fov_rad = self.horizontal_fov_deg.to_radians() as f64;
        let start_angle = -fov_rad / 2.0;
        let angle_increment = fov_rad / (self.horizontal_beams - 1) as f64;

        let points: Vec<Point2<f64>> = hits
            .iter()
            .map(|hit| {
                let noisy_distance = hit.distance as f64 + self.range_noise_dist.sample(rng);
                let perfect_angle = start_angle + (hit.ray_id as f64) * angle_increment;
                let noisy_angle = perfect_angle + self.angular_noise_dist.sample(rng);
                let direction = Vector2::new(noisy_angle.cos(), noisy_angle.sin());
                Point2::from(direction * noisy_distance)
            })
            .collect();

        RaycastingOutput::PointCloud2D(sensor::PointCloud2D { points })
    }

    fn get_max_range(&self) -> f32 {
        self.max_range
    }
}
