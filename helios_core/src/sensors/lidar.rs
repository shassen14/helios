use crate::data::{PointCloud, PointCloudBuilder};
use crate::frames::conventions::Flu;
use crate::frames::quantities::Point;
use crate::sensors::{RayHit, RaycastingOutput, RaycastingSensorModel, SensorRay};
use nalgebra::Vector3;
use rand::RngCore;
use rand_distr::{Distribution, Normal};

/// Forward (truth) model for a ray lidar of any beam layout — 2D planar,
/// spinning multi-ring, forward-looking solid-state, or flash. The layout is not
/// a type; it is entirely the scan geometry below. A 2D lidar is a single ring at
/// `0.0` elevation, a spinning unit is a `2π` azimuth sweep over a list of ring
/// elevations, and a flash unit is a `sweep_period` of zero.
///
/// The model is deliberately richer than any consumer's belief — noise lives in
/// spherical space and points carry per-point time — so simulating against it is
/// not an inverse crime.
#[derive(Debug, Clone)]
pub struct LidarModel {
    // Scan geometry — the beam pattern, and the whole of what distinguishes one
    // lidar from another. Azimuth is a uniform sweep (real units hold a constant
    // angular rate); ring elevations are an explicit list (datasheet tables are
    // non-uniform). `azimuth_start` / `azimuth_increment` are derived from the
    // sweep once in `new`.
    azimuth_start: f64,
    azimuth_increment: f64,
    n_azimuth: u32,
    ring_elevations: Vec<f64>,

    /// Seconds for one full azimuth sweep. `0.0` models a flash capture, where
    /// every point shares the reading's instant (all per-point offsets zero).
    sweep_period: f64,

    /// Maximum reported range, in meters.
    max_range: f32,

    /// Range noise standard deviation, in meters.
    pub range_noise_stddev: f32,
    /// Angular noise standard deviation, in degrees, applied to both azimuth and
    /// elevation.
    pub angular_noise_stddev: f32,

    range_noise_dist: Normal<f64>,
    angular_noise_dist: Normal<f64>,
}

impl LidarModel {
    /// Builds the model, or returns `None` if the configuration is unusable:
    /// either noise standard deviation is not strictly positive, there are no
    /// azimuth beams, or the ring-elevation list is empty.
    pub fn new(
        azimuth_fov: f64,
        n_azimuth: u32,
        ring_elevations: Vec<f64>,
        sweep_period: f64,
        max_range: f32,
        range_noise_stddev: f32,
        angular_noise_stddev: f32,
    ) -> Option<Self> {
        if range_noise_stddev <= 0.0 || angular_noise_stddev <= 0.0 {
            return None;
        }

        if n_azimuth < 1 || ring_elevations.is_empty() {
            return None;
        }

        let range_noise_dist = Normal::new(0.0, range_noise_stddev as f64).ok()?;
        let angular_noise_dist = Normal::new(0.0, angular_noise_stddev.to_radians() as f64).ok()?;

        let full_circle = azimuth_fov >= std::f64::consts::TAU - 1e-9;
        let azimuth_increment = if n_azimuth <= 1 {
            0.0
        } else if full_circle {
            azimuth_fov / n_azimuth as f64
        } else {
            azimuth_fov / (n_azimuth - 1) as f64
        };
        let azimuth_start = -azimuth_fov / 2.0;

        Some(Self {
            azimuth_start,
            azimuth_increment,
            n_azimuth,
            ring_elevations,
            sweep_period,
            max_range,
            range_noise_stddev,
            angular_noise_stddev,
            range_noise_dist,
            angular_noise_dist,
        })
    }

    /// Maps a beam's `(azimuth, elevation)` to a unit direction in the sensor's
    /// FLU frame: `x` forward, `y` left, `z` up. Azimuth turns about `+z`, so a
    /// positive azimuth points left; elevation tilts up from the `xy` plane. Both
    /// `generate_rays` and `process_hits` project through here, so the convention
    /// lives in exactly one place.
    fn direction(azimuth: f64, elevation: f64) -> Vector3<f64> {
        Vector3::new(
            elevation.cos() * azimuth.cos(),
            elevation.cos() * azimuth.sin(),
            elevation.sin(),
        )
    }

    /// Recovers the `(ring, azimuth)` indices from a beam's flat id, inverting the
    /// `ring * n_azimuth + azimuth` encoding `generate_rays` assigns.
    fn split_id(&self, id: u32) -> (u32, u32) {
        (id / self.n_azimuth, id % self.n_azimuth)
    }

    /// The perfect (noise-free) azimuth of a beam index within the sweep.
    fn azimuth_of(&self, az_idx: u32) -> f64 {
        self.azimuth_start + az_idx as f64 * self.azimuth_increment
    }

    /// The per-point time offset for a beam index, in nanoseconds from the start
    /// of the sweep — a spinning lidar samples later azimuths later. Zero for
    /// every beam when `sweep_period` is zero (a flash capture).
    fn dt_of(&self, az_idx: u32) -> i32 {
        let fraction = az_idx as f64 / self.n_azimuth as f64;
        (fraction * self.sweep_period * 1e9) as i32
    }
}

impl RaycastingSensorModel for LidarModel {
    fn generate_rays(&self) -> Vec<SensorRay> {
        let mut rays = Vec::with_capacity(self.ring_elevations.len() * self.n_azimuth as usize);

        for (ring_idx, &elev) in self.ring_elevations.iter().enumerate() {
            for az_idx in 0..self.n_azimuth {
                let id = ring_idx as u32 * self.n_azimuth + az_idx;
                let direction = Self::direction(self.azimuth_of(az_idx), elev);
                rays.push(SensorRay { id, direction });
            }
        }

        rays
    }

    /// Projects each ray return into a sensor-frame point, applying the error
    /// model in spherical space.
    ///
    /// A lidar measures a range along a beam and that beam's two angles, so its
    /// error lives on `(range, azimuth, elevation)`, not on cartesian `xyz`. Each
    /// is perturbed independently before the spherical-to-cartesian projection;
    /// perturbing `xyz` directly would fabricate a range-independent, isotropic
    /// error the real device never exhibits. This is the one property to preserve
    /// on every future change to the model.
    ///
    /// The cloud is built timed: each point carries an offset from its azimuth's
    /// position in the sweep, so a later de-skew step can undo motion during the
    /// scan. No consumer reads the offsets yet — stamping them now keeps the
    /// producer ready rather than retrofitting timing later.
    ///
    /// Misses never reach here — the host omits them from `hits` — so the cloud
    /// is exactly the set of returns. `finalize` can only fail when a column falls
    /// out of step with the points, which the fused `push_timed` makes unreachable
    /// here, so the error arm yields an empty cloud rather than surfacing a
    /// `Result` the caller cannot act on.
    fn process_hits(&self, hits: &[RayHit], rng: &mut dyn RngCore) -> RaycastingOutput {
        let mut points = PointCloudBuilder::timed();
        for hit in hits {
            let (ring_idx, az_idx) = self.split_id(hit.ray_id);
            let perfect_az = self.azimuth_of(az_idx);
            let perfect_el = self.ring_elevations[ring_idx as usize];
            let noisy_az = perfect_az + self.angular_noise_dist.sample(rng);
            let noisy_el = perfect_el + self.angular_noise_dist.sample(rng);
            let noisy_range = hit.distance as f64 + self.range_noise_dist.sample(rng);

            let point = Self::direction(noisy_az, noisy_el) * noisy_range;
            points.push_timed(Point::from_raw(point), (), self.dt_of(az_idx))
        }

        let Ok(cloud) = points.finalize() else {
            return RaycastingOutput::PointCloud(PointCloud::<Flu>::empty(()));
        };

        RaycastingOutput::PointCloud(cloud)
    }

    fn get_max_range(&self) -> f32 {
        self.max_range
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::{FRAC_PI_2, TAU};

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    /// A planar (2D-equivalent) model: one ring at 0° elevation, a 90° forward
    /// sweep of 3 beams. Both endpoints are included, so the increment is 45° and
    /// beams 0/1/2 point at -45°/0°/+45° — the middle beam (id 1) is forward.
    /// Angles are radians, angular noise degrees, range noise meters; the noise
    /// is small enough that the geometry assertions hold under any seed.
    fn planar_model() -> LidarModel {
        LidarModel::new(FRAC_PI_2, 3, vec![0.0], 0.1, 30.0, 0.01, 0.1)
            .expect("configuration is valid")
    }

    /// Two rings tilted ±0.3 rad over the same 3-beam forward sweep. Ring 0 is up,
    /// ring 1 is down; the forward beam of each carries flat id 1 and 4.
    fn two_ring_model() -> LidarModel {
        LidarModel::new(FRAC_PI_2, 3, vec![0.3, -0.3], 0.1, 30.0, 0.01, 0.1)
            .expect("configuration is valid")
    }

    fn cloud_of(output: RaycastingOutput) -> PointCloud<Flu, ()> {
        let RaycastingOutput::PointCloud(cloud) = output;
        cloud
    }

    #[test]
    fn projects_the_forward_beam_along_x() {
        // Center beam (id 1): 0° azimuth, 0° elevation. A 5 m return lands near
        // (5, 0, 0) — range rides x while the near-zero angles keep y and z small.
        // Pins the range→x assignment: scaling x by the angle instead of the
        // range drives x toward 0 and trips here.
        let mut rng = StdRng::seed_from_u64(42);
        let hits = [RayHit {
            ray_id: 1,
            distance: 5.0,
        }];

        let cloud = cloud_of(planar_model().process_hits(&hits, &mut rng));
        assert_eq!(cloud.len(), 1);

        let p = cloud.point(0).into_inner();
        assert!((p.x - 5.0).abs() < 0.2, "x should track range, got {}", p.x);
        assert!(p.y.abs() < 0.2, "forward-beam y should be ~0, got {}", p.y);
        assert!(
            p.z.abs() < 0.2,
            "0° ring keeps z ~0 up to elevation noise, got {}",
            p.z
        );
    }

    #[test]
    fn azimuth_sign_follows_the_beam() {
        // id 0 is the -45° beam, id 2 the +45° beam. Positive azimuth points left
        // (+y), so the near beam projects to -y and the far to +y; the tiny
        // angular noise cannot flip a 45° bearing.
        let mut rng = StdRng::seed_from_u64(7);
        let hits = [
            RayHit {
                ray_id: 0,
                distance: 4.0,
            },
            RayHit {
                ray_id: 2,
                distance: 4.0,
            },
        ];

        let cloud = cloud_of(planar_model().process_hits(&hits, &mut rng));
        assert_eq!(cloud.len(), 2);
        assert!(cloud.point(0).into_inner().y < 0.0, "-45° beam → -y");
        assert!(cloud.point(1).into_inner().y > 0.0, "+45° beam → +y");
    }

    #[test]
    fn elevation_and_ring_index_lift_z() {
        // One forward beam per ring: id 1 is ring 0 (+0.3 rad, up), id 4 is ring 1
        // (-0.3 rad, down). Pins elevation→z and the flat-id ring recovery at
        // once — a wrong split reads the other ring's elevation and flips z.
        let mut rng = StdRng::seed_from_u64(11);
        let hits = [
            RayHit {
                ray_id: 1,
                distance: 6.0,
            },
            RayHit {
                ray_id: 4,
                distance: 6.0,
            },
        ];

        let cloud = cloud_of(two_ring_model().process_hits(&hits, &mut rng));
        assert_eq!(cloud.len(), 2);
        assert!(cloud.point(0).into_inner().z > 0.0, "up ring → +z");
        assert!(cloud.point(1).into_inner().z < 0.0, "down ring → -z");
    }

    #[test]
    fn generate_rays_covers_every_ring_and_azimuth() {
        // One ray per (ring, azimuth) cell, with contiguous flat ids 0..N.
        let rays = two_ring_model().generate_rays();
        assert_eq!(rays.len(), 2 * 3);

        let ids: Vec<u32> = rays.iter().map(|r| r.id).collect();
        assert_eq!(ids, (0..6).collect::<Vec<_>>());
    }

    #[test]
    fn full_circle_omits_the_duplicate_seam_beam() {
        // A limited FOV includes both endpoints, so the increment spans the whole
        // sweep across (n - 1) gaps.
        let limited = planar_model(); // 90° over 3 beams
        assert!((limited.azimuth_increment - FRAC_PI_2 / 2.0).abs() < 1e-12);

        // A full 360° sweep must not place a beam at both 0 and 2π (the same
        // direction), so its increment divides by n, not (n - 1).
        let spinning = LidarModel::new(TAU, 4, vec![0.0], 0.1, 30.0, 0.01, 0.1)
            .expect("configuration is valid");
        assert!((spinning.azimuth_increment - TAU / 4.0).abs() < 1e-12);
    }

    #[test]
    fn stamps_per_point_time_by_azimuth() {
        // The cloud is timed, and a later azimuth is sampled later in the sweep,
        // so it carries a larger offset; id 0 opens the sweep at offset 0.
        let mut rng = StdRng::seed_from_u64(3);
        let hits = [
            RayHit {
                ray_id: 0,
                distance: 2.0,
            },
            RayHit {
                ray_id: 2,
                distance: 2.0,
            },
        ];

        let cloud = cloud_of(planar_model().process_hits(&hits, &mut rng));
        let times = cloud.time().expect("built timed").as_slice();
        assert_eq!(times.len(), 2);
        assert_eq!(times[0], 0, "the first azimuth opens the sweep");
        assert!(
            times[1] > times[0],
            "a later azimuth carries a larger offset"
        );
    }

    #[test]
    fn flash_capture_has_zero_time_offsets() {
        // sweep_period 0 models a simultaneous capture: every point shares the
        // reading's instant, so all offsets are zero even across azimuths.
        let mut rng = StdRng::seed_from_u64(3);
        let flash = LidarModel::new(FRAC_PI_2, 3, vec![0.0], 0.0, 30.0, 0.01, 0.1)
            .expect("configuration is valid");
        let hits = [
            RayHit {
                ray_id: 0,
                distance: 2.0,
            },
            RayHit {
                ray_id: 2,
                distance: 2.0,
            },
        ];

        let cloud = cloud_of(flash.process_hits(&hits, &mut rng));
        assert_eq!(cloud.time().expect("built timed").as_slice(), &[0, 0]);
    }

    #[test]
    fn emits_one_point_per_hit() {
        let mut rng = StdRng::seed_from_u64(1);
        let hits = [
            RayHit {
                ray_id: 0,
                distance: 1.0,
            },
            RayHit {
                ray_id: 1,
                distance: 2.0,
            },
            RayHit {
                ray_id: 2,
                distance: 3.0,
            },
        ];

        let cloud = cloud_of(planar_model().process_hits(&hits, &mut rng));
        assert_eq!(cloud.len(), hits.len());
    }

    #[test]
    fn no_hits_yields_an_empty_cloud() {
        // Misses are dropped by the host, so an empty hit list is a valid scan
        // that returned nothing; the builder must finalize to an empty cloud.
        let mut rng = StdRng::seed_from_u64(1);
        let cloud = cloud_of(planar_model().process_hits(&[], &mut rng));
        assert!(cloud.is_empty());
    }

    #[test]
    fn new_rejects_unusable_configurations() {
        // No beams, no rings, or non-positive noise each fail construction.
        assert!(LidarModel::new(FRAC_PI_2, 0, vec![0.0], 0.1, 30.0, 0.01, 0.1).is_none());
        assert!(LidarModel::new(FRAC_PI_2, 3, vec![], 0.1, 30.0, 0.01, 0.1).is_none());
        assert!(LidarModel::new(FRAC_PI_2, 3, vec![0.0], 0.1, 30.0, 0.0, 0.1).is_none());
        assert!(LidarModel::new(FRAC_PI_2, 3, vec![0.0], 0.1, 30.0, 0.01, 0.0).is_none());
    }
}
