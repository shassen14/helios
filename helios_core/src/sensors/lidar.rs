//! The forward (truth) model of a ray lidar: which rays it casts and what it
//! reports for their returns.
//!
//! The lidar is a world sensor, so it runs in two phases around the host's
//! raycast: [`LidarModel`] generates the rays, the host casts them into its scene,
//! and the model packs the returns into an organized [`RangeField`] in the
//! sensor's FLU frame. Everything a real unit gets wrong lives here, not in the
//! host.
//!
//! [`RangeField`]: crate::prelude::RangeField

use crate::prelude::{
    DirectionModel, RangeFieldBuilder, ScanTiming, SphericalAngular, NO_INFORMATION,
};
use crate::sensors::{RayHit, RaycastingOutput, RaycastingSensorModel, SensorRay};
use crate::spatial::conventions::Flu;
use crate::spatial::primitives::NANOS_PER_SECOND;

use rand::RngCore;
use rand_distr::{Distribution, Normal};

/// Forward (truth) model for a ray lidar of any beam layout — 2D planar,
/// spinning multi-ring, forward-looking solid-state, or flash. The layout is not
/// a type; it is entirely the beam geometry. A 2D lidar is a single ring at `0.0`
/// elevation, a spinning unit is a `2π` azimuth sweep over a list of ring
/// elevations, and a flash unit has a `sweep_period` of zero.
///
/// Each scan is a [`RangeField`](crate::prelude::RangeField) with one cell per
/// beam: rows are rings, columns are azimuths. Noise follows the physics of a
/// real unit. Pointing error perturbs each beam's azimuth and elevation before it
/// is cast, so it shows up where it does on hardware: as a range error that
/// depends on what the beam struck. Range error is added to each return. The
/// range is written into the beam's nominal cell, because that is how a driver
/// organizes a scan.
///
/// The model is deliberately richer than any consumer's belief, so simulating
/// against it is not an inverse crime.
#[derive(Debug, Clone)]
pub struct LidarModel {
    /// The beam pattern: ring elevations and a uniform azimuth sweep. It is the
    /// whole of what distinguishes one lidar's layout from another's.
    geometry: SphericalAngular,
    /// An all-miss grid shaped by `geometry`, with the sweep's per-column timing
    /// already attached. Built and checked once in `new`; every scan fills a
    /// copy, so a scan has nothing left to reject.
    empty_scan: RangeFieldBuilder<Flu>,
    /// The shortest return the unit reports, in meters. Anything nearer is a miss.
    range_min: f64,
    /// The longest return the unit reports, in meters. Anything farther is a miss.
    range_max: f64,
    /// The unit's range and pointing error.
    noise: LidarNoise,
}

impl LidarModel {
    /// Builds the model around a beam `geometry`, or returns `None` for a
    /// negative or non-finite `sweep_period`, or range limits that are
    /// negative, non-finite, or leave no span (`range_min >= range_max`).
    ///
    /// `sweep_period` is the seconds one azimuth sweep takes; zero is a flash
    /// capture, whose scans carry no timing. Range limits are meters.
    pub fn new(
        geometry: SphericalAngular,
        sweep_period: f64,
        range_min: f64,
        range_max: f64,
        noise: LidarNoise,
    ) -> Option<Self> {
        if !sweep_period.is_finite() || sweep_period < 0.0 {
            return None;
        }

        let direction = DirectionModel::SphericalAngular(geometry.clone());
        let mut empty_scan = RangeFieldBuilder::new(direction, range_min, range_max).ok()?;

        // A spinning unit fires each azimuth column at its own fraction of the
        // sweep, every ring in that column together. A flash capture has no
        // offsets to record.
        if sweep_period > 0.0 {
            let n_azimuth = geometry.n_azimuth();
            let mut offsets = Vec::with_capacity(n_azimuth as usize);
            for c in 0..n_azimuth {
                let fraction = c as f64 / n_azimuth as f64;
                offsets.push((fraction * sweep_period * NANOS_PER_SECOND) as i32);
            }

            empty_scan = empty_scan
                .with_timing(ScanTiming::PerColumn(offsets))
                .ok()?;
        }

        Some(Self {
            geometry,
            empty_scan,
            range_min,
            range_max,
            noise,
        })
    }

    /// The flat id of the beam in cell `(row, col)`: rows are rings, columns are
    /// azimuths, numbered ring by ring. It is the id a ray carries out to the
    /// host and back on its hit.
    fn ray_id(&self, row: usize, col: usize) -> u32 {
        (row * self.geometry.n_azimuth() as usize + col) as u32
    }

    /// The `(row, col)` cell a flat beam id names, inverting [`Self::ray_id`].
    ///
    /// Not bounds-checked: an id past the last beam yields a row past the last
    /// ring, which the scan's `set` rejects.
    fn cell_of(&self, id: u32) -> (usize, usize) {
        (
            (id / self.geometry.n_azimuth()) as usize,
            (id % self.geometry.n_azimuth()) as usize,
        )
    }
}

impl RaycastingSensorModel for LidarModel {
    /// Casts one ray per beam, each perturbed by pointing error.
    ///
    /// A real unit's encoder and ring mounts are each slightly off, so every
    /// beam's azimuth and elevation get their own independent error before the
    /// angles become a direction. Perturbing the angles, rather than a point
    /// after the hit, keeps the error geometry-dependent the way it is on
    /// hardware: small against a wall faced head-on, large at a grazing angle.
    fn generate_rays(&self, rng: &mut dyn RngCore) -> Vec<SensorRay> {
        let (rows, cols) = self.geometry.shape();

        let mut rays = Vec::with_capacity(rows * cols);

        for row in 0..rows {
            for col in 0..cols {
                let Some(mut beam_angles) = self.geometry.beam_angles(row, col) else {
                    continue;
                };

                let azimuth_noise = self.noise.angular.sample(rng);
                let elevation_noise = self.noise.angular.sample(rng);
                beam_angles.azimuth += azimuth_noise;
                beam_angles.elevation += elevation_noise;

                let direction = SphericalAngular::unit_vector(beam_angles);

                rays.push(SensorRay {
                    id: self.ray_id(row, col),
                    direction,
                });
            }
        }

        rays
    }

    /// Packs the host's returns into one scan, adding range noise to each.
    ///
    /// Each return lands in its beam's *nominal* cell: the pointing error was
    /// already spent when the ray was cast, so it shows up here only through the
    /// distance the ray travelled.
    ///
    /// A noisy range the unit would not report becomes one of two misses,
    /// depending on what it says about the world:
    /// - beyond `range_max`: the beam crossed empty space out to the limit, so
    ///   the cell keeps [`NOTHING_RETURNED`], as does every ray that hit
    ///   nothing and so never arrives;
    /// - below `range_min`, or a non-finite distance from the host: something
    ///   may be there but the unit cannot say where, so the cell is
    ///   [`NO_INFORMATION`].
    ///
    /// A ray id outside the grid came from a faulty host and is skipped rather
    /// than panicking.
    ///
    /// [`NOTHING_RETURNED`]: crate::prelude::NOTHING_RETURNED
    fn process_hits(&self, hits: &[RayHit], rng: &mut dyn RngCore) -> RaycastingOutput {
        let mut scan = self.empty_scan.clone();
        for hit in hits {
            let (row, col) = self.cell_of(hit.ray_id);

            // Drawn for every hit, even one about to be discarded, so each hit
            // consumes the same draws and a seed reproduces the whole scan.
            let noisy_range = hit.distance as f64 + self.noise.range.sample(rng);

            // Non-finite first: every comparison with NaN is false, so a NaN
            // would slip past both limit checks below.
            let range = if !noisy_range.is_finite() || noisy_range < self.range_min {
                NO_INFORMATION
            } else if noisy_range > self.range_max {
                continue;
            } else {
                noisy_range
            };

            // An id past the grid came from a faulty host: skip that return and
            // keep filling.
            if scan.set(row, col, range).is_err() {
                continue;
            }
        }

        RaycastingOutput::RangeField(scan.finalize())
    }

    fn get_max_range(&self) -> f32 {
        self.range_max as f32
    }
}

/// A lidar's measurement error: a zero-mean Gaussian on each reported range,
/// and another on each beam's azimuth and elevation.
///
/// Grouped apart from the beam geometry because it is what varies between
/// units of the same layout, and where richer error models (a range error that
/// grows with distance, dropout) belong.
#[derive(Debug, Clone)]
pub struct LidarNoise {
    range: Normal<f64>,
    angular: Normal<f64>,
}

impl LidarNoise {
    /// Builds the noise from standard deviations in meters (`range_stddev`)
    /// and radians (`angular_stddev`), or returns `None` unless both are finite
    /// and strictly positive.
    pub fn new(range_stddev: f64, angular_stddev: f64) -> Option<Self> {
        let usable = |stddev: f64| stddev.is_finite() && stddev > 0.0;
        if !usable(range_stddev) || !usable(angular_stddev) {
            return None;
        }

        Some(Self {
            range: Normal::new(0.0, range_stddev).ok()?,
            angular: Normal::new(0.0, angular_stddev).ok()?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::prelude::{RangeField, NOTHING_RETURNED};

    use std::f64::consts::FRAC_PI_2;

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    const SWEEP_PERIOD: f64 = 0.1;
    const RANGE_MIN: f64 = 0.1;
    const RANGE_MAX: f64 = 30.0;
    /// Range noise standard deviation, in meters.
    const RANGE_NOISE: f64 = 0.01;
    /// Angular noise standard deviation, in radians (about 0.1°).
    const ANGULAR_NOISE: f64 = 0.001_745;

    /// Ten standard deviations of angular noise: a bound no seed crosses in
    /// practice, and still far tighter than the 45° beam spacing.
    const ANGULAR_BOUND: f64 = 10.0 * ANGULAR_NOISE;
    /// Ten standard deviations of range noise, in meters.
    const RANGE_BOUND: f64 = 10.0 * RANGE_NOISE;

    /// A 3-beam, 90° forward sector over the given rings.
    fn sector(ring_elevations: Vec<f64>) -> SphericalAngular {
        SphericalAngular::from_field_of_view(ring_elevations, FRAC_PI_2, 3).expect("valid geometry")
    }

    fn noise() -> LidarNoise {
        LidarNoise::new(RANGE_NOISE, ANGULAR_NOISE).expect("valid noise")
    }

    fn model(ring_elevations: Vec<f64>, sweep_period: f64) -> LidarModel {
        LidarModel::new(
            sector(ring_elevations),
            sweep_period,
            RANGE_MIN,
            RANGE_MAX,
            noise(),
        )
        .expect("configuration is valid")
    }

    /// A planar (2D) model: one ring at 0° over a 90° forward sector of 3 beams.
    /// A sector puts a beam on each edge, so beams sit at -45°/0°/+45° and the
    /// middle one (column 1) points straight ahead.
    fn planar_model() -> LidarModel {
        model(vec![0.0], SWEEP_PERIOD)
    }

    /// Two rings, 0.3 rad up and 0.3 rad down, over the same 3-beam sector.
    fn two_ring_model() -> LidarModel {
        model(vec![0.3, -0.3], SWEEP_PERIOD)
    }

    fn field_of(output: RaycastingOutput) -> RangeField<Flu> {
        let RaycastingOutput::RangeField(field) = output;
        field
    }

    fn hit(ray_id: u32, distance: f32) -> RayHit {
        RayHit { ray_id, distance }
    }

    /// Asserts every cell except those listed is [`NOTHING_RETURNED`].
    fn assert_misses_except(field: &RangeField<Flu>, filled: &[(usize, usize)]) {
        let (rows, cols) = field.shape();
        for row in 0..rows {
            for col in 0..cols {
                if !filled.contains(&(row, col)) {
                    assert_eq!(
                        field.range(row, col),
                        Some(NOTHING_RETURNED),
                        "({row}, {col}) should have returned nothing"
                    );
                }
            }
        }
    }

    /// Whether `(row, col)` holds [`NO_INFORMATION`]. Checked with `is_nan`
    /// because `NaN` never compares equal, not even to itself.
    fn has_no_information(field: &RangeField<Flu>, row: usize, col: usize) -> bool {
        field.range(row, col).is_some_and(f64::is_nan)
    }

    #[test]
    fn new_rejects_unusable_configurations() {
        let build = |sweep: f64, min: f64, max: f64| {
            LidarModel::new(sector(vec![0.0]), sweep, min, max, noise())
        };

        assert!(build(-0.1, RANGE_MIN, RANGE_MAX).is_none());
        assert!(build(f64::NAN, RANGE_MIN, RANGE_MAX).is_none());
        assert!(build(SWEEP_PERIOD, -1.0, RANGE_MAX).is_none());
        assert!(build(SWEEP_PERIOD, RANGE_MAX, RANGE_MIN).is_none());
        assert!(build(SWEEP_PERIOD, RANGE_MIN, f64::INFINITY).is_none());
    }

    #[test]
    fn noise_must_be_finite_and_strictly_positive() {
        assert!(LidarNoise::new(RANGE_NOISE, ANGULAR_NOISE).is_some());
        for bad in [0.0, -0.1, f64::NAN, f64::INFINITY] {
            assert!(LidarNoise::new(bad, ANGULAR_NOISE).is_none(), "range {bad}");
            assert!(LidarNoise::new(RANGE_NOISE, bad).is_none(), "angular {bad}");
        }
    }

    #[test]
    fn get_max_range_reports_the_upper_limit() {
        assert_eq!(planar_model().get_max_range(), RANGE_MAX as f32);
    }

    #[test]
    fn generate_rays_casts_one_unit_ray_per_beam_with_distinct_ids() {
        let mut rng = StdRng::seed_from_u64(1);
        let rays = two_ring_model().generate_rays(&mut rng);

        assert_eq!(rays.len(), 2 * 3);
        let mut ids: Vec<u32> = rays.iter().map(|ray| ray.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), rays.len(), "every beam needs its own id");

        for ray in &rays {
            let norm = ray.direction.norm();
            assert!((norm - 1.0).abs() < 1e-12, "ray {} has norm {norm}", ray.id);
        }
    }

    #[test]
    fn ray_ids_round_trip_to_their_cell() {
        let model = two_ring_model();
        for row in 0..2 {
            for col in 0..3 {
                assert_eq!(model.cell_of(model.ray_id(row, col)), (row, col));
            }
        }
    }

    #[test]
    fn each_ray_stays_near_its_beams_nominal_bearing() {
        let model = two_ring_model();
        let mut rng = StdRng::seed_from_u64(2);

        for ray in model.generate_rays(&mut rng) {
            let (row, col) = model.cell_of(ray.id);
            let nominal = model.geometry.bearing(row, col).expect("in the grid");
            let error = ray.direction.angle(&nominal);
            assert!(
                error < ANGULAR_BOUND,
                "ray {} is {error} rad off its nominal bearing",
                ray.id
            );
        }
    }

    #[test]
    fn rays_carry_pointing_error() {
        let model = two_ring_model();
        let mut rng = StdRng::seed_from_u64(3);

        let jittered = model.generate_rays(&mut rng).iter().any(|ray| {
            let (row, col) = model.cell_of(ray.id);
            let nominal = model.geometry.bearing(row, col).expect("in the grid");
            ray.direction.angle(&nominal) > 0.0
        });
        assert!(jittered, "no ray was perturbed off its nominal bearing");
    }

    #[test]
    fn a_seed_reproduces_the_same_rays_and_scan() {
        let model = two_ring_model();
        let run = |seed| {
            let mut rng = StdRng::seed_from_u64(seed);
            let rays = model.generate_rays(&mut rng);
            let field = field_of(model.process_hits(&[hit(4, 5.0)], &mut rng));
            (
                rays.iter().map(|ray| ray.direction).collect::<Vec<_>>(),
                field.range(1, 1),
            )
        };
        assert_eq!(run(7), run(7));
    }

    #[test]
    fn a_scan_is_shaped_rings_by_azimuths_and_carries_the_limits() {
        let mut rng = StdRng::seed_from_u64(4);
        let field = field_of(two_ring_model().process_hits(&[], &mut rng));

        assert_eq!(field.shape(), (2, 3));
        assert_eq!(field.range_min(), RANGE_MIN);
        assert_eq!(field.range_max(), RANGE_MAX);
        assert_misses_except(&field, &[]);
    }

    #[test]
    fn a_return_lands_in_its_beams_cell_with_range_noise() {
        // Id 4 is ring 1, column 1: the lower ring's forward beam.
        let mut rng = StdRng::seed_from_u64(5);
        let field = field_of(two_ring_model().process_hits(&[hit(4, 5.0)], &mut rng));

        let range = field.range(1, 1).expect("in the grid");
        assert!(
            (range - 5.0).abs() < RANGE_BOUND,
            "range should be 5 m up to noise, got {range}"
        );
        assert_misses_except(&field, &[(1, 1)]);
    }

    #[test]
    fn a_return_beyond_range_max_is_nothing_returned() {
        let mut rng = StdRng::seed_from_u64(6);
        let too_far = hit(0, (RANGE_MAX + 1.0) as f32);
        let field = field_of(planar_model().process_hits(&[too_far], &mut rng));

        assert_misses_except(&field, &[]);
    }

    #[test]
    fn a_return_below_range_min_is_no_information() {
        let mut rng = StdRng::seed_from_u64(6);
        let too_near = hit(2, (RANGE_MIN / 10.0) as f32);
        let field = field_of(planar_model().process_hits(&[too_near], &mut rng));

        assert!(has_no_information(&field, 0, 2));
        assert_misses_except(&field, &[(0, 2)]);
    }

    /// A zero-distance hit whose noise draw is negative yields a range below
    /// zero. `process_hits` draws exactly one sample per hit, so drawing from a
    /// fresh RNG with the same seed predicts it.
    #[test]
    fn a_negative_noisy_range_is_no_information() {
        let first_draw = |seed| noise().range.sample(&mut StdRng::seed_from_u64(seed));
        let seed = (0..)
            .find(|&seed| first_draw(seed) < 0.0)
            .expect("half of all draws");

        let mut rng = StdRng::seed_from_u64(seed);
        let field = field_of(planar_model().process_hits(&[hit(1, 0.0)], &mut rng));

        assert!(has_no_information(&field, 0, 1));
        assert_misses_except(&field, &[(0, 1)]);
    }

    #[test]
    fn a_non_finite_distance_from_the_host_is_no_information() {
        let mut rng = StdRng::seed_from_u64(8);
        let hits = [hit(0, f32::NAN), hit(1, 5.0), hit(2, f32::INFINITY)];
        let field = field_of(planar_model().process_hits(&hits, &mut rng));

        assert!(has_no_information(&field, 0, 0));
        assert!(has_no_information(&field, 0, 2));
        assert!(field.range(0, 1).is_some_and(f64::is_finite));
    }

    /// A discarded hit still consumes its noise draw, so a later hit's range
    /// does not depend on whether an earlier one was kept.
    #[test]
    fn a_discarded_hit_still_consumes_its_noise_draw() {
        let model = planar_model();
        let second_range = |first: RayHit| {
            let mut rng = StdRng::seed_from_u64(9);
            field_of(model.process_hits(&[first, hit(1, 5.0)], &mut rng)).range(0, 1)
        };

        let kept = second_range(hit(0, 5.0));
        assert_eq!(second_range(hit(0, f32::NAN)), kept);
        assert_eq!(second_range(hit(0, (RANGE_MAX + 1.0) as f32)), kept);
    }

    #[test]
    fn no_information_cells_never_reach_the_cloud() {
        let mut rng = StdRng::seed_from_u64(10);
        let hits = [
            hit(0, f32::NAN),
            hit(1, 5.0),
            hit(2, (RANGE_MIN / 10.0) as f32),
        ];
        let field = field_of(planar_model().process_hits(&hits, &mut rng));

        assert_eq!(field.to_point_cloud().len(), 1);
    }

    #[test]
    fn a_ray_id_outside_the_grid_is_skipped() {
        let mut rng = StdRng::seed_from_u64(7);
        let field = field_of(planar_model().process_hits(&[hit(99, 5.0), hit(1, 5.0)], &mut rng));

        assert_misses_except(&field, &[(0, 1)]);
    }

    #[test]
    fn a_forward_return_deprojects_straight_ahead() {
        // The planar model's forward beam is column 1. End to end through the
        // field's deprojection, a 5 m return must land on +x.
        let mut rng = StdRng::seed_from_u64(8);
        let field = field_of(planar_model().process_hits(&[hit(1, 5.0)], &mut rng));

        let cloud = field.to_point_cloud();
        assert_eq!(cloud.len(), 1);
        let p = cloud.point(0).into_inner();
        assert!(
            (p.x - 5.0).abs() < RANGE_BOUND,
            "x should track range, got {}",
            p.x
        );
        assert!(
            p.y.abs() < 1e-12,
            "the nominal forward beam has no y, got {}",
            p.y
        );
        assert!(p.z.abs() < 1e-12, "a 0° ring has no z, got {}", p.z);
    }

    #[test]
    fn a_spinning_scan_times_each_column_by_its_share_of_the_sweep() {
        let mut rng = StdRng::seed_from_u64(9);
        let field = field_of(planar_model().process_hits(&[], &mut rng));

        let Some(ScanTiming::PerColumn(offsets)) = field.timing() else {
            panic!("a spinning scan should be timed per column");
        };
        assert_eq!(offsets.len(), 3);
        for (col, &offset) in offsets.iter().enumerate() {
            let expected = col as f64 / 3.0 * SWEEP_PERIOD * NANOS_PER_SECOND;
            assert!(
                (offset as f64 - expected).abs() <= 1.0,
                "column {col}: expected {expected} ns, got {offset}"
            );
        }
    }

    #[test]
    fn a_flash_scan_carries_no_timing() {
        let mut rng = StdRng::seed_from_u64(10);
        let field = field_of(model(vec![0.0], 0.0).process_hits(&[], &mut rng));

        assert!(field.timing().is_none());
    }
}
