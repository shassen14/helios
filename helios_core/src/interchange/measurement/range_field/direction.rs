//! How an organized range grid's cells map to directions in space.
//!
//! A [`DirectionModel`] turns a cell index `(row, col)` into the beam's nominal
//! direction and, given a range, into the point that beam struck. Each variant
//! owns both its projection and its grid shape, so a grid can never disagree
//! with the model that addresses it. Results are raw vectors in the model's
//! native sensor frame; the grid that carries the model attaches the frame tag.

use nalgebra::Vector3;

use std::f64::consts::TAU;

/// How close to a full turn, in radians, a field of view must be to count as
/// one. Absorbs the rounding in a configured `360°` converted to radians.
const FULL_CIRCLE_TOLERANCE: f64 = 1e-9;

/// The projection that addresses a range grid's cells.
///
/// Closed on purpose: each variant is a family of beam geometry, not a device.
/// Every method matches once and delegates to the variant, so the per-family
/// math lives on the variant's own type.
#[derive(Debug, Clone, PartialEq)]
pub enum DirectionModel {
    /// Beams at fixed (azimuth, elevation) angles — every angular lidar,
    /// from a single-ring 2D scanner to a multi-ring spinning unit.
    SphericalAngular(SphericalAngular),
}

impl DirectionModel {
    /// The grid dimensions this model addresses, as `(rows, cols)`.
    pub fn shape(&self) -> (usize, usize) {
        match self {
            Self::SphericalAngular(model) => model.shape(),
        }
    }

    /// Whether the grid is a single flat scan through the sensor's horizontal
    /// plane — what a 2D consumer requires.
    pub fn is_planar(&self) -> bool {
        match self {
            Self::SphericalAngular(model) => model.is_planar(),
        }
    }

    /// The unit direction of the beam at `(row, col)`, or `None` if the index
    /// lies outside [`shape`](Self::shape).
    pub fn bearing(&self, row: usize, col: usize) -> Option<Vector3<f64>> {
        match self {
            Self::SphericalAngular(model) => model.bearing(row, col),
        }
    }

    /// The point struck by the beam at `(row, col)` when it returned `range`
    /// in the model's native unit, or `None` for a miss (a non-finite range)
    /// or an index outside [`shape`](Self::shape).
    pub fn deproject(&self, row: usize, col: usize, range: f64) -> Option<Vector3<f64>> {
        match self {
            Self::SphericalAngular(model) => model.deproject(row, col, range),
        }
    }
}

/// Beams laid out on a (ring × azimuth) grid, in the sensor's FLU frame.
///
/// Rows are rings, each at its own elevation — an explicit list, because
/// datasheet ring tables are rarely uniform. Columns are azimuths, a uniform
/// sweep starting at `azimuth_min`. Azimuth turns about `+z` (positive points
/// left), and elevation tilts up from the `xy` plane. Ranges are radial: the
/// distance along the beam. All angles are radians.
#[derive(Debug, Clone, PartialEq)]
pub struct SphericalAngular {
    ring_elevations: Vec<f64>,
    azimuth_min: f64,
    azimuth_increment: f64,
    n_azimuth: u32,
}

impl SphericalAngular {
    /// Builds the geometry, or returns `None` if it addresses no cells (no
    /// rings or no azimuths) or any angle is non-finite.
    pub fn new(
        ring_elevations: Vec<f64>,
        azimuth_min: f64,
        azimuth_increment: f64,
        n_azimuth: u32,
    ) -> Option<Self> {
        let angles_finite = azimuth_min.is_finite()
            && azimuth_increment.is_finite()
            && ring_elevations
                .iter()
                .all(|elevation| elevation.is_finite());

        if n_azimuth == 0 || ring_elevations.is_empty() || !angles_finite {
            return None;
        }

        Some(Self {
            ring_elevations,
            azimuth_min,
            azimuth_increment,
            n_azimuth,
        })
    }

    /// Builds the geometry from how a datasheet states it: `n_azimuth` beams
    /// spread across `azimuth_fov` radians, centred on straight ahead.
    ///
    /// A full turn spreads its beams evenly around the circle, so the last beam
    /// does not repeat the first at the seam. A sector puts a beam on each edge.
    /// A single beam points straight ahead whatever the field of view. Returns
    /// `None` for a negative or non-finite field of view, and for anything
    /// [`new`](Self::new) rejects.
    pub fn from_field_of_view(
        ring_elevations: Vec<f64>,
        azimuth_fov: f64,
        n_azimuth: u32,
    ) -> Option<Self> {
        if !azimuth_fov.is_finite() || azimuth_fov < 0.0 {
            return None;
        }

        let full_circle = azimuth_fov >= TAU - FULL_CIRCLE_TOLERANCE;
        let azimuth_increment = if n_azimuth <= 1 {
            0.0
        } else if full_circle {
            azimuth_fov / n_azimuth as f64
        } else {
            azimuth_fov / (n_azimuth - 1) as f64
        };
        let azimuth_min = if n_azimuth <= 1 {
            0.0
        } else {
            -azimuth_fov / 2.0
        };

        Self::new(ring_elevations, azimuth_min, azimuth_increment, n_azimuth)
    }

    /// Projects a beam's angles to its unit direction: `x` forward, `y` left,
    /// `z` up. The one place the azimuth/elevation convention becomes a vector;
    /// [`bearing`](Self::bearing) and any caller that perturbs the angles first
    /// both project through here.
    pub fn unit_vector(angles: BeamAngles) -> Vector3<f64> {
        Vector3::new(
            angles.elevation.cos() * angles.azimuth.cos(),
            angles.elevation.cos() * angles.azimuth.sin(),
            angles.elevation.sin(),
        )
    }

    /// The grid dimensions, as `(rings, azimuths)`.
    pub fn shape(&self) -> (usize, usize) {
        (self.ring_elevations.len(), self.n_azimuth as usize)
    }

    /// The number of azimuth columns in the sweep.
    pub fn n_azimuth(&self) -> u32 {
        self.n_azimuth
    }

    /// Whether this is a single ring at zero elevation. The comparison is exact
    /// on purpose: a planar scan is configured as exactly `0.0` (zero degrees
    /// converts to zero radians exactly), and any tilt, however small, sweeps a
    /// cone rather than a plane.
    pub fn is_planar(&self) -> bool {
        self.ring_elevations == [0.0]
    }

    /// The nominal angles of the beam at `(ring, azimuth)` = `(row, col)`, or
    /// `None` if either index is out of range.
    pub fn beam_angles(&self, row: usize, col: usize) -> Option<BeamAngles> {
        let elevation = *self.ring_elevations.get(row)?;
        let azimuth = self.azimuth_of(col)?;

        Some(BeamAngles { azimuth, elevation })
    }

    /// The unit direction of the beam at `(ring, azimuth)` = `(row, col)`, or
    /// `None` if either index is out of range.
    pub fn bearing(&self, row: usize, col: usize) -> Option<Vector3<f64>> {
        let angles = self.beam_angles(row, col)?;
        Some(Self::unit_vector(angles))
    }

    /// The point `range` meters along the beam at `(row, col)`, or `None` for a
    /// miss (a non-finite range) or an out-of-range index.
    pub fn deproject(&self, row: usize, col: usize, range: f64) -> Option<Vector3<f64>> {
        if !range.is_finite() {
            return None;
        }

        Some(self.bearing(row, col)? * range)
    }

    /// The nominal azimuth of column `col`, or `None` past the last column.
    fn azimuth_of(&self, col: usize) -> Option<f64> {
        (col < self.n_azimuth as usize)
            .then_some(self.azimuth_min + col as f64 * self.azimuth_increment)
    }
}

/// The pointing of one beam, in radians, in [`SphericalAngular`]'s convention:
/// `azimuth` turns about `+z` (positive points left), `elevation` tilts up from
/// the `xy` plane. Named fields rather than a tuple because the grid indexes
/// them in the opposite order — row is elevation, column is azimuth — and a
/// swapped pair would still compile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeamAngles {
    pub azimuth: f64,
    pub elevation: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    /// Tolerance for comparing directions and points built from trig.
    const TOLERANCE: f64 = 1e-12;

    /// A 3-ring × 4-azimuth grid sweeping a full turn from azimuth zero in
    /// quarter turns, with rings below, at, and above the horizon.
    fn three_ring_quarter_sweep() -> SphericalAngular {
        SphericalAngular::new(vec![-FRAC_PI_4, 0.0, FRAC_PI_4], 0.0, FRAC_PI_2, 4)
            .expect("valid geometry")
    }

    fn assert_close(actual: Vector3<f64>, expected: Vector3<f64>) {
        assert!(
            (actual - expected).norm() < TOLERANCE,
            "expected {expected:?}, got {actual:?}"
        );
    }

    /// The azimuth of every column in the geometry's single sweep.
    fn azimuths(geometry: &SphericalAngular) -> Vec<f64> {
        (0..geometry.shape().1)
            .map(|col| geometry.beam_angles(0, col).expect("in the grid").azimuth)
            .collect()
    }

    fn assert_angles_close(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len());
        for (a, e) in actual.iter().zip(expected) {
            assert!(
                (a - e).abs() < TOLERANCE,
                "expected {expected:?}, got {actual:?}"
            );
        }
    }

    #[test]
    fn a_sector_puts_a_beam_on_each_edge() {
        let geometry =
            SphericalAngular::from_field_of_view(vec![0.0], FRAC_PI_2, 3).expect("valid");
        assert_angles_close(&azimuths(&geometry), &[-FRAC_PI_4, 0.0, FRAC_PI_4]);
    }

    #[test]
    fn a_full_turn_does_not_repeat_the_seam_beam() {
        let geometry = SphericalAngular::from_field_of_view(vec![0.0], TAU, 4).expect("valid");
        assert_angles_close(&azimuths(&geometry), &[-PI, -FRAC_PI_2, 0.0, FRAC_PI_2]);
    }

    #[test]
    fn a_single_beam_points_straight_ahead() {
        let geometry =
            SphericalAngular::from_field_of_view(vec![0.0], FRAC_PI_2, 1).expect("valid");
        assert_angles_close(&azimuths(&geometry), &[0.0]);
    }

    #[test]
    fn from_field_of_view_rejects_an_unusable_field_of_view() {
        for fov in [-0.1, f64::NAN, f64::INFINITY] {
            assert!(
                SphericalAngular::from_field_of_view(vec![0.0], fov, 3).is_none(),
                "fov {fov}"
            );
        }
        assert!(SphericalAngular::from_field_of_view(vec![], FRAC_PI_2, 3).is_none());
    }

    #[test]
    fn new_rejects_a_grid_with_no_cells() {
        assert!(SphericalAngular::new(vec![], 0.0, 0.1, 4).is_none());
        assert!(SphericalAngular::new(vec![0.0], 0.0, 0.1, 0).is_none());
    }

    #[test]
    fn new_rejects_non_finite_angles() {
        assert!(SphericalAngular::new(vec![0.0, f64::NAN], 0.0, 0.1, 4).is_none());
        assert!(SphericalAngular::new(vec![0.0], f64::INFINITY, 0.1, 4).is_none());
        assert!(SphericalAngular::new(vec![0.0], 0.0, f64::NAN, 4).is_none());
    }

    #[test]
    fn shape_is_rings_by_azimuths() {
        assert_eq!(three_ring_quarter_sweep().shape(), (3, 4));
    }

    #[test]
    fn zero_azimuth_zero_elevation_points_forward() {
        let model = three_ring_quarter_sweep();
        assert_close(model.bearing(1, 0).unwrap(), Vector3::x());
    }

    #[test]
    fn positive_azimuth_points_left() {
        let model = three_ring_quarter_sweep();
        assert_close(model.bearing(1, 1).unwrap(), Vector3::y());
    }

    #[test]
    fn positive_elevation_points_up() {
        let model = three_ring_quarter_sweep();
        let up_forward = Vector3::new(FRAC_PI_4.cos(), 0.0, FRAC_PI_4.sin());
        assert_close(model.bearing(2, 0).unwrap(), up_forward);
    }

    #[test]
    fn columns_step_by_the_azimuth_increment_from_azimuth_min() {
        let model = SphericalAngular::new(vec![0.0], -FRAC_PI_2, FRAC_PI_4, 3).unwrap();
        assert_close(model.bearing(0, 0).unwrap(), -Vector3::y());
        assert_close(
            model.bearing(0, 1).unwrap(),
            Vector3::new(FRAC_PI_4.cos(), -FRAC_PI_4.sin(), 0.0),
        );
        assert_close(model.bearing(0, 2).unwrap(), Vector3::x());
    }

    #[test]
    fn every_bearing_is_unit_length() {
        let model = three_ring_quarter_sweep();
        let (rows, cols) = model.shape();
        for row in 0..rows {
            for col in 0..cols {
                let norm = model.bearing(row, col).unwrap().norm();
                assert!(
                    (norm - 1.0).abs() < TOLERANCE,
                    "({row}, {col}) has norm {norm}"
                );
            }
        }
    }

    #[test]
    fn out_of_range_indices_yield_none() {
        let model = three_ring_quarter_sweep();
        assert!(model.bearing(3, 0).is_none());
        assert!(model.bearing(0, 4).is_none());
        assert!(model.deproject(3, 0, 1.0).is_none());
        assert!(model.deproject(0, 4, 1.0).is_none());
    }

    #[test]
    fn deproject_scales_the_bearing_by_the_range() {
        let model = three_ring_quarter_sweep();
        let range = 7.5;
        for (row, col) in [(0, 0), (1, 1), (2, 3)] {
            let expected = model.bearing(row, col).unwrap() * range;
            assert_close(model.deproject(row, col, range).unwrap(), expected);
        }
    }

    #[test]
    fn deproject_treats_a_non_finite_range_as_a_miss() {
        let model = three_ring_quarter_sweep();
        assert!(model.deproject(0, 0, f64::INFINITY).is_none());
        assert!(model.deproject(0, 0, f64::NAN).is_none());
    }

    #[test]
    fn a_single_ring_at_zero_elevation_is_planar() {
        let planar = SphericalAngular::new(vec![0.0], 0.0, 0.1, 8).unwrap();
        let negative_zero = SphericalAngular::new(vec![-0.0], 0.0, 0.1, 8).unwrap();
        assert!(planar.is_planar());
        assert!(negative_zero.is_planar());
    }

    #[test]
    fn a_tilted_ring_or_several_rings_are_not_planar() {
        let tilted = SphericalAngular::new(vec![0.01], 0.0, 0.1, 8).unwrap();
        assert!(!tilted.is_planar());
        assert!(!three_ring_quarter_sweep().is_planar());
    }

    #[test]
    fn beam_angles_read_the_rings_elevation_and_the_columns_azimuth() {
        let model = three_ring_quarter_sweep();
        assert_eq!(
            model.beam_angles(2, 3),
            Some(BeamAngles {
                azimuth: 3.0 * FRAC_PI_2,
                elevation: FRAC_PI_4,
            })
        );
        assert!(model.beam_angles(3, 0).is_none());
        assert!(model.beam_angles(0, 4).is_none());
    }

    #[test]
    fn bearing_is_the_unit_vector_of_the_beam_angles() {
        let model = three_ring_quarter_sweep();
        let (rows, cols) = model.shape();
        for row in 0..rows {
            for col in 0..cols {
                let angles = model.beam_angles(row, col).unwrap();
                assert_eq!(
                    model.bearing(row, col),
                    Some(SphericalAngular::unit_vector(angles))
                );
            }
        }
    }

    #[test]
    fn unit_vector_follows_the_flu_convention() {
        let project =
            |azimuth, elevation| SphericalAngular::unit_vector(BeamAngles { azimuth, elevation });
        assert_close(project(0.0, 0.0), Vector3::x());
        assert_close(project(FRAC_PI_2, 0.0), Vector3::y());
        assert_close(project(-FRAC_PI_2, 0.0), -Vector3::y());
        assert_close(project(0.0, FRAC_PI_2), Vector3::z());
        assert_close(project(0.0, -FRAC_PI_2), -Vector3::z());
    }

    #[test]
    fn direction_model_delegates_to_its_variant() {
        let variant = three_ring_quarter_sweep();
        let model = DirectionModel::SphericalAngular(variant.clone());
        assert_eq!(model.shape(), variant.shape());
        assert_eq!(model.is_planar(), variant.is_planar());
        assert_eq!(model.bearing(2, 3), variant.bearing(2, 3));
        assert_eq!(model.deproject(2, 3, 4.0), variant.deproject(2, 3, 4.0));
        assert_eq!(model.bearing(3, 0), None);
    }
}
