//! Forward model for a 3-axis magnetometer.

use crate::data::sensor::MagneticField3D;
use crate::sensors::noise::TriaxialGaussian;

use nalgebra::{UnitQuaternion, Vector3};
use rand::RngCore;

/// Forward model for a 3-axis magnetometer: reports the ambient magnetic field
/// expressed in the sensor frame, with bias and per-axis noise.
///
/// The ambient field is fixed in the world frame — a still sensor at a fixed
/// location always feels the same field — so it is stored once as
/// `reference_field` and rotated into the sensor frame each reading.
#[derive(Debug, Clone)]
pub struct MagnetometerModel {
    reference_field: Vector3<f64>,
    noise: TriaxialGaussian,
}

impl MagnetometerModel {
    /// Builds the model from the local geomagnetic reference. `declination` is
    /// the angle from true north toward east and `inclination` the downward dip
    /// below horizontal, both in radians; `magnitude` is the field strength.
    /// These resolve to a world-ENU vector (+X east, +Y north, +Z up):
    /// east and north from the horizontal component, up negated because a
    /// positive dip points downward. Returns `None` if any component of
    /// `stddev` is not strictly positive.
    pub fn from_reference_field(
        declination: f64,
        inclination: f64,
        magnitude: f64,
        bias: Vector3<f64>,
        stddev: Vector3<f64>,
    ) -> Option<Self> {
        let reference_field = Vector3::new(
            magnitude * inclination.cos() * declination.sin(),
            magnitude * inclination.cos() * declination.cos(),
            -magnitude * inclination.sin(),
        );
        Some(Self {
            reference_field,
            noise: TriaxialGaussian::new(bias, stddev)?,
        })
    }

    /// Noise-free, bias-free measurement: the world-frame reference field
    /// rotated into the sensor frame. `q_sensor_from_world` maps world vectors
    /// into the sensor's own axes.
    pub fn ideal(&self, q_sensor_from_world: UnitQuaternion<f64>) -> Vector3<f64> {
        q_sensor_from_world * self.reference_field
    }

    /// Full measurement: the ideal field perturbed by bias and noise.
    pub fn sample(
        &self,
        q_sensor_from_world: UnitQuaternion<f64>,
        rng: &mut dyn RngCore,
    ) -> MagneticField3D {
        let ideal = self.ideal(q_sensor_from_world);
        MagneticField3D {
            value: self.noise.apply(ideal, rng),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::FRAC_PI_2;

    use rand::rngs::StdRng;
    use rand::SeedableRng;

    fn model(declination: f64, inclination: f64, magnitude: f64) -> MagnetometerModel {
        MagnetometerModel::from_reference_field(
            declination,
            inclination,
            magnitude,
            Vector3::zeros(),
            Vector3::new(0.01, 0.01, 0.01),
        )
        .unwrap()
    }

    #[test]
    fn zero_declination_and_inclination_point_north() {
        // The degenerate case reproduces a unit field pointing due north.
        let got = model(0.0, 0.0, 1.0).ideal(UnitQuaternion::identity());
        assert!((got - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn positive_inclination_dips_the_field_downward() {
        // A 90° dip points the field straight down (-Z in ENU).
        let got = model(0.0, FRAC_PI_2, 1.0).ideal(UnitQuaternion::identity());
        assert!((got - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-9);
    }

    #[test]
    fn positive_declination_rotates_the_field_toward_east() {
        // A 90° declination swings the horizontal field from north to east.
        let got = model(FRAC_PI_2, 0.0, 1.0).ideal(UnitQuaternion::identity());
        assert!((got - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn from_reference_field_rejects_nonpositive_stddev() {
        assert!(MagnetometerModel::from_reference_field(
            0.0,
            0.0,
            1.0,
            Vector3::zeros(),
            Vector3::new(1.0, 1.0, 0.0),
        )
        .is_none());
    }

    #[test]
    fn sample_is_reproducible_under_equal_seeds() {
        let model = model(0.0, 0.0, 1.0);
        let q = UnitQuaternion::identity();

        let mut rng_a = StdRng::seed_from_u64(9);
        let mut rng_b = StdRng::seed_from_u64(9);

        assert_eq!(model.sample(q, &mut rng_a).value, model.sample(q, &mut rng_b).value);
    }
}
