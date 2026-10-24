//! The core measurement attribute keys: per-point facts a sensor reports about
//! its own returns, shared by every producer and consumer of clouds and range
//! fields.
//!
//! Core keys have bare names (`intensity`, `ring`). Any other key is
//! namespaced by its owner (`ouster.signal`), so a third-party key can never
//! take a core name. Only measurements live here: interpretations of the world
//! (a class label, an instance id) are perception's keys, not a sensor's.

use super::key::{AttributeKey, TransformMarker};

/// Relative return strength, unitless, normalized to `[0, 1]`: higher is a
/// stronger return.
///
/// Comparable only within one sensor. Vendors disagree on what "intensity"
/// is, so this key fixes the scale rather than the physics: a driver divides
/// its raw value by the device's maximum, and the sim emits the same range, so
/// a threshold tuned in sim holds on hardware. Calibrated surface reflectance,
/// comparable across sensors, is a different quantity and a different key.
///
/// NaN where the sensor gave no value for a cell.
pub const INTENSITY: AttributeKey<f32> =
    AttributeKey::nan_blank("intensity", TransformMarker::Scalar);

/// The beam a point came from: its row in a spherical range field, which is its
/// index into the sensor's configured `ring_elevations`.
///
/// Indexes the configured list in its given order, which need not be sorted
/// by elevation: look the elevation up rather than assuming ring 0 is lowest.
/// Derived from position when a range field is flattened, so it is never
/// missing. Only a spherical direction model has rings; a pinhole row is a
/// pixel row, not a ring.
pub const RING: AttributeKey<u16> = AttributeKey::no_blank("ring", TransformMarker::Scalar);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interchange::measurement::attribute::key::BlankPolicy;

    /// An unwritten intensity cell must not read as a real (zero) reflectance.
    #[test]
    fn intensity_blanks_to_nan() {
        assert_eq!(INTENSITY.blank(), BlankPolicy::Nan);
    }

    /// Ring is derived from a cell's row, so it can never be missing.
    #[test]
    fn ring_has_no_blank() {
        assert_eq!(RING.blank(), BlankPolicy::NoBlank);
    }

    #[test]
    fn core_key_names_are_distinct() {
        assert_ne!(INTENSITY.name(), RING.name());
    }
}
