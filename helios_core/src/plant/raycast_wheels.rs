//! The L1 raycast wheel plant: a per-wheel suspension-and-tire map from
//! wheel-ground contacts and an actuator command to a body-frame wrench.
//!
//! It sits one rung above [`L0ShimPlant`](super::l0_shim::L0ShimPlant) on the
//! fidelity ladder. Where the shim folds a command straight into one chassis
//! wrench, this plant carries each wheel's geometry and lets the ground it
//! stands on shape the forces — so weight transfer, per-surface grip, and
//! understeer emerge from the model rather than being faked.
//!
//! It runs in two phases split across the core/host boundary, mirroring a
//! world sensor: the plant generates one downward suspension ray per wheel
//! ([`generate_rays`](RaycastWheelPlant::generate_rays)), the host casts those
//! against the scene it alone owns, and the plant folds the resulting contacts,
//! the body twist, and the command into a wrench. This is the ray-generation
//! half; the force computation that consumes the contacts is the plant's other
//! half, which is why the tire and most suspension parameters are held here but
//! not yet read.

use crate::frames::{
    conventions::Flu,
    quantities::{FluVector, Point},
};

/// A four-corner (or N-corner) car body modeled as independent wheels on
/// spring-damper suspensions, each generating a downward ray to find the ground.
///
/// The geometry is static — mount offsets, axle assignment, and drive/steer
/// roles are fixed at construction (the host resolves them from config before
/// building the plant). Only the ground contacts and the command change per
/// tick.
pub struct RaycastWheelPlant {
    wheels: Vec<Wheel>,
    suspension: SuspensionParams,
    tire: TireParams,
}

impl RaycastWheelPlant {
    /// Build a plant from already-resolved wheels and shared parameters.
    ///
    /// Each [`Wheel`] arrives with its role flags (`is_drive` / `is_steer`)
    /// already decided — the host maps config wheel-name lists to those bools
    /// before calling this, so the plant never sees a wheel name. The parameters
    /// are shared across every wheel.
    pub fn new(wheels: Vec<Wheel>, suspension: SuspensionParams, tire: TireParams) -> Self {
        Self {
            wheels,
            suspension,
            tire,
        }
    }

    /// One downward suspension ray per wheel, in body [`Flu`], for the host to
    /// cast against the world.
    ///
    /// Every ray starts at its wheel's mount offset, points body-down, and
    /// reaches `rest_length + wheel_radius + ray_margin`: far enough that a wheel
    /// anywhere in its suspension travel finds the ground, and a wheel lifted
    /// past the margin reports no hit and counts as airborne.
    ///
    /// The ray is deliberately steer-independent — it only *finds* the ground.
    /// Steering changes the direction the tire pushes, not where the wheel sits,
    /// so the steer angle enters the force computation, never ray generation.
    /// The host casts each ray excluding the vehicle's own colliders (a wheel
    /// must not hit its own chassis) and rotates it from body to world first.
    pub fn generate_rays(&self) -> Vec<WheelRay> {
        self.wheels
            .iter()
            .enumerate()
            .map(|(wheel, w)| WheelRay {
                wheel,
                origin: w.offset,
                direction: Self::suspension_axis_flu(),
                max_distance: self.suspension.rest_length
                    + self.suspension.wheel_radius
                    + self.suspension.ray_margin,
            })
            .collect()
    }

    /// The body-down unit vector the suspension compresses along: `-Z`, because
    /// [`Flu`]'s up is `+Z`. Named once here rather than inlined so the axis
    /// convention lives in a single, stated place.
    fn suspension_axis_flu() -> FluVector {
        FluVector::new(0.0, 0.0, -1.0)
    }
}

/// One wheel's static description: where it is mounted and what it does.
pub struct Wheel {
    /// Mount position in the body [`Flu`] frame — where the suspension attaches
    /// to the chassis, and where the ray originates.
    offset: Point<Flu>,
    /// Which axle the wheel belongs to, selecting its per-axle tire behavior.
    axle: Axle,
    /// Whether drive torque reaches this wheel.
    is_drive: bool,
    /// Whether this wheel steers.
    is_steer: bool,
}

impl Wheel {
    pub fn new(offset: Point<Flu>, axle: Axle, is_drive: bool, is_steer: bool) -> Self {
        Self {
            offset,
            axle,
            is_drive,
            is_steer,
        }
    }
}

/// Which axle a wheel is on. The front and rear axles carry different cornering
/// stiffness, and that front/rear ratio is what sets a car's understeer balance.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Axle {
    Front,
    Rear,
}

/// Spring-damper and geometry parameters shared by every wheel's suspension.
pub struct SuspensionParams {
    /// Uncompressed suspension length: mount-to-hub distance at full droop.
    rest_length: f64,
    /// Wheel radius — hub-to-ground at zero compression is `rest_length + wheel_radius`.
    wheel_radius: f64,
    /// Spring stiffness `k` (N/m): normal load per unit of compression.
    stiffness: f64,
    /// Damping `c` (N·s/m): normal load per unit of compression *rate*.
    damping: f64,
    /// Maximum compression travel before the suspension bottoms out (m).
    max_travel: f64,
    /// How far past full droop the ray still reports a hit before the wheel
    /// counts as airborne (m). Extends [`generate_rays`]'s ray length beyond
    /// `rest_length + wheel_radius`.
    ray_margin: f64,
}

impl SuspensionParams {
    pub fn new(
        rest_length: f64,
        wheel_radius: f64,
        stiffness: f64,
        damping: f64,
        max_travel: f64,
        ray_margin: f64,
    ) -> Self {
        Self {
            rest_length,
            wheel_radius,
            stiffness,
            damping,
            max_travel,
            ray_margin,
        }
    }
}

/// Tire parameters shared across wheels — per-axle cornering stiffness, rolling
/// resistance, and the low-speed slip regularization threshold.
///
/// Empty for now: these are read only by the force computation, which is the
/// plant's other half. The type exists so the plant's shape is complete and the
/// host builds it the same way it will once the fields land.
pub struct TireParams {}

impl TireParams {
    pub fn new() -> Self {
        Self {}
    }
}

/// A single suspension ray for the host to cast, in the body [`Flu`] frame.
pub struct WheelRay {
    /// Index of the wheel this ray belongs to, back into the plant's wheel list.
    /// The host returns contacts aligned to this index so the force computation
    /// can match a hit to its wheel.
    pub wheel: usize,
    /// Ray start: the wheel's mount offset in body [`Flu`].
    pub origin: Point<Flu>,
    /// Ray direction: body-down (`-Z`). The host rotates it to world before casting.
    pub direction: FluVector,
    /// Farthest distance a hit is reported at; beyond it the wheel is airborne.
    pub max_distance: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    // A conventional four-corner car. The corner offsets are all distinct so a
    // test that transposed two wheels, or read the wrong axis out of an offset,
    // fails on position rather than passing by coincidence. The suspension
    // magnitudes that ray generation does not touch (stiffness, damping,
    // max_travel) are set far from the three that feed `max_distance`, so a ray
    // that summed the wrong field would miss.
    fn plant() -> RaycastWheelPlant {
        let wheels = vec![
            Wheel::new(Point::new(1.4, 0.8, -0.3), Axle::Front, false, true),
            Wheel::new(Point::new(1.4, -0.8, -0.3), Axle::Front, false, true),
            Wheel::new(Point::new(-1.4, 0.8, -0.3), Axle::Rear, true, false),
            Wheel::new(Point::new(-1.4, -0.8, -0.3), Axle::Rear, true, false),
        ];
        let suspension = SuspensionParams::new(0.40, 0.30, 50_000.0, 4_000.0, 0.20, 0.10);
        RaycastWheelPlant::new(wheels, suspension, TireParams::new())
    }

    #[test]
    fn one_ray_per_wheel_indexed_in_order() {
        // One ray per wheel, and each ray's `wheel` id is its position in the
        // list — the alignment the host casts contacts back on.
        let rays = plant().generate_rays();

        assert_eq!(rays.len(), 4);
        for (i, ray) in rays.iter().enumerate() {
            assert_eq!(ray.wheel, i);
        }
    }

    #[test]
    fn ray_origin_is_the_wheel_mount_offset() {
        // Each ray starts exactly at its wheel's configured mount point.
        let rays = plant().generate_rays();

        assert_eq!(rays[0].origin, Point::new(1.4, 0.8, -0.3));
        assert_eq!(rays[1].origin, Point::new(1.4, -0.8, -0.3));
        assert_eq!(rays[2].origin, Point::new(-1.4, 0.8, -0.3));
        assert_eq!(rays[3].origin, Point::new(-1.4, -0.8, -0.3));
    }

    #[test]
    fn every_ray_points_body_down() {
        // Body-down is -Z in FLU (up is +Z). The ray is identical for every
        // wheel and carries no steer dependence — it only finds the ground.
        for ray in plant().generate_rays() {
            assert_eq!(ray.direction, FluVector::new(0.0, 0.0, -1.0));
        }
    }

    #[test]
    fn max_distance_reaches_full_droop_plus_margin() {
        // rest_length + wheel_radius is the hub-to-ground distance at zero
        // compression; ray_margin is how much further the ray still reports a
        // hit before the wheel counts as airborne.
        for ray in plant().generate_rays() {
            assert_eq!(ray.max_distance, 0.40 + 0.30 + 0.10);
        }
    }

    #[test]
    fn empty_wheel_list_generates_no_rays() {
        // The degenerate body: no wheels yields no rays rather than panicking.
        let bare = RaycastWheelPlant::new(
            vec![],
            SuspensionParams::new(0.40, 0.30, 50_000.0, 4_000.0, 0.20, 0.10),
            TireParams::new(),
        );

        assert!(bare.generate_rays().is_empty());
    }
}
