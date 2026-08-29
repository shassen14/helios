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
//! the body twist, and the command into a wrench
//! ([`compute_wrench`](RaycastWheelPlant::compute_wrench)). Both the suspension
//! and the tire forces are pure body-frame functions of the ray-hit distances
//! and the instantaneous body twist — no per-wheel spring-length or wheel-spin
//! state — so the plant stays a stateless map the host calls each tick.

use crate::control::actuators::{ActuatorCommand, SetpointKind, SetpointValue};
use crate::control::commands::{BodyTwist, BodyWrench};
use crate::frames::{
    conventions::Flu,
    quantities::{FluVector, Point},
};
use crate::plant::PlantWrench;

use nalgebra::Vector3;

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

    /// Fold this tick's ground contacts, body twist, and resolved actuator
    /// command into a single body-frame ([`Flu`]) wrench about the centre of
    /// mass — the force-computation half of the two-phase map.
    ///
    /// The host has cast the [`generate_rays`](Self::generate_rays) rays and
    /// returns one `Option<WheelContact>` per wheel, aligned to the wheel index:
    /// `None` where the ray found no ground, so that wheel is airborne and
    /// contributes nothing. Each wheel in contact runs a spring-damper
    /// suspension to a normal load `N`, a friction-circle tyre for the
    /// longitudinal and lateral ground forces, and applies the result *at the
    /// wheel's mount offset*. The per-wheel forces and their moments about the
    /// centre of mass sum to the returned wrench — and because each force is
    /// applied off-centre, weight transfer, pitch/roll, and grip-limited
    /// cornering emerge from the geometry rather than being modelled separately.
    ///
    /// The command is read by command space, as [`L0ShimPlant::fold`] reads it:
    /// `Torque` setpoints are the drive torque, summed and then split evenly
    /// across the drive wheels; a `Position` setpoint is the steer angle,
    /// applied to the steer wheels to rotate their tyre-force basis. A
    /// `Velocity` or `Force` setpoint has no place in this model — it drives
    /// from wheel torque, not a commanded speed or a body force — so it
    /// contributes nothing and its actuator is reported in
    /// [`unsupported`](PlantWrench::unsupported).
    ///
    /// Stateless: every quantity is instantaneous from the hit distances and the
    /// body twist, so the suspension needs no spring-length integrator and the
    /// tyre no wheel-spin state. Expects an already-resolved command (see
    /// [`ActuationModel::resolve`](crate::control::actuation_model::ActuationModel::resolve)):
    /// every value is finite and speaks its actuator's declared space, so the
    /// fold never guards against `NaN`.
    ///
    /// [`L0ShimPlant::fold`]: super::l0_shim::L0ShimPlant::fold
    pub fn compute_wrench(
        &self,
        contacts: &[Option<WheelContact>],
        twist: BodyTwist,
        command: &ActuatorCommand,
    ) -> PlantWrench {
        let mut drive_torque = 0.0;
        let mut steer_angle = 0.0;
        let mut unsupported = Vec::new();
        for sp in command.setpoints() {
            match sp.value() {
                SetpointValue::Torque(t) => drive_torque += t,
                SetpointValue::Position(a) => steer_angle += a,
                SetpointValue::Velocity(_) | SetpointValue::Force(_) => {
                    unsupported.push(sp.actuator().clone())
                }
            }
        }

        // The drive torque is split evenly across the drive wheels; a car with no
        // drive wheel simply produces no longitudinal force (a config oddity, not
        // a panic). Steer wheels all take the same steer angle.
        let drive_wheels = self.wheels.iter().filter(|w| w.is_drive).count();
        let torque_per_drive_wheel = if drive_wheels > 0 {
            drive_torque / drive_wheels as f64
        } else {
            0.0
        };

        let mut force = Vector3::zeros();
        let mut torque = Vector3::zeros();
        for (wheel, contact) in self.wheels.iter().zip(contacts) {
            let Some(contact) = contact else { continue };
            let Some(wheel_force) =
                self.wheel_force(wheel, contact, twist, torque_per_drive_wheel, steer_angle)
            else {
                continue;
            };
            // Apply each wheel's force at its mount offset: the moment arm r × F
            // about the centre of mass is what turns four off-centre forces into
            // body pitch, roll, and yaw.
            force += wheel_force;
            torque += wheel.offset.raw().cross(&wheel_force);
        }

        PlantWrench {
            wrench: BodyWrench::new(FluVector::from_raw(force), FluVector::from_raw(torque)),
            unsupported,
        }
    }

    /// Whether this plant can apply a setpoint of the given command space.
    ///
    /// The L1 raycast plant drives from wheel `Torque` and steers from a
    /// `Position` angle; a `Velocity` or `Force` setpoint has no place in the
    /// model. This is the same partition [`compute_wrench`](Self::compute_wrench)
    /// applies per setpoint, exposed ahead of any command so a host can reject an
    /// incompatible actuation contract at spawn rather than warn-and-ignore every
    /// tick. The wildcard-free match keeps the two in step: a new
    /// [`SetpointKind`] fails to compile here until it is classified.
    pub fn accepts(&self, kind: SetpointKind) -> bool {
        match kind {
            SetpointKind::Torque | SetpointKind::Position => true,
            SetpointKind::Velocity | SetpointKind::Force => false,
        }
    }

    /// The body-down unit vector the suspension compresses along: `-Z`, because
    /// [`Flu`]'s up is `+Z`. Named once here rather than inlined so the axis
    /// convention lives in a single, stated place.
    fn suspension_axis_flu() -> FluVector {
        FluVector::new(0.0, 0.0, -1.0)
    }

    /// One wheel's body-frame ground force, or `None` if it carries no load.
    ///
    /// Runs the per-wheel model (all body-frame, all instantaneous):
    ///
    /// - **Suspension → normal load.** Compression `x = (rest_length +
    ///   wheel_radius) − distance`, clamped to `[0, max_travel]`; a wheel that
    ///   hit within the ray's detection margin but is not actually compressed
    ///   (`x ≤ 0`) is airborne and returns `None`. The compression *rate* is the
    ///   mount's velocity along the suspension axis, `ẋ = −(v + ω × r)·up`, read
    ///   straight off the body twist so no spring-length state is needed. The
    ///   load is `N = max(0, k·x + c·ẋ)`; the non-negative clamp is mandatory —
    ///   a tyre can push the ground but never pull it, so a fast-rebounding
    ///   damper must not suck the chassis down. `N = 0` returns `None`.
    /// - **Longitudinal force.** `F_long = τ/wheel_radius − rolling`, where the
    ///   drive torque converts straight to a ground force (the friction circle,
    ///   not a wheel-spin ODE, is what caps it) and rolling resistance opposes
    ///   motion and scales with load.
    /// - **Lateral force.** `F_lat = −C_α·α` from the slip angle `α =
    ///   atan2(v_lateral, v_longitudinal)` in the wheel's steered frame, with
    ///   the velocity taken at the wheel (`v + ω × r`), not the centre of mass —
    ///   the yaw term is what makes a wheel corner. `C_α` is per axle. The force
    ///   fades to zero below [`low_speed_threshold`](TireParams) so the car sits
    ///   still instead of sliding sideways on phantom slip at a standstill.
    /// - **Friction circle.** Longitudinal and lateral compete for one grip
    ///   budget `μ·N`; if their combined magnitude exceeds it, both scale down to
    ///   the budget, preserving direction. This is where the tyre lets go.
    ///
    /// The force is assembled in body frame: `F_long` along the wheel heading,
    /// `F_lat` perpendicular, `N` up the suspension axis. `μ` arrives per contact
    /// (constant today, per-surface later); the normal `N` and tyre forces stay
    /// in the body horizontal/vertical basis rather than resolving against the
    /// true surface normal — a body-frame approximation accurate for moderate
    /// slopes.
    fn wheel_force(
        &self,
        wheel: &Wheel,
        contact: &WheelContact,
        twist: BodyTwist,
        drive_torque: f64,
        steer_angle: f64,
    ) -> Option<Vector3<f64>> {
        let rest_extension = self.suspension.rest_length + self.suspension.wheel_radius;
        let compression = rest_extension - contact.distance;
        if compression <= 0.0 {
            return None;
        }
        let compression = compression.min(self.suspension.max_travel);

        // Mount velocity carries the yaw contribution (v + ω × r); it feeds both
        // the compression rate and the tyre slip, so it is computed once.
        let mount_offset = *wheel.offset.raw();
        let mount_velocity =
            twist.linear().into_inner() + twist.angular().into_inner().cross(&mount_offset);

        // Compression rate: the mount velocity along body-up (+Z). A mount moving
        // down (−Z) compresses the spring, hence the leading minus.
        let compression_rate = -mount_velocity.z;
        let normal_load =
            (self.suspension.stiffness * compression + self.suspension.damping * compression_rate)
                .max(0.0);
        if normal_load <= 0.0 {
            return None;
        }

        // The steered wheel frame: a steer wheel points along `steer_angle`, a
        // fixed wheel straight ahead. Longitudinal is along the heading, lateral
        // perpendicular (left).
        let steer = if wheel.is_steer { steer_angle } else { 0.0 };
        let (sin, cos) = steer.sin_cos();
        let heading = Vector3::new(cos, sin, 0.0);
        let lateral = Vector3::new(-sin, cos, 0.0);

        let speed_long = mount_velocity.dot(&heading);
        let speed_lat = mount_velocity.dot(&lateral);
        // Horizontal speed in the wheel's own basis: heading and lateral are
        // orthonormal and span the ground plane, so this is the contact-patch
        // speed the slip regularization keys off.
        let speed = (speed_long * speed_long + speed_lat * speed_lat).sqrt();

        // Longitudinal: drive torque to force, minus load-scaled rolling
        // resistance directed against travel (and vanishing at a standstill via
        // the same saturation that fades lateral force).
        let drive = if wheel.is_drive { drive_torque } else { 0.0 };
        let rolling = self.tire.rolling_resistance * normal_load * self.speed_saturation(speed_long);
        let mut force_long = drive / self.suspension.wheel_radius - rolling;

        // Lateral: cornering stiffness times slip angle, faded to zero at crawl.
        let cornering_stiffness = match wheel.axle {
            Axle::Front => self.tire.cornering_stiffness_front,
            Axle::Rear => self.tire.cornering_stiffness_rear,
        };
        let slip_angle = speed_lat.atan2(speed_long);
        let mut force_lat = -cornering_stiffness * slip_angle * self.speed_saturation(speed);

        // Friction circle: one grip budget μ·N shared between the two, scaled
        // down together (direction preserved) when they exceed it.
        let budget = contact.mu * normal_load;
        let planar = (force_long * force_long + force_lat * force_lat).sqrt();
        if planar > budget && planar > 0.0 {
            let scale = budget / planar;
            force_long *= scale;
            force_lat *= scale;
        }

        Some(heading * force_long + lateral * force_lat + Vector3::new(0.0, 0.0, normal_load))
    }

    /// A signed, unit-bounded ramp of a speed through
    /// [`low_speed_threshold`](TireParams): `0` at rest, saturating to `±1` at
    /// the threshold. It regularises the two places a raw speed misbehaves near
    /// zero — it fades the lateral force so a standing car does not slide on the
    /// `atan2(0, 0)` phantom slip angle, and it directs rolling resistance so it
    /// vanishes at a standstill instead of a `signum(0)` shove. Passing a
    /// non-negative speed yields `[0, 1]`; a signed one yields `[-1, 1]`.
    fn speed_saturation(&self, speed: f64) -> f64 {
        (speed / self.tire.low_speed_threshold).clamp(-1.0, 1.0)
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
/// resistance, and the low-speed regularization threshold.
///
/// Cornering stiffness is per axle, not one global value: the front/rear ratio,
/// together with the weight distribution, is what sets the understeer/oversteer
/// balance, which a single number cannot express. The friction coefficient `μ`
/// is deliberately *not* here — it arrives per contact (see [`WheelContact`]),
/// so a constant today becomes per-surface materials later with no change to
/// the force computation.
pub struct TireParams {
    /// Front-axle cornering stiffness `C_αf` (N/rad): lateral force per radian
    /// of slip angle before the friction circle clips it.
    cornering_stiffness_front: f64,
    /// Rear-axle cornering stiffness `C_αr` (N/rad).
    cornering_stiffness_rear: f64,
    /// Rolling-resistance coefficient (dimensionless): the fraction of the
    /// normal load that opposes forward motion.
    rolling_resistance: f64,
    /// Speed (m/s) below which lateral force fades toward zero and rolling
    /// resistance ramps through zero — the regularization that keeps a standing
    /// car still instead of sliding on phantom slip. Must be strictly positive
    /// (the host validates it on load); it is a divisor in
    /// [`speed_saturation`](RaycastWheelPlant::speed_saturation).
    low_speed_threshold: f64,
}

impl TireParams {
    pub fn new(
        cornering_stiffness_front: f64,
        cornering_stiffness_rear: f64,
        rolling_resistance: f64,
        low_speed_threshold: f64,
    ) -> Self {
        Self {
            cornering_stiffness_front,
            cornering_stiffness_rear,
            rolling_resistance,
            low_speed_threshold,
        }
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

/// One wheel's ground contact, the host's answer to a [`WheelRay`]: how far
/// down the ray hit, and how much grip that surface offers.
///
/// The host builds one per wheel after casting the rays, aligned to the wheel
/// index, and hands them back to the force computation. A wheel whose ray found
/// no ground has no contact (`None` in the contact slice) and counts as
/// airborne. Both fields come from the world the host alone owns: `distance`
/// from the cast, `mu` from the surface it struck — the plant never queries the
/// scene, so grip is told to it rather than looked up.
pub struct WheelContact {
    /// Ray hit distance (m): how far along the downward suspension ray the
    /// ground was found. Suspension compression is `max_distance − distance`,
    /// so a shorter distance means a more compressed spring and a higher load.
    distance: f64,
    /// Coulomb friction coefficient at the contact patch, supplied by the host
    /// from the struck surface. Sets the friction-circle radius `mu · N` that
    /// bounds the combined tyre force.
    mu: f64,
}

impl WheelContact {
    pub fn new(distance: f64, mu: f64) -> Self {
        Self { distance, mu }
    }
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
        RaycastWheelPlant::new(wheels, suspension, force_tire())
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
            force_tire(),
        );

        assert!(bare.generate_rays().is_empty());
    }

    // --- Force computation (`compute_wrench`) ---

    use crate::control::actuators::{ActuatorId, ActuatorSetpoint};

    // Adaptive float compare: absolute near zero, relative for the large forces
    // a stiff spring and cornering stiffness produce.
    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-6 * a.abs().max(b.abs()).max(1.0)
    }

    // Asserts a body vector matches component-wise within `close`.
    fn vec_close(actual: FluVector, expected: FluVector) {
        assert!(
            close(actual.x(), expected.x())
                && close(actual.y(), expected.y())
                && close(actual.z(), expected.z()),
            "expected {expected:?}, got {actual:?}"
        );
    }

    // rest_length + wheel_radius = 0.70, so a contact at distance 0.60 (see
    // `contact`) compresses 0.10 m; at 50 kN/m and zero compression rate that is
    // a normal load of exactly N = 5000 N — the figure the force tests expect.
    fn force_suspension() -> SuspensionParams {
        SuspensionParams::new(0.40, 0.30, 50_000.0, 4_000.0, 0.20, 0.10)
    }

    // Distinct front/rear cornering stiffness so an axle mix-up is caught;
    // rolling resistance off so longitudinal tests isolate drive force; a 1 m/s
    // low-speed threshold, above which the lateral fade is fully open.
    fn force_tire() -> TireParams {
        TireParams::new(80_000.0, 60_000.0, 0.0, 1.0)
    }

    // A one-wheel plant mounted at the centre of mass: every force it makes has
    // a zero moment arm, so the returned wrench is pure force and a single
    // wheel's output reads straight off `compute_wrench`.
    fn one_wheel_at_origin(axle: Axle, is_drive: bool, is_steer: bool) -> RaycastWheelPlant {
        RaycastWheelPlant::new(
            vec![Wheel::new(Point::new(0.0, 0.0, 0.0), axle, is_drive, is_steer)],
            force_suspension(),
            force_tire(),
        )
    }

    // A contact 0.60 m down (0.10 m compression → N = 5000 N) at grip `mu`.
    fn contact(mu: f64) -> WheelContact {
        WheelContact::new(0.60, mu)
    }

    fn twist(vx: f64, vy: f64, vz: f64, wx: f64, wy: f64, wz: f64) -> BodyTwist {
        BodyTwist::new(FluVector::new(vx, vy, vz), FluVector::new(wx, wy, wz))
    }

    fn drive_cmd(torque: f64) -> ActuatorCommand {
        ActuatorCommand::new(vec![ActuatorSetpoint::new(
            ActuatorId::new("drive"),
            SetpointValue::Torque(torque),
        )])
    }

    fn drive_and_steer_cmd(torque: f64, steer: f64) -> ActuatorCommand {
        ActuatorCommand::new(vec![
            ActuatorSetpoint::new(ActuatorId::new("drive"), SetpointValue::Torque(torque)),
            ActuatorSetpoint::new(ActuatorId::new("steer"), SetpointValue::Position(steer)),
        ])
    }

    #[test]
    fn airborne_wheel_contributes_no_force() {
        // No contact (the ray found no ground) means no force, even under a live
        // drive command — an airborne wheel cannot push on anything. A valid
        // command kind is still not "unsupported": the wheel is simply off the
        // ground.
        let plant = one_wheel_at_origin(Axle::Rear, true, false);

        let out = plant.compute_wrench(&[None], BodyTwist::zero(), &drive_cmd(300.0));

        assert_eq!(out.wrench(), BodyWrench::zero());
        assert!(out.unsupported().is_empty());
    }

    #[test]
    fn resting_wheel_carries_only_normal_load() {
        // A wheel in contact with no command and no motion holds the chassis up
        // and nothing else: force is pure normal load, and because the mount is
        // off-centre that load makes a moment r × F about the centre of mass.
        let plant = RaycastWheelPlant::new(
            vec![Wheel::new(Point::new(-1.4, 0.8, -0.3), Axle::Rear, false, false)],
            force_suspension(),
            force_tire(),
        );

        let out = plant.compute_wrench(
            &[Some(contact(1.0))],
            BodyTwist::zero(),
            &ActuatorCommand::new(vec![]),
        );

        vec_close(out.wrench().force(), FluVector::new(0.0, 0.0, 5000.0));
        // (-1.4, 0.8, -0.3) × (0, 0, 5000) = (4000, 7000, 0).
        vec_close(out.wrench().torque(), FluVector::new(4000.0, 7000.0, 0.0));
    }

    #[test]
    fn drive_torque_becomes_forward_force() {
        // Drive torque converts to a forward ground force F = τ / wheel_radius =
        // 300 / 0.30 = 1000 N, well inside the grip budget (μN = 5000), so it
        // passes through uncapped alongside the normal load.
        let plant = one_wheel_at_origin(Axle::Rear, true, false);

        let out = plant.compute_wrench(&[Some(contact(1.0))], BodyTwist::zero(), &drive_cmd(300.0));

        vec_close(out.wrench().force(), FluVector::new(1000.0, 0.0, 5000.0));
        vec_close(out.wrench().torque(), FluVector::zeros());
        assert!(out.unsupported().is_empty());
    }

    #[test]
    fn lateral_force_opposes_slip_angle() {
        // Sliding forward-and-left (v = (2, 2)) is a slip angle of atan2(2, 2) =
        // π/4; the tyre answers with a lateral force −C_αf·α that points right
        // (−y) to resist the slide. Grip is set high so the friction circle does
        // not clip it and the linear law shows through.
        let plant = one_wheel_at_origin(Axle::Front, false, false);

        let out = plant.compute_wrench(
            &[Some(contact(20.0))],
            twist(2.0, 2.0, 0.0, 0.0, 0.0, 0.0),
            &ActuatorCommand::new(vec![]),
        );

        let expected_lat = -80_000.0 * std::f64::consts::FRAC_PI_4;
        vec_close(out.wrench().force(), FluVector::new(0.0, expected_lat, 5000.0));
        assert!(
            out.wrench().force().y() < 0.0,
            "a leftward slide must produce a rightward (−y) restoring force"
        );
    }

    #[test]
    fn slip_angle_uses_wheel_offset_not_cg() {
        // Pure forward motion at the centre of mass, but a yaw rate: the wheel
        // sits 2 m ahead, so ω × r gives it 2 m/s of lateral velocity the centre
        // of mass does not have. The lateral force is therefore nonzero — a slip
        // computed from centre-of-mass velocity alone would be exactly zero here.
        let plant = RaycastWheelPlant::new(
            vec![Wheel::new(Point::new(2.0, 0.0, 0.0), Axle::Front, false, false)],
            force_suspension(),
            force_tire(),
        );

        let out = plant.compute_wrench(
            &[Some(contact(20.0))],
            twist(10.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            &ActuatorCommand::new(vec![]),
        );

        let expected_lat = -80_000.0 * (2.0_f64).atan2(10.0);
        vec_close(out.wrench().force(), FluVector::new(0.0, expected_lat, 5000.0));
        assert!(
            out.wrench().force().y().abs() > 1.0,
            "yaw-induced lateral force must be nonzero; a centre-of-mass slip would give 0"
        );
    }

    #[test]
    fn friction_circle_saturates() {
        // Drive force and a large slip together demand more than the grip budget
        // μN = 0.5 · 5000 = 2500 N. The friction circle scales both down so the
        // planar force magnitude is exactly the budget, in the same direction —
        // this is the tyre letting go. The normal load is outside the circle and
        // is untouched.
        let plant = one_wheel_at_origin(Axle::Front, true, false);

        let out = plant.compute_wrench(
            &[Some(contact(0.5))],
            twist(2.0, 2.0, 0.0, 0.0, 0.0, 0.0),
            &drive_cmd(300.0),
        );

        let f = out.wrench().force();
        let planar = (f.x() * f.x() + f.y() * f.y()).sqrt();
        assert!(close(planar, 2500.0), "planar force should clamp to μN = 2500, got {planar}");
        assert!(close(f.z(), 5000.0), "normal load is outside the circle and unchanged");
        // Direction preserved: the clamp scales both components equally, so the
        // longitudinal:lateral ratio matches the unclamped forces.
        let raw_long = 1000.0;
        let raw_lat = -80_000.0 * std::f64::consts::FRAC_PI_4;
        assert!(close(f.x() / f.y(), raw_long / raw_lat));
    }

    #[test]
    fn steer_rotates_the_force_basis() {
        // Steering does not add force; it turns the tyre's force basis. With the
        // same drive torque, a straight wheel pushes along +x, and a wheel
        // steered 90° pushes the identical magnitude along +y. Zero motion keeps
        // it a clean drive-only force with no lateral term.
        let steered = one_wheel_at_origin(Axle::Rear, true, true);

        let straight =
            steered.compute_wrench(&[Some(contact(1.0))], BodyTwist::zero(), &drive_cmd(300.0));
        vec_close(straight.wrench().force(), FluVector::new(1000.0, 0.0, 5000.0));

        let turned = steered.compute_wrench(
            &[Some(contact(1.0))],
            BodyTwist::zero(),
            &drive_and_steer_cmd(300.0, std::f64::consts::FRAC_PI_2),
        );
        vec_close(turned.wrench().force(), FluVector::new(0.0, 1000.0, 5000.0));

        // A fixed (non-steer) wheel ignores the steer angle entirely.
        let fixed = one_wheel_at_origin(Axle::Rear, true, false);
        let out = fixed.compute_wrench(
            &[Some(contact(1.0))],
            BodyTwist::zero(),
            &drive_and_steer_cmd(300.0, std::f64::consts::FRAC_PI_2),
        );
        vec_close(out.wrench().force(), FluVector::new(1000.0, 0.0, 5000.0));
    }

    #[test]
    fn wrong_kind_setpoint_is_unsupported() {
        // The L1 plant drives from torque, so a Velocity setpoint has no place in
        // the model: it is reported unsupported and adds no longitudinal force,
        // leaving only the normal load. (Inverse of the L0 shim, which folds
        // Velocity and rejects Torque.)
        let plant = one_wheel_at_origin(Axle::Rear, true, false);

        let out = plant.compute_wrench(
            &[Some(contact(1.0))],
            BodyTwist::zero(),
            &ActuatorCommand::new(vec![ActuatorSetpoint::new(
                ActuatorId::new("drive"),
                SetpointValue::Velocity(5.0),
            )]),
        );

        assert_eq!(out.unsupported(), &[ActuatorId::new("drive")]);
        vec_close(out.wrench().force(), FluVector::new(0.0, 0.0, 5000.0));
    }

    #[test]
    fn accepts_agrees_with_compute_wrench() {
        // The spawn-time guard (`accepts`) and the per-tick fold must classify a
        // command space the same way, or a kind the guard passed would still be
        // warned-and-ignored at runtime. Fold a lone setpoint of each kind and
        // check `accepts` predicts exactly whether it lands (empty unsupported).
        let plant = one_wheel_at_origin(Axle::Rear, true, true);
        for kind in [
            SetpointKind::Force,
            SetpointKind::Torque,
            SetpointKind::Position,
            SetpointKind::Velocity,
        ] {
            let out = plant.compute_wrench(
                &[Some(contact(1.0))],
                BodyTwist::zero(),
                &ActuatorCommand::new(vec![ActuatorSetpoint::new(
                    ActuatorId::new("a"),
                    kind.value(1.0),
                )]),
            );
            assert_eq!(
                plant.accepts(kind),
                out.unsupported().is_empty(),
                "accepts and compute_wrench disagree for {kind:?}"
            );
        }
    }
}
