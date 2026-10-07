//! Control-channel *role* constructors.
//!
//! Unlike the *pinned* oracle channels (a fixed name bound to one fixed type
//! forever), a role here fixes the *name* but leaves the *type* to the
//! morphology. The *reference* role ([`reference`]) is the guidance→tracking
//! seam: the instantaneous setpoint the controllers track, `BodyTwistRef` for
//! a decoupled car and an `AttitudeThrustRef` for a multirotor. So it is
//! generic over `T`: it binds a reserved role string to whatever payload type
//! the caller's morphology uses. Teleop-vs-autonomy is arbitrated *here*, on
//! the single-signal reference. The contenders (a path follower, a teleop
//! mapper) have no role: each writes a channel named after its node, and the
//! reference seam's `Selector` resolves the winner onto [`reference`], which
//! the controllers read.
//!
//! Commands have no role. A stack may fold several commands, even several of
//! one type, so each `[command]` fold writes a channel named after itself and
//! each allocator names the fold it reads.
//!
//! The *actuator terminal* ([`actuators`]) is the exception: the whole point of
//! the actuator seam is that everything downstream of the allocator speaks one
//! universal type, [`ActuatorCommand`], regardless of morphology. So it is not
//! generic — the role and the type are both fixed.
//!
//! All roles are `Internal` kind: brain-produced, or host-injected intent the
//! brain then arbitrates — never sensor or oracle data. Each returns the kinded
//! [`InternalChannel`] so it drops straight into the descriptor builders
//! (`input_internal` / `output_internal`); call `.into()` for a
//! [`ChannelKey`](crate::ChannelKey) when reading the bus.
//!
//! The role strings live as consts so a producer and a consumer can't drift on
//! a typo.

use helios_core::control::actuators::ActuatorCommand;

use crate::port::InternalChannel;

const ROLE_ACTUATORS: &str = "actuators";
const ROLE_INTENT: &str = "intent";
const ROLE_REFERENCE: &str = "reference";

/// The resolved guidance reference the tracking layer consumes.
///
/// Plural role (Internal). Written by the reference seam's `Selector`, read by
/// every controller's input builder.
/// This is the single-signal seam: one instantaneous setpoint below all
/// planning, the same slot whether autonomy or teleop currently owns it.
pub fn reference<T>() -> InternalChannel
where
    T: 'static,
{
    InternalChannel::named::<T>(ROLE_REFERENCE)
}

/// The per-actuator terminal: the pipeline's final control output.
///
/// Singular fixed type (Internal). Written by the allocator, read by the host
/// relay (and by [`AutonomyPipeline::read_actuators`]). Unlike the other
/// roles here, this is not generic — the seam pins it to [`ActuatorCommand`]
/// so every host consumes one universal command type.
///
/// [`AutonomyPipeline::read_actuators`]: crate::pipeline::AutonomyPipeline::read_actuators
pub fn actuators() -> InternalChannel {
    InternalChannel::named::<ActuatorCommand>(ROLE_ACTUATORS)
}

/// The operator's raw motion intent, before scaling or framing.
///
/// Plural role (Internal). Written by the host as dimensionless per-axis
/// deflection, read by the teleop mapper node, which scales it into a guidance
/// reference. Generic for the same reason the other roles are: the role fixes
/// the name, the payload follows the morphology — `TwistIntent` for a velocity
/// body, a future `SurfaceIntent` for a plane — never the reference type
/// itself, so it stays distinct from [`reference`] on the same payload.
pub fn intent<T: 'static>() -> InternalChannel {
    InternalChannel::named::<T>(ROLE_INTENT)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::{ChannelKey, ChannelKind};

    struct BodyTwistRef;
    struct AttitudeThrustRef;

    #[test]
    fn roles_are_internal_kind() {
        let key: ChannelKey = reference::<BodyTwistRef>().into();
        assert_eq!(key.kind(), ChannelKind::Internal);
        let terminal: ChannelKey = actuators().into();
        assert_eq!(terminal.kind(), ChannelKind::Internal);
    }

    #[test]
    fn actuator_terminal_is_its_own_role() {
        // The terminal carries its own reserved name, so it stays distinct even
        // from a same-typed reference.
        assert_ne!(
            actuators(),
            reference::<helios_core::control::actuators::ActuatorCommand>()
        );
    }

    #[test]
    fn intent_is_distinct_from_the_reference() {
        // Intent is the host's pre-scaling ingress, never a seam role: it must
        // not collide with the reference, even on the same payload type.
        assert_ne!(intent::<BodyTwistRef>(), reference::<BodyTwistRef>());
    }

    #[test]
    fn same_role_same_type_is_equal() {
        assert_eq!(reference::<BodyTwistRef>(), reference::<BodyTwistRef>());
    }

    #[test]
    fn same_role_different_type_is_distinct() {
        // A role fixes the name but not the payload type: a twist reference and
        // an attitude-thrust reference are different slots on one role string.
        assert_ne!(
            reference::<BodyTwistRef>(),
            reference::<AttitudeThrustRef>()
        );
    }
}
