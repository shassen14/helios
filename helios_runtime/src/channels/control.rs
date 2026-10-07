//! Control-channel *role* constructors.
//!
//! Unlike the *pinned* oracle channels (a fixed name bound to one fixed type
//! forever), a *command* role fixes the *name* but leaves the *type* to the
//! morphology: "the command the allocator reads" is `BodyTwist` for a ground
//! vehicle and `Wrench` for a free-flyer. So [`command`] is generic over `T`:
//! it binds a reserved role string to whatever payload type the caller's
//! morphology uses.
//!
//! The *reference* role ([`reference`]) is the same shape one layer up, at the
//! guidance→tracking seam: the instantaneous setpoint the controllers track,
//! `BodyTwistRef` for a decoupled car and an `AttitudeThrustRef` for a
//! multirotor. Teleop-vs-autonomy is arbitrated *here*, on the single-signal
//! reference, not on the fanned-out command spaces below it. The contenders
//! (a path follower, a teleop mapper) have no role: each writes a channel named
//! after its node, and the reference seam's `Selector` resolves the winner onto
//! [`reference`], which the controllers read.
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

const ROLE_COMMAND: &str = "command";
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

/// The command the downstream consumer reads.
///
/// Plural role (Internal). Written by the command-space `Sum` fold (or a lone
/// controller), read by the allocator, which converts it into the
/// [`actuators`] terminal.
pub fn command<T>() -> InternalChannel
where
    T: 'static,
{
    InternalChannel::named::<T>(ROLE_COMMAND)
}

/// The per-actuator terminal: the pipeline's final control output.
///
/// Singular fixed type (Internal). Written by the allocator, read by the host
/// relay (and by [`AutonomyPipeline::read_actuators`]). Unlike the command
/// roles above, this is not generic — the seam pins it to [`ActuatorCommand`]
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

    struct BodyTwist;
    struct Wrench;

    #[test]
    fn roles_are_internal_kind() {
        let key: ChannelKey = command::<BodyTwist>().into();
        assert_eq!(key.kind(), ChannelKind::Internal);
        let terminal: ChannelKey = actuators().into();
        assert_eq!(terminal.kind(), ChannelKind::Internal);
    }

    #[test]
    fn actuator_terminal_is_its_own_role() {
        // The terminal must not share the `command` role: it carries its own
        // reserved name, so it stays distinct even from a same-typed `command`.
        // (Regression guard against the terminal being wired to ROLE_COMMAND.)
        assert_ne!(
            actuators(),
            command::<helios_core::control::actuators::ActuatorCommand>()
        );
    }

    #[test]
    fn distinct_roles_are_distinct_channels() {
        // Intent is the host's pre-scaling ingress, never a seam role: it must not
        // collide with the reference or the command, even when they carry the
        // same payload type.
        assert_ne!(intent::<BodyTwist>(), reference::<BodyTwist>());
        assert_ne!(intent::<BodyTwist>(), command::<BodyTwist>());
    }

    #[test]
    fn reference_is_distinct_from_command() {
        // The reference seam sits one layer up from the command seam. Its
        // resolved slot must not alias a command-seam role even on the same
        // payload type, or the two seams would fight over one slot.
        assert_ne!(reference::<BodyTwist>(), command::<BodyTwist>());
    }

    #[test]
    fn same_role_same_type_is_equal() {
        assert_eq!(command::<BodyTwist>(), command::<BodyTwist>());
    }

    #[test]
    fn same_role_different_type_is_distinct() {
        // A role fixes the name but not the payload type: a body-twist command
        // and a wrench command are different slots on the same role string.
        assert_ne!(command::<BodyTwist>(), command::<Wrench>());
    }
}
