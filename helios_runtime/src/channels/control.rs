//! Control-channel *role* constructors.
//!
//! Unlike the *pinned* oracle channels (a fixed name bound to one fixed type
//! forever), a control role fixes the *name* but leaves the *type* to the
//! morphology: "the command the arbiter reads" is `BodyTwist` for a ground
//! vehicle and, after the actuator seam, `Wrench` for others. So every
//! constructor here is generic over `T` — it binds a reserved role string to
//! whatever payload type the caller's morphology uses.
//!
//! All roles are `Internal` kind: brain-produced, or host-injected intent the
//! brain then arbitrates — never sensor or oracle data. Each returns the kinded
//! [`InternalChannel`] so it drops straight into the descriptor builders
//! (`input_internal` / `output_internal`); call `.into()` for a
//! [`ChannelKey`](crate::ChannelKey) when reading the bus.
//!
//! The role strings live as consts so a producer and a consumer can't drift on
//! a typo.

use crate::port::InternalChannel;

const ROLE_AUTONOMY: &str = "autonomy";
const ROLE_TELEOP: &str = "teleop";
const ROLE_COMMAND: &str = "command";

/// The autonomy stack's command.
///
/// Plural role (Internal). Written by the controller node, read by the arbiter.
pub fn autonomy<T>() -> InternalChannel
where
    T: 'static,
{
    InternalChannel::named::<T>(ROLE_AUTONOMY)
}

/// The human operator's command.
///
/// Plural role (Internal). Written by the host (keyboard / gamepad), read by
/// the arbiter.
pub fn teleop<T>() -> InternalChannel
where
    T: 'static,
{
    InternalChannel::named::<T>(ROLE_TELEOP)
}

/// The command the downstream consumer reads.
///
/// Plural role (Internal). Written by the arbiter (or a lone command source),
/// read by the arbiter's consumer — the read surface today, the allocator once
/// the actuator seam lands.
pub fn command<T>() -> InternalChannel
where
    T: 'static,
{
    InternalChannel::named::<T>(ROLE_COMMAND)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::{ChannelKey, ChannelKind};

    struct BodyTwist;
    struct Wrench;

    #[test]
    fn roles_are_internal_kind() {
        let key: ChannelKey = autonomy::<BodyTwist>().into();
        assert_eq!(key.kind(), ChannelKind::Internal);
    }

    #[test]
    fn distinct_roles_are_distinct_channels() {
        assert_ne!(autonomy::<BodyTwist>(), teleop::<BodyTwist>());
        assert_ne!(teleop::<BodyTwist>(), command::<BodyTwist>());
        assert_ne!(autonomy::<BodyTwist>(), command::<BodyTwist>());
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
