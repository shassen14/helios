//! Canonical oracle [`ChannelKey`] constructors for host-published channels.
//!
//! Producers (sim host, future hw host) and consumers (mocks, debug viz)
//! agree on both the instance string and the Rust type binding each slot
//! by calling the *same* function here. That makes the kind+name+type
//! tuple a single source of truth — neither side can drift without a
//! compile-time type error.
//!
//! Functions (not consts) are required because a channel carries a
//! [`std::any::TypeId`], which is not `const`-constructible. The call is
//! a couple of cheap stack moves and is not a hot-path concern.

use crate::port::ChannelKey;

use helios_core::prelude::Twist;
use nalgebra::Isometry3;

/// Oracle channel carrying the agent body's world pose at the current tick.
///
/// Payload: [`Isometry3<f64>`] expressed in **world ENU**. Translation is
/// the body origin's position; rotation is body→world.
///
/// Published by the sim host's `publish_oracle_channels_system`. Only mock
/// nodes may declare this as an input; the pipeline build refuses an
/// algorithm node that does.
pub fn oracle_pose_channel() -> ChannelKey {
    ChannelKey::oracle::<Isometry3<f64>>("oracle.pose")
}

/// Oracle channel carrying the agent body's twist at the current tick.
///
/// Payload: [`Twist`] expressed in **world ENU**. `linear` is the body
/// origin's translational velocity in world; `angular` is the body's
/// angular velocity expressed in world. The estimate's odom frame is
/// ENU-aligned to world, so passthrough mocks write these straight into the
/// odom-frame velocity blocks of a `FrameAwareState` without rotating.
///
/// Published by the sim host's `publish_oracle_channels_system`. Only mock
/// nodes may declare this as an input.
pub fn oracle_twist_channel() -> ChannelKey {
    ChannelKey::oracle::<Twist>("oracle.twist")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::ChannelKind;

    #[test]
    fn oracle_pose_channel_is_kind_oracle() {
        let key = oracle_pose_channel();
        assert_eq!(key.kind(), ChannelKind::Oracle);
    }

    #[test]
    fn oracle_twist_channel_is_kind_oracle() {
        let key = oracle_twist_channel();
        assert_eq!(key.kind(), ChannelKind::Oracle);
    }

    #[test]
    fn oracle_pose_and_twist_are_distinct() {
        let pose = oracle_pose_channel();
        let twist = oracle_twist_channel();
        assert_ne!(pose, twist);
    }

    #[test]
    fn repeated_calls_return_equal_keys() {
        assert_eq!(oracle_pose_channel(), oracle_pose_channel());
        assert_eq!(oracle_twist_channel(), oracle_twist_channel());
    }
}
