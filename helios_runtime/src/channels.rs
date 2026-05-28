//! Canonical [`ChannelKey`] constructors for host-published channels.
//!
//! Producers (sim host, future hw host) and consumers (mocks, debug viz)
//! agree on both the instance string and the Rust type binding each slot
//! by calling the *same* function here. That makes the kind+name+type
//! tuple a single source of truth — neither side can drift without a
//! compile-time type error.
//!
//! Functions (not consts) are required because [`ChannelKey`] carries a
//! [`std::any::TypeId`], which is not `const`-constructible. The call is
//! a couple of cheap stack moves and is not a hot-path concern.

use crate::{port::OracleChannel, ChannelKey};

use helios_core::data::Twist;
use nalgebra::Isometry3;

/// Oracle channel carrying the agent body's world pose at the current tick.
///
/// Payload: [`Isometry3<f64>`] expressed in **world ENU**. Translation is
/// the body origin's position; rotation is body→world.
///
/// Published by the sim host's `publish_oracle_channels_system`. Only mock
/// nodes (`MockNodePortDescriptor`) may declare this as an input — the type
/// system forbids algorithm nodes from doing so.
pub fn oracle_pose_channel() -> ChannelKey {
    OracleChannel::named::<Isometry3<f64>>("oracle/pose").into()
}

/// Oracle channel carrying the agent body's twist at the current tick.
///
/// Payload: [`Twist`] expressed in **body FLU**. `linear` is the body
/// origin's translational velocity, and `angular` is its angular velocity,
/// both rotated into the body frame at the publisher. This matches the
/// convention real hardware odometry / IMU stacks deliver, so mocks
/// written against this channel transfer to hw without a frame change.
///
/// Published by the sim host's `publish_oracle_channels_system`. Only mock
/// nodes may declare this as an input.
pub fn oracle_twist_channel() -> ChannelKey {
    OracleChannel::named::<Twist>("oracle/twist").into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::ChannelKind;

    #[test]
    fn oracle_pose_channel_is_kind_oracle() {
        assert_eq!(oracle_pose_channel().kind(), ChannelKind::Oracle);
    }

    #[test]
    fn oracle_twist_channel_is_kind_oracle() {
        assert_eq!(oracle_twist_channel().kind(), ChannelKind::Oracle);
    }

    #[test]
    fn oracle_pose_and_twist_are_distinct() {
        assert_ne!(oracle_pose_channel(), oracle_twist_channel());
    }

    #[test]
    fn repeated_calls_return_equal_keys() {
        assert_eq!(oracle_pose_channel(), oracle_pose_channel());
        assert_eq!(oracle_twist_channel(), oracle_twist_channel());
    }
}
