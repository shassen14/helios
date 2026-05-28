use crate::port::OracleChannel;

use helios_core::data::Twist;
use nalgebra::Isometry3;

pub fn oracle_pose_channel() -> OracleChannel {
    OracleChannel::named::<Isometry3<f64>>("oracle/pose")
}

pub fn oracle_twist_channel() -> OracleChannel {
    OracleChannel::named::<Twist>("oracle/twist")
}
