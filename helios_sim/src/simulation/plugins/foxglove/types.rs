// helios_sim/src/simulation/plugins/foxglove/types.rs
//
// Converts internal helios types into flat, Foxglove-plottable JSON values.
// Every field becomes a top-level scalar so Foxglove's Plot and Raw Message
// panels can directly address them (e.g. `pos_x`, `vel_y`, `acc_z`).
//
// None of this touches the core types — it is presentation-only.

use helios_core::frames::{FrameAwareState, StateVariable};
use helios_core::messages::{MeasurementData, MeasurementMessage /*, PointCloud*/};
use serde_json::{json, Map, Value};

use crate::simulation::core::components::GroundTruthState;

// ---------------------------------------------------------------------------
// GroundTruthState
// ---------------------------------------------------------------------------

pub fn ground_truth_to_json(gt: &GroundTruthState) -> Value {
    let t = &gt.pose.translation.vector;
    let q = gt.pose.rotation.quaternion();
    json!({
        "pos_x": t.x, "pos_y": t.y, "pos_z": t.z,
        "quat_x": q.i, "quat_y": q.j, "quat_z": q.k, "quat_w": q.w,
        "vel_x":  gt.linear_velocity.x,
        "vel_y":  gt.linear_velocity.y,
        "vel_z":  gt.linear_velocity.z,
        "ang_vel_x": gt.angular_velocity.x,
        "ang_vel_y": gt.angular_velocity.y,
        "ang_vel_z": gt.angular_velocity.z,
        "acc_x":  gt.linear_acceleration.x,
        "acc_y":  gt.linear_acceleration.y,
        "acc_z":  gt.linear_acceleration.z,
    })
}

// ---------------------------------------------------------------------------
// FrameAwareState  (estimator output published as odometry/estimated)
// ---------------------------------------------------------------------------

/// Maps StateVariable variants to short, readable JSON key names.
fn state_var_key(var: &StateVariable) -> &'static str {
    match var {
        StateVariable::Px(_) => "pos_x",
        StateVariable::Py(_) => "pos_y",
        StateVariable::Pz(_) => "pos_z",
        StateVariable::Vx(_) => "vel_x",
        StateVariable::Vy(_) => "vel_y",
        StateVariable::Vz(_) => "vel_z",
        StateVariable::Ax(_) => "acc_x",
        StateVariable::Ay(_) => "acc_y",
        StateVariable::Az(_) => "acc_z",
        StateVariable::Qx(_, _) => "quat_x",
        StateVariable::Qy(_, _) => "quat_y",
        StateVariable::Qz(_, _) => "quat_z",
        StateVariable::Qw(_, _) => "quat_w",
        StateVariable::Wx(_) => "ang_vel_x",
        StateVariable::Wy(_) => "ang_vel_y",
        StateVariable::Wz(_) => "ang_vel_z",
        StateVariable::Alphax(_) => "ang_acc_x",
        StateVariable::Alphay(_) => "ang_acc_y",
        StateVariable::Alphaz(_) => "ang_acc_z",
        StateVariable::MagX(_) => "mag_x",
        StateVariable::MagY(_) => "mag_y",
        StateVariable::MagZ(_) => "mag_z",
    }
}

pub fn frame_aware_state_to_json(state: &FrameAwareState) -> Value {
    let mut map = Map::new();
    map.insert("timestamp".to_string(), state.last_update_timestamp.into());
    for (i, var) in state.layout.iter().enumerate() {
        let key = state_var_key(var).to_string();
        // If two layout variables map to the same key (shouldn't happen in
        // practice), append the index to avoid silent overwrites.
        let key = if map.contains_key(&key) {
            format!("{key}_{i}")
        } else {
            key
        };
        map.insert(key, state.vector[i].into());
    }
    Value::Object(map)
}

// ---------------------------------------------------------------------------
// MeasurementMessage
// ---------------------------------------------------------------------------

pub fn measurement_to_json(msg: &MeasurementMessage) -> Value {
    let data = match &msg.data {
        MeasurementData::Imu6Dof(v) => json!({
            "type": "imu_6dof",
            "acc_x": v[0], "acc_y": v[1], "acc_z": v[2],
            "gyro_x": v[3], "gyro_y": v[4], "gyro_z": v[5],
        }),
        MeasurementData::Imu9Dof { accel_gyro, mag } => json!({
            "type": "imu_9dof",
            "acc_x": accel_gyro[0], "acc_y": accel_gyro[1], "acc_z": accel_gyro[2],
            "gyro_x": accel_gyro[3], "gyro_y": accel_gyro[4], "gyro_z": accel_gyro[5],
            "mag_x": mag[0], "mag_y": mag[1], "mag_z": mag[2],
        }),
        MeasurementData::GpsPosition(v) => json!({
            "type": "gps",
            "pos_x": v[0], "pos_y": v[1], "pos_z": v[2],
        }),
        MeasurementData::Magnetometer(v) => json!({
            "type": "magnetometer",
            "mag_x": v[0], "mag_y": v[1], "mag_z": v[2],
        }),
        MeasurementData::PointCloud(pc) => json!({
            "type": "point_cloud",
            "num_points": pc.points.len(),
        }),
    };

    let mut out = match data {
        Value::Object(m) => m,
        _ => Map::new(),
    };
    out.insert("timestamp".to_string(), msg.timestamp.into());
    Value::Object(out)
}

// PointCloud serializer commented out — no Foxglove 3D panel; belongs in MCAP logger when implemented.
// pub fn point_cloud_to_json(pc: &PointCloud) -> Value { ... }
