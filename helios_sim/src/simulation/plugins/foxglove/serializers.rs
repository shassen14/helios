// helios_sim/src/simulation/plugins/foxglove/serializers.rs
//
// JSON Schema strings matching the flat output produced by foxglove/types.rs.
// These are sent to Foxglove Studio in the `advertise` message so it knows
// every field type — enabling plots and Raw Messages panels.

pub static MEASUREMENT_MESSAGE_SCHEMA: &str = r#"{
  "type": "object",
  "title": "MeasurementMessage",
  "properties": {
    "timestamp":  { "type": "number" },
    "type":       { "type": "string" },
    "acc_x":      { "type": "number" },
    "acc_y":      { "type": "number" },
    "acc_z":      { "type": "number" },
    "gyro_x":     { "type": "number" },
    "gyro_y":     { "type": "number" },
    "gyro_z":     { "type": "number" },
    "mag_x":      { "type": "number" },
    "mag_y":      { "type": "number" },
    "mag_z":      { "type": "number" },
    "pos_x":      { "type": "number" },
    "pos_y":      { "type": "number" },
    "pos_z":      { "type": "number" },
    "num_points": { "type": "integer" }
  }
}"#;

pub static POINT_CLOUD_SCHEMA: &str = r#"{
  "type": "object",
  "title": "PointCloud",
  "properties": {
    "timestamp":  { "type": "number" },
    "num_points": { "type": "integer" },
    "points": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "x":         { "type": "number" },
          "y":         { "type": "number" },
          "z":         { "type": "number" },
          "intensity": { "type": ["number", "null"] }
        }
      }
    }
  }
}"#;

pub static GROUND_TRUTH_SCHEMA: &str = r#"{
  "type": "object",
  "title": "GroundTruthState",
  "properties": {
    "pos_x":     { "type": "number", "description": "ENU position X (m)" },
    "pos_y":     { "type": "number", "description": "ENU position Y (m)" },
    "pos_z":     { "type": "number", "description": "ENU position Z (m)" },
    "quat_x":    { "type": "number" },
    "quat_y":    { "type": "number" },
    "quat_z":    { "type": "number" },
    "quat_w":    { "type": "number" },
    "vel_x":     { "type": "number", "description": "ENU velocity X (m/s)" },
    "vel_y":     { "type": "number", "description": "ENU velocity Y (m/s)" },
    "vel_z":     { "type": "number", "description": "ENU velocity Z (m/s)" },
    "ang_vel_x": { "type": "number", "description": "Angular velocity X (rad/s)" },
    "ang_vel_y": { "type": "number" },
    "ang_vel_z": { "type": "number" },
    "acc_x":     { "type": "number", "description": "Linear acceleration X (m/s²)" },
    "acc_y":     { "type": "number" },
    "acc_z":     { "type": "number" }
  }
}"#;

pub static FRAME_AWARE_STATE_SCHEMA: &str = r#"{
  "type": "object",
  "title": "FrameAwareState",
  "description": "Estimator state — keys are named after StateVariable layout (pos_x, vel_y, …)",
  "properties": {
    "timestamp": { "type": "number" },
    "pos_x":     { "type": "number" },
    "pos_y":     { "type": "number" },
    "pos_z":     { "type": "number" },
    "vel_x":     { "type": "number" },
    "vel_y":     { "type": "number" },
    "vel_z":     { "type": "number" },
    "acc_x":     { "type": "number" },
    "acc_y":     { "type": "number" },
    "acc_z":     { "type": "number" },
    "quat_x":    { "type": "number" },
    "quat_y":    { "type": "number" },
    "quat_z":    { "type": "number" },
    "quat_w":    { "type": "number" },
    "ang_vel_x": { "type": "number" },
    "ang_vel_y": { "type": "number" },
    "ang_vel_z": { "type": "number" }
  },
  "additionalProperties": { "type": "number" }
}"#;
