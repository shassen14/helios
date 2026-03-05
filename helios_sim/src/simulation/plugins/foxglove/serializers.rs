// helios_sim/src/simulation/plugins/foxglove/serializers.rs
//
// JSON Schema strings for each type published through the Foxglove bridge.
// Schemas are intentionally minimal — enough for Foxglove to display fields.

pub static MEASUREMENT_MESSAGE_SCHEMA: &str = r#"{
  "type": "object",
  "title": "MeasurementMessage",
  "properties": {
    "agent_handle": { "type": "integer", "description": "Agent entity ID" },
    "sensor_handle": { "type": "integer", "description": "Sensor entity ID" },
    "timestamp": { "type": "number", "description": "Seconds since sim start" },
    "data": { "type": "object", "description": "Sensor-specific measurement payload" }
  }
}"#;

pub static POINT_CLOUD_SCHEMA: &str = r#"{
  "type": "object",
  "title": "PointCloud",
  "properties": {
    "sensor_handle": { "type": "integer" },
    "timestamp": { "type": "number" },
    "points": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "position": {
            "type": "object",
            "description": "nalgebra Point3 — coords.data contains [[x,y,z]]"
          },
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
    "pose": { "type": "object", "description": "Isometry3 (translation + rotation)" },
    "linear_velocity": { "type": "object", "description": "ENU m/s" },
    "angular_velocity": { "type": "object", "description": "ENU rad/s" },
    "linear_acceleration": { "type": "object", "description": "ENU m/s²" },
    "last_linear_velocity": { "type": "object" }
  }
}"#;

pub static FRAME_AWARE_STATE_SCHEMA: &str = r#"{
  "type": "object",
  "title": "FrameAwareState",
  "properties": {
    "layout": {
      "type": "array",
      "items": { "type": "object", "description": "StateVariable enum variant" }
    },
    "vector": { "type": "object", "description": "DVector state values" },
    "covariance": { "type": "object", "description": "DMatrix covariance" },
    "last_update_timestamp": { "type": "number" }
  }
}"#;

pub static ODOMETRY_SCHEMA: &str = r#"{
  "type": "object",
  "title": "Odometry",
  "properties": {
    "timestamp": { "type": "number" },
    "pose": { "type": "object", "description": "Isometry3 (translation + rotation)" },
    "velocity_body": { "type": "object", "description": "Body-frame 6-DOF velocity" },
    "linear_acceleration_body": { "type": "object" },
    "angular_acceleration_body": { "type": "object" },
    "pose_covariance": { "type": "object" },
    "velocity_covariance": { "type": "object" }
  }
}"#;
