use std::collections::{HashMap, VecDeque};

use bevy::prelude::*;

/// Tracks the set of spawned `Text2d` label entities for TF frames.
/// Empty when `show_tf_frames` is off.
#[derive(Resource, Default)]
pub struct TfLabelEntities(pub HashMap<String, Entity>);

/// Global resource controlling all debug visualization toggles.
#[derive(Resource, Default)]
pub struct DebugVisualizationConfig {
    pub show_pose_gimbals: bool,   // F1
    pub show_covariance: bool,     // F2
    pub show_point_cloud: bool,    // F3
    pub show_velocity: bool,       // F4
    pub show_error_line: bool,     // F5
    pub show_path_trail: bool,     // F6
    pub show_occupancy_grid: bool, // F7
    /// Draw RGB coordinate axes for every tracked TF frame and the world ENU origin.
    pub show_tf_frames: bool, // F8
    /// Draw the planned path polyline and active look-ahead waypoint.
    pub show_planned_path: bool, // F9
    /// Draw axes at the body's oracle pose (read from `oracle/pose` on the
    /// autonomy bus). Distinct from `show_pose_gimbals`, which reads the
    /// ECS `GlobalTransform` directly. Use this to verify the oracle
    /// publish/read path end-to-end.
    pub show_oracle_pose: bool, // F10
    pub show_legend: bool,         // H
    pub show_vehicle_hud: bool,    // C
}

/// Caches the most recent world-space point cloud per sensor entity.
/// Populated by `cache_sensor_data`; read by `draw_point_cloud`.
///
/// Points are world-frame and frozen at the moment the scan was captured.
/// They are not re-transformed between lidar fires — the bus retains the
/// last-known-good batch across many `Update` frames, and retransforming
/// each frame would make the points slide with the moving sensor instead
/// of staying where the laser actually hit.
#[derive(Resource, Default)]
pub struct DebugSensorCache {
    pub point_clouds: HashMap<Entity, Vec<Vec3>>,
    /// Last batch timestamp (`Stamped::timestamp.0`) cached per sensor.
    /// `cache_sensor_data` re-transforms only when this advances.
    pub last_batch_ts: HashMap<Entity, f64>,
}

/// Marker component for the legend UI entity.
#[derive(Component)]
pub struct DebugLegendNode;

/// Ring buffer of recent GT positions for the path trail gizmo.
#[derive(Component)]
pub struct PathTrail {
    pub positions: VecDeque<Vec3>,
    pub max_len: usize,
}
