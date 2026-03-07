use nalgebra::{DMatrix, Isometry3, Translation3, UnitQuaternion};

use crate::estimation::FilterContext;
use crate::mapping::{MapData, Mapper};
use crate::messages::{MeasurementData, ModuleInput};
use crate::types::FrameHandle;

// Log-odds update constants
const L_FREE: f32 = -0.4;
const L_OCC: f32 = 0.85;
const L_MIN: f32 = -2.0;
const L_MAX: f32 = 3.5;

/// A local, egocentric, rolling-window occupancy grid (2-D, world-axis-aligned).
///
/// The grid follows the robot: when the robot drifts more than half the half-extent
/// from the current grid center, the log-odds buffer is shifted and newly exposed
/// strips are reset to 0 (unknown prior).  All coordinates are ENU (m).
///
/// # Known issues (to be fixed)
///
/// - **Coordinate frame bug**: hit points are currently transformed using the EKF
///   estimated pose rather than the physics ground-truth sensor pose from the TfTree.
///   This causes a ~90° rotation of the mapped obstacles relative to reality and
///   causes occupied cells to drift/smear as the EKF estimate evolves.
///
/// - **Hardcoded grid dimensions**: width and height are fixed at 200 m × 200 m in the
///   factory (`build_occupancy_grid_mapper`). They should be promoted to the
///   `MapperConfig::OccupancyGrid2D` TOML fields.
pub struct OccupancyGridMapper {
    resolution: f64,
    width: usize,
    height: usize,
    /// X coordinate of the west (left) edge of the grid in world ENU.
    origin_x: f64,
    /// Y coordinate of the south (bottom) edge of the grid in world ENU.
    origin_y: f64,
    /// Log-odds buffer, row-major: index = row * width + col, row 0 = south.
    log_odds: Vec<f32>,
    robot_pose: Option<Isometry3<f64>>,
    agent_handle: FrameHandle,
    cached_map: MapData,
}

impl OccupancyGridMapper {
    pub fn new(resolution: f64, width_m: f64, height_m: f64, agent_handle: FrameHandle) -> Self {
        let width = (width_m / resolution).ceil() as usize;
        let height = (height_m / resolution).ceil() as usize;
        Self {
            resolution,
            width,
            height,
            origin_x: -(width_m / 2.0),
            origin_y: -(height_m / 2.0),
            log_odds: vec![0.0_f32; width * height],
            robot_pose: None,
            agent_handle,
            cached_map: MapData::None,
        }
    }

    /// Shifts the grid buffer so the robot stays near the center.
    /// Exposed strips after the shift are zeroed (unknown prior).
    fn recenter_on(&mut self, rx: f64, ry: f64) {
        let half_w = (self.width as f64 * self.resolution) / 2.0;
        let half_h = (self.height as f64 * self.resolution) / 2.0;
        let cx = self.origin_x + half_w;
        let cy = self.origin_y + half_h;

        let dx = rx - cx;
        let dy = ry - cy;

        // Guard zone: 50 % of the half-extent.
        if dx.abs() <= half_w * 0.5 && dy.abs() <= half_h * 0.5 {
            return;
        }

        let dcol = (dx / self.resolution).round() as i64;
        let drow = (dy / self.resolution).round() as i64;
        if dcol == 0 && drow == 0 {
            return;
        }

        let w = self.width as i64;
        let h = self.height as i64;
        let mut shifted = vec![0.0_f32; self.width * self.height];

        for row in 0..h {
            for col in 0..w {
                let src_col = col - dcol;
                let src_row = row - drow;
                if src_col >= 0 && src_col < w && src_row >= 0 && src_row < h {
                    shifted[(row as usize) * self.width + (col as usize)] =
                        self.log_odds[(src_row as usize) * self.width + (src_col as usize)];
                }
            }
        }

        self.log_odds = shifted;
        self.origin_x += dcol as f64 * self.resolution;
        self.origin_y += drow as f64 * self.resolution;
    }

    /// Returns `(col, row)` for a world ENU point, or `None` if out of bounds.
    fn world_to_cell(&self, wx: f64, wy: f64) -> Option<(usize, usize)> {
        let col = ((wx - self.origin_x) / self.resolution).floor() as i64;
        let row = ((wy - self.origin_y) / self.resolution).floor() as i64;
        if col >= 0 && col < self.width as i64 && row >= 0 && row < self.height as i64 {
            Some((col as usize, row as usize))
        } else {
            None
        }
    }

    fn add_log_odds(&mut self, col: usize, row: usize, delta: f32) {
        let idx = row * self.width + col;
        self.log_odds[idx] = (self.log_odds[idx] + delta).clamp(L_MIN, L_MAX);
    }

    /// Bresenham ray from robot cell to hit cell.
    /// All intermediate cells get L_FREE; the endpoint gets L_OCC.
    fn raycast(&mut self, robot_wx: f64, robot_wy: f64, hit_wx: f64, hit_wy: f64) {
        let (Some((x0, y0)), Some((x1, y1))) = (
            self.world_to_cell(robot_wx, robot_wy),
            self.world_to_cell(hit_wx, hit_wy),
        ) else {
            return;
        };

        let cells: Vec<(i64, i64)> =
            bresenham(x0 as i64, y0 as i64, x1 as i64, y1 as i64).collect();
        let n = cells.len();

        let w = self.width as i64;
        let h = self.height as i64;
        for (i, (col, row)) in cells.into_iter().enumerate() {
            if col < 0 || col >= w || row < 0 || row >= h {
                continue;
            }
            let delta = if i + 1 == n { L_OCC } else { L_FREE };
            self.add_log_odds(col as usize, row as usize, delta);
        }
    }

    /// Converts the log-odds buffer to a `DMatrix<u8>` and updates `cached_map`.
    fn rebuild_cache(&mut self) {
        let data: Vec<u8> = self.log_odds.iter().map(|&lo| {
            // p = eˡ / (1 + eˡ)
            let p = lo.exp() / (1.0 + lo.exp());
            (p * 255.0) as u8
        }).collect();

        let mat = DMatrix::from_row_slice(self.height, self.width, &data);

        self.cached_map = MapData::OccupancyGrid2D {
            origin: Isometry3::from_parts(
                Translation3::new(self.origin_x, self.origin_y, 0.0),
                UnitQuaternion::identity(),
            ),
            resolution: self.resolution,
            data: mat,
        };
    }
}

impl Mapper for OccupancyGridMapper {
    fn process(&mut self, input: &ModuleInput, context: &FilterContext) {
        match input {
            ModuleInput::PoseUpdate { pose } => {
                if let Some(iso) = pose.get_pose_isometry() {
                    self.recenter_on(iso.translation.x, iso.translation.y);
                    self.robot_pose = Some(iso);
                    self.rebuild_cache();
                }
            }
            ModuleInput::Measurement { message } => {
                let MeasurementData::PointCloud(ref cloud) = message.data else {
                    return;
                };
                let robot_pose = match self.robot_pose {
                    Some(p) => p,
                    None => return,
                };
                let tf = match context.tf {
                    Some(tf) => tf,
                    None => return,
                };
                // T_{body←sensor}: transforms a sensor-frame point into body frame.
                let body_from_sensor =
                    match tf.get_transform(self.agent_handle, message.sensor_handle) {
                        Some(t) => t,
                        None => return,
                    };

                let t_world_sensor = robot_pose * body_from_sensor;
                let robot_wx = robot_pose.translation.x;
                let robot_wy = robot_pose.translation.y;

                for point in &cloud.points {
                    let p_world = t_world_sensor * point.position;
                    self.raycast(robot_wx, robot_wy, p_world.x, p_world.y);
                }
                // Cache is rebuilt on the next PoseUpdate (timer-gated).
                // Do not rebuild here to avoid O(width*height) work per scan.
            }
            _ => {}
        }
    }

    fn get_map(&self) -> &MapData {
        &self.cached_map
    }
}

// ---------------------------------------------------------------------------
// Bresenham line iterator
// ---------------------------------------------------------------------------

struct BresenhamIter {
    x: i64,
    y: i64,
    x1: i64,
    y1: i64,
    dx: i64,
    dy: i64,
    sx: i64,
    sy: i64,
    err: i64,
    done: bool,
}

impl Iterator for BresenhamIter {
    type Item = (i64, i64);

    fn next(&mut self) -> Option<(i64, i64)> {
        if self.done {
            return None;
        }
        let p = (self.x, self.y);
        if self.x == self.x1 && self.y == self.y1 {
            self.done = true;
            return Some(p);
        }
        let e2 = 2 * self.err;
        if e2 > -self.dy {
            self.err -= self.dy;
            self.x += self.sx;
        }
        if e2 < self.dx {
            self.err += self.dx;
            self.y += self.sy;
        }
        Some(p)
    }
}

fn bresenham(x0: i64, y0: i64, x1: i64, y1: i64) -> impl Iterator<Item = (i64, i64)> {
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    BresenhamIter {
        x: x0,
        y: y0,
        x1,
        y1,
        dx,
        dy,
        sx,
        sy,
        err: dx - dy,
        done: false,
    }
}
