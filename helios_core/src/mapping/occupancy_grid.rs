use nalgebra::{DMatrix, Isometry3, Translation3, UnitQuaternion};

use crate::estimation::FilterContext;
use crate::frames::conventions::sensor_pose_flu_to_world;
use crate::mapping::{MapData, Mapper};
use crate::messages::{MeasurementData, ModuleInput};
use crate::types::FrameHandle;

/// Selects how the mapper obtains the sensor's world pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapperPoseSource {
    /// Physics ground-truth from TfTree. Decoupled from estimation.
    GroundTruth,
    /// EKF/odom estimate. Map lives in the odom frame.
    Estimated,
}

/// Log-odds free-space update increment (negative → lowers occupancy probability).
const L_FREE: f32 = -0.4;
/// Log-odds occupied update increment (positive → raises occupancy probability).
const L_OCC: f32 = 0.85;
/// Minimum clamped log-odds value (saturates at very-free).
const L_MIN: f32 = -2.0;
/// Maximum clamped log-odds value (saturates at very-occupied).
const L_MAX: f32 = 3.5;

/// A local, egocentric, rolling-window occupancy grid (2-D, world-axis-aligned).
///
/// The grid follows the robot: when the robot drifts more than half the half-extent
/// from the current grid center, the log-odds buffer is shifted and newly exposed
/// strips are reset to 0 (unknown prior).  All coordinates are ENU (m).
///
/// # Allocation strategy
///
/// - `log_odds` and `shift_buffer` are allocated once in `new()` and reused for the
///   lifetime of the mapper.
/// - `rebuild_cache()` updates the cached `DMatrix<u8>` **in place** after the first
///   call; no heap allocation occurs during normal operation.
/// - `recenter_on()` reuses `shift_buffer` instead of allocating a temporary `Vec`.
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
    /// Pre-allocated scratch buffer for `recenter_on` shifts. Same length as `log_odds`.
    shift_buffer: Vec<f32>,
    /// Set to `true` whenever `log_odds` or `origin` changes. Cleared by `rebuild_cache`.
    map_dirty: bool,
    /// Monotonically increasing; copied into `MapData::OccupancyGrid2D::version` on rebuild.
    map_version: u64,
    robot_pose: Option<Isometry3<f64>>,
    agent_handle: FrameHandle,
    pose_source: MapperPoseSource,
    cached_map: MapData,
}

impl OccupancyGridMapper {
    /// Creates a new mapper with the given grid dimensions.
    ///
    /// `width_m` and `height_m` are in metres; `resolution` is metres-per-cell.
    /// The grid is initially centred on the world origin with all cells unknown (log-odds 0).
    pub fn new(
        resolution: f64,
        width_m: f64,
        height_m: f64,
        agent_handle: FrameHandle,
        pose_source: MapperPoseSource,
    ) -> Self {
        let width = (width_m / resolution).ceil() as usize;
        let height = (height_m / resolution).ceil() as usize;
        let n = width * height;
        Self {
            resolution,
            width,
            height,
            origin_x: -(width_m / 2.0),
            origin_y: -(height_m / 2.0),
            log_odds: vec![0.0_f32; n],
            shift_buffer: vec![0.0_f32; n],
            map_dirty: false,
            map_version: 0,
            robot_pose: None,
            agent_handle,
            pose_source,
            cached_map: MapData::None,
        }
    }

    /// Shifts the grid buffer so the robot stays near the center.
    ///
    /// If the robot (`rx`, `ry`) is within 50 % of the half-extent from the current
    /// grid center, this is a no-op. Otherwise the log-odds buffer is shifted by the
    /// corresponding number of cells using the pre-allocated `shift_buffer`, and newly
    /// exposed strips are zeroed (unknown prior).
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

        // Zero the scratch buffer and copy the overlapping region into it.
        self.shift_buffer.fill(0.0);
        for row in 0..h {
            for col in 0..w {
                let src_col = col + dcol;
                let src_row = row + drow;
                if src_col >= 0 && src_col < w && src_row >= 0 && src_row < h {
                    self.shift_buffer[(row as usize) * self.width + (col as usize)] =
                        self.log_odds[(src_row as usize) * self.width + (src_col as usize)];
                }
            }
        }

        // Swap buffers — avoids a copy_from_slice and re-uses both allocations.
        std::mem::swap(&mut self.log_odds, &mut self.shift_buffer);

        self.origin_x += dcol as f64 * self.resolution;
        self.origin_y += drow as f64 * self.resolution;
        self.map_dirty = true;
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

    /// Applies a log-odds delta to a single cell, clamped to `[L_MIN, L_MAX]`.
    fn add_log_odds(&mut self, col: usize, row: usize, delta: f32) {
        let idx = row * self.width + col;
        self.log_odds[idx] = (self.log_odds[idx] + delta).clamp(L_MIN, L_MAX);
        self.map_dirty = true;
    }

    /// Bresenham ray from robot cell to hit cell.
    ///
    /// All intermediate cells receive an `L_FREE` update; the endpoint receives `L_OCC`.
    /// Cells that fall outside the grid bounds are silently skipped.
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
    ///
    /// This is a no-op when `map_dirty` is `false` (i.e. no `add_log_odds` or
    /// `recenter_on` calls occurred since the last rebuild). On the first call the
    /// `DMatrix` is allocated; subsequent calls update it **in place**, avoiding
    /// heap allocation entirely.
    fn rebuild_cache(&mut self) {
        if !self.map_dirty {
            return;
        }

        let new_origin = Isometry3::from_parts(
            Translation3::new(self.origin_x, self.origin_y, 0.0),
            UnitQuaternion::identity(),
        );
        let new_resolution = self.resolution;

        if let MapData::OccupancyGrid2D {
            ref mut data,
            ref mut origin,
            ref mut resolution,
            ref mut version,
        } = self.cached_map
        {
            // Update in-place: no allocation.
            for row in 0..self.height {
                for col in 0..self.width {
                    let lo = self.log_odds[row * self.width + col];
                    let p = lo.exp() / (1.0 + lo.exp());
                    data[(row, col)] = (p * 255.0) as u8;
                }
            }
            *origin = new_origin;
            *resolution = new_resolution;
            self.map_version += 1;
            *version = self.map_version;
        } else {
            // First call: allocate the DMatrix once.
            let data = DMatrix::from_fn(self.height, self.width, |row, col| {
                let lo = self.log_odds[row * self.width + col];
                let p = lo.exp() / (1.0 + lo.exp());
                (p * 255.0) as u8
            });
            self.map_version += 1;
            self.cached_map = MapData::OccupancyGrid2D {
                origin: new_origin,
                resolution: new_resolution,
                data,
                version: self.map_version,
            };
        }

        self.map_dirty = false;
    }
}

impl Mapper for OccupancyGridMapper {
    fn process(&mut self, input: &ModuleInput, context: &FilterContext) {
        match input {
            ModuleInput::PoseUpdate { pose } => {
                self.recenter_on(pose.translation.x, pose.translation.y);
                self.robot_pose = Some(*pose);
                self.rebuild_cache();
            }
            ModuleInput::Measurement { message } => {
                let MeasurementData::PointCloud(ref cloud) = message.data else {
                    return;
                };
                let tf = match context.tf {
                    Some(tf) => tf,
                    None => return,
                };

                let (sensor_world_pose, robot_wx, robot_wy) = match self.pose_source {
                    MapperPoseSource::GroundTruth => {
                        let robot_pose = match tf.world_pose(self.agent_handle) {
                            Some(p) => p,
                            None => return,
                        };
                        let sensor_pose = match tf.world_pose(message.sensor_handle) {
                            Some(p) => p,
                            None => return,
                        };
                        (
                            sensor_pose,
                            robot_pose.translation.x,
                            robot_pose.translation.y,
                        )
                    }
                    MapperPoseSource::Estimated => {
                        let robot_pose = match self.robot_pose {
                            Some(p) => p,
                            None => return,
                        };
                        let body_from_sensor =
                            match tf.get_transform(self.agent_handle, message.sensor_handle) {
                                Some(t) => t,
                                None => return,
                            };
                        let sensor_pose = robot_pose * body_from_sensor;
                        (
                            sensor_pose,
                            robot_pose.translation.x,
                            robot_pose.translation.y,
                        )
                    }
                };

                // points are in FLU; compose the FLU→ENU correction so we
                // can project directly into the ENU world frame.
                let t_world_from_flu = sensor_pose_flu_to_world(sensor_world_pose);

                for point in &cloud.points {
                    let p_world = t_world_from_flu * point.position;
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

/// Iterator that yields integer grid cells along a line using Bresenham's algorithm.
///
/// Produces every cell from `(x0, y0)` to `(x1, y1)` inclusive, in order.
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

/// Returns an iterator yielding all grid cells from `(x0, y0)` to `(x1, y1)` inclusive
/// using Bresenham's line algorithm.
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_mapper(width_m: f64, height_m: f64) -> OccupancyGridMapper {
        OccupancyGridMapper::new(1.0, width_m, height_m, FrameHandle(0), MapperPoseSource::Estimated)
    }

    // -------------------------------------------------------------------------
    // world_to_cell
    // -------------------------------------------------------------------------

    #[test]
    fn world_to_cell_in_bounds() {
        // 10×10 grid centred at origin: origin_x = -5, origin_y = -5.
        let m = make_mapper(10.0, 10.0);
        // Point (1.5, 2.5) → col = floor((1.5 - (-5)) / 1) = 6, row = 7.
        assert_eq!(m.world_to_cell(1.5, 2.5), Some((6, 7)));
    }

    #[test]
    fn world_to_cell_out_of_bounds() {
        let m = make_mapper(10.0, 10.0);
        // x = 100 is far outside the 10-wide grid.
        assert_eq!(m.world_to_cell(100.0, 0.0), None);
    }

    #[test]
    fn world_to_cell_origin_offset() {
        // Grid manually offset: origin_x = 10, origin_y = 10.
        let mut m = make_mapper(10.0, 10.0);
        m.origin_x = 10.0;
        m.origin_y = 10.0;
        // Point (11.5, 10.5) → col = floor((11.5-10)/1) = 1, row = 0.
        assert_eq!(m.world_to_cell(11.5, 10.5), Some((1, 0)));
        // Point (9.9, 10.0) is to the left of origin_x → out of bounds.
        assert_eq!(m.world_to_cell(9.9, 10.0), None);
    }

    // -------------------------------------------------------------------------
    // add_log_odds
    // -------------------------------------------------------------------------

    #[test]
    fn add_log_odds_clamps() {
        let mut m = make_mapper(10.0, 10.0);
        // Drive the cell to L_MAX.
        for _ in 0..100 {
            m.add_log_odds(0, 0, L_OCC);
        }
        assert_eq!(m.log_odds[0], L_MAX);

        // Drive it to L_MIN.
        for _ in 0..100 {
            m.add_log_odds(0, 0, L_FREE);
        }
        assert_eq!(m.log_odds[0], L_MIN);
    }

    // -------------------------------------------------------------------------
    // recenter_on
    // -------------------------------------------------------------------------

    #[test]
    fn recenter_no_shift_in_guard_zone() {
        let mut m = make_mapper(10.0, 10.0);
        // Robot at origin (0,0) — well within the 50% guard zone of a 10×10 grid.
        m.recenter_on(0.0, 0.0);
        // origin should stay at (-5, -5).
        assert_eq!(m.origin_x, -5.0);
        assert_eq!(m.origin_y, -5.0);
        // map_dirty should not have been set.
        assert!(!m.map_dirty);
    }

    #[test]
    fn recenter_shifts_correctly() {
        let mut m = make_mapper(10.0, 10.0);
        // origin_x = -5, cx = 0.  Guard zone half-extent = 2.5.
        // Place the robot at x = 4.0 (dx = 4.0 > 2.5) → triggers shift.
        // Record both pre-allocated buffer addresses.
        let ptr_log_odds = m.log_odds.as_ptr();
        let ptr_shift = m.shift_buffer.as_ptr();
        m.recenter_on(4.0, 0.0);
        // After the swap, log_odds must point to one of the two pre-allocated buffers —
        // no new heap allocation should have occurred.
        let ptr_after = m.log_odds.as_ptr();
        assert!(
            ptr_after == ptr_log_odds || ptr_after == ptr_shift,
            "recenter_on must not allocate a new buffer"
        );
        // origin_x should have advanced by 4 cells * 1.0 m/cell = 4.0 m.
        assert!((m.origin_x - (-1.0)).abs() < 1e-9);
        assert!(m.map_dirty);
    }

    // -------------------------------------------------------------------------
    // raycast
    // -------------------------------------------------------------------------

    #[test]
    fn raycast_marks_free_and_occupied() {
        let mut m = make_mapper(10.0, 10.0);
        // Robot at origin (0,0) cell = (5,5).  Hit at (2,0) cell = (7,5).
        m.raycast(0.0, 0.0, 2.0, 0.0);

        // The endpoint cell (7,5) should be > 0 (occupied update).
        let occ_idx = 5 * m.width + 7;
        assert!(m.log_odds[occ_idx] > 0.0, "endpoint should have positive log-odds");

        // An intermediate cell, e.g. (6,5), should have negative log-odds (free update).
        let free_idx = 5 * m.width + 6;
        assert!(m.log_odds[free_idx] < 0.0, "intermediate cell should have negative log-odds");

        assert!(m.map_dirty);
    }

    // -------------------------------------------------------------------------
    // rebuild_cache
    // -------------------------------------------------------------------------

    #[test]
    fn rebuild_cache_sigmoid_conversion() {
        let mut m = make_mapper(4.0, 4.0);

        // log_odds = 0 → p = 0.5 → value ≈ 127.
        m.map_dirty = true;
        m.rebuild_cache();
        let MapData::OccupancyGrid2D { ref data, .. } = m.cached_map else {
            panic!("expected OccupancyGrid2D");
        };
        assert_eq!(data[(0, 0)], 127);

        // log_odds = L_MAX → p near 1 → value near 255.
        m.log_odds[0] = L_MAX;
        m.map_dirty = true;
        m.rebuild_cache();
        let MapData::OccupancyGrid2D { ref data, .. } = m.cached_map else {
            panic!("expected OccupancyGrid2D");
        };
        assert!(data[(0, 0)] > 240, "L_MAX log-odds should map to near-255 value");
    }

    #[test]
    fn rebuild_cache_skips_when_not_dirty() {
        let mut m = make_mapper(4.0, 4.0);

        // First build.
        m.map_dirty = true;
        m.rebuild_cache();
        let v1 = match &m.cached_map {
            MapData::OccupancyGrid2D { version, .. } => *version,
            _ => panic!("expected OccupancyGrid2D"),
        };

        // Second call with no intervening dirty — version must not change.
        m.rebuild_cache();
        let v2 = match &m.cached_map {
            MapData::OccupancyGrid2D { version, .. } => *version,
            _ => panic!("expected OccupancyGrid2D"),
        };

        assert_eq!(v1, v2, "version should not increment when map is not dirty");
    }

    #[test]
    fn rebuild_cache_in_place_no_realloc() {
        let mut m = make_mapper(4.0, 4.0);

        m.map_dirty = true;
        m.rebuild_cache();

        let ptr1 = match &m.cached_map {
            MapData::OccupancyGrid2D { data, .. } => data.as_slice().as_ptr(),
            _ => panic!("expected OccupancyGrid2D"),
        };

        // Dirty the map and rebuild again.
        m.add_log_odds(0, 0, L_OCC);
        m.rebuild_cache();

        let ptr2 = match &m.cached_map {
            MapData::OccupancyGrid2D { data, .. } => data.as_slice().as_ptr(),
            _ => panic!("expected OccupancyGrid2D"),
        };

        assert_eq!(ptr1, ptr2, "DMatrix buffer must not be reallocated on second rebuild");
    }

    // -------------------------------------------------------------------------
    // process (Estimated pose source, no robot_pose) — measurement is a no-op
    // -------------------------------------------------------------------------

    #[test]
    fn process_measurement_without_pose_is_noop() {
        use crate::messages::{MeasurementData, MeasurementMessage, PointCloud};

        let mut m = OccupancyGridMapper::new(
            1.0, 10.0, 10.0, FrameHandle(0), MapperPoseSource::Estimated,
        );

        let msg = MeasurementMessage {
            agent_handle: FrameHandle(0),
            sensor_handle: FrameHandle(1),
            timestamp: 0.0,
            data: MeasurementData::PointCloud(PointCloud {
                sensor_handle: FrameHandle(1),
                timestamp: 0.0,
                points: vec![],
            }),
        };
        let ctx = FilterContext { tf: None };
        m.process(&ModuleInput::Measurement { message: &msg }, &ctx);

        // No pose was ever set, so the process call should be a silent no-op.
        assert!(matches!(m.cached_map, MapData::None));
        assert!(!m.map_dirty);
    }
}
