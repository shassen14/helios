use bevy::prelude::*;
use helios_core::mapping::MapData;
use helios_runtime::stage::PipelineLevel;

use crate::simulation::core::components::GroundTruthState;
use crate::simulation::plugins::autonomy::MapperComponent;
use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;

/// Render radius around each agent. Only cells within this many metres of
/// the agent's ground-truth position are drawn.
const GRID_DRAW_RADIUS_M: f32 = 150.0;

/// Draws a bird's-eye view of the occupancy grid for every agent that has
/// an active `OccupancyGrid2D` mapper.
pub fn draw_occupancy_grid(
    config: Res<DebugVisualizationConfig>,
    mut gizmos: Gizmos,
    query: Query<(&MapperComponent, &GlobalTransform), With<GroundTruthState>>,
) {
    if !config.show_occupancy_grid {
        return;
    }

    let flat = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);

    for (module, gt_transform) in &query {
        let Some(map) = module.0.get_map(&PipelineLevel::Local) else {
            continue;
        };

        let MapData::OccupancyGrid2D { origin, resolution, data } = map else {
            continue;
        };

        let res = *resolution as f32;
        let rows = data.nrows();
        let cols = data.ncols();
        let ox = origin.translation.x as f32;
        let oy = origin.translation.y as f32;

        let agent_bevy = gt_transform.translation();
        let agent_ex = agent_bevy.x;
        let agent_ey = -agent_bevy.z;
        let ground_y = agent_bevy.y;

        let grid_w = cols as f32 * res;
        let grid_h = rows as f32 * res;
        let border_cx_enu = ox + grid_w * 0.5;
        let border_cy_enu = oy + grid_h * 0.5;
        let border_center = Vec3::new(border_cx_enu, ground_y, -border_cy_enu);
        gizmos.rect(
            Isometry3d::new(border_center, flat),
            Vec2::new(grid_w, grid_h),
            Color::srgba(1.0, 1.0, 1.0, 0.3),
        );

        let r_cells_x = (GRID_DRAW_RADIUS_M / res).ceil() as i64;
        let r_cells_y = (GRID_DRAW_RADIUS_M / res).ceil() as i64;

        let agent_col = ((agent_ex - ox) / res).floor() as i64;
        let agent_row = ((agent_ey - oy) / res).floor() as i64;

        let col_min = (agent_col - r_cells_x).max(0) as usize;
        let col_max = (agent_col + r_cells_x).min(cols as i64 - 1) as usize;
        let row_min = (agent_row - r_cells_y).max(0) as usize;
        let row_max = (agent_row + r_cells_y).min(rows as i64 - 1) as usize;

        let r2 = GRID_DRAW_RADIUS_M * GRID_DRAW_RADIUS_M;

        for row in row_min..=row_max {
            for col in col_min..=col_max {
                let p = data[(row, col)] as f32 / 255.0;

                let color = if p > 0.65 {
                    let a = ((p - 0.65) / 0.35).clamp(0.1, 0.85);
                    Color::srgba(1.0, 0.15, 0.0, a)
                } else if p < 0.45 {
                    let a = ((0.45 - p) / 0.45).clamp(0.05, 0.4);
                    Color::srgba(0.1, 0.9, 0.2, a)
                } else {
                    continue;
                };

                let cx_enu = ox + (col as f32 + 0.5) * res;
                let cy_enu = oy + (row as f32 + 0.5) * res;

                let dx = cx_enu - agent_ex;
                let dy = cy_enu - agent_ey;
                if dx * dx + dy * dy > r2 {
                    continue;
                }

                let center = Vec3::new(cx_enu, ground_y + 0.02, -cy_enu);
                gizmos.rect(Isometry3d::new(center, flat), Vec2::splat(res * 0.92), color);
            }
        }
    }
}
