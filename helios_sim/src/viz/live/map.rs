//! Occupancy-grid gizmo — the agent's 2-D map drawn as ground-plane cells.
//!
//! Like `path.rs`, `MapData` is a *plural* channel whose names belong to the
//! stack, so this pulls every declared producer via [`declared_outputs`] rather
//! than hardcoding a name. Only occupied cells are drawn; a full grid of
//! unknown/free cells would be visual noise and one gizmo per cell gets
//! expensive fast.
//!
//! The map is *not* redrawn from a per-frame scan — the occupancy grid is a
//! rolling-window log-odds map that accumulates belief over time, so the cells
//! shown are everything the agent currently remembers as occupied. (Bevy
//! gizmos are immediate-mode and already cleared each frame; the persistence
//! is in the map data, not here.) Because those cells are placed from the
//! agent's *estimated* pose, a divergent estimate shows up here as cells
//! offset from the true obstacles — the map faithfully draws a wrong belief.
//!
use crate::{
    core::transforms::EnuVector,
    prelude::AutonomyPipelineComponent,
    viz::live::discovery::declared_outputs,
};

use helios_core::mapping::MapData;

use bevy::{color, prelude::*};
use nalgebra::Vector3;
use std::f32::consts::FRAC_PI_2;

/// Cells at or below this map value (unknown ≈ 127, free < 127, occupied >
/// 127) are skipped; only confidently occupied cells are drawn.
// TODO: pull the occupancy cutoff from viz config once that surface exists.
const OCCUPIED_THRESHOLD: u8 = 127;

/// Occupied-cell color. Muted green-grey, distinct from the amber path line
/// and the red/green/blue pose triads it is drawn among.
// TODO: pull the map color from viz config once that surface exists.
const MAP_COLOR: color::Color = color::Color::Srgba(color::Srgba {
    red: 0.5,
    green: 0.65,
    blue: 0.5,
    alpha: 1.0,
});

pub fn map_update_system(query: Query<&AutonomyPipelineComponent>, mut gizmos: Gizmos) {
    for pipeline in &query {
        for map in declared_outputs::<MapData>(&pipeline.0) {
            let MapData::OccupancyGrid2D {
                origin,
                resolution,
                data,
                ..
            } = &map.value
            else {
                continue;
            };

            // `data[(row, col)]`: row 0 = south (+row → +north → +Y ENU),
            // col 0 = west (+col → +east → +X ENU); `origin` is the SW corner.
            for row in 0..data.nrows() {
                for col in 0..data.ncols() {
                    if data[(row, col)] <= OCCUPIED_THRESHOLD {
                        continue;
                    }

                    // Cell *center* in world ENU (the +0.5 centers the square
                    // on the cell). The map is a ground-plane belief, so z = 0;
                    // elevation would be a different MapData variant, not this.
                    let x = origin.translation.x + (col as f64 + 0.5) * resolution;
                    let y = origin.translation.y + (row as f64 + 0.5) * resolution;
                    let center_enu = EnuVector(Vector3::<f64>::new(x, y, 0.0));

                    // ENU center → Bevy point via the sole sanctioned helper.
                    let p = Vec3::from(center_enu);

                    // A cell-sized square laid flat on the ground. `rect` draws
                    // in its local XY plane; rotating +90° about X drops that
                    // plane onto Bevy's XZ ground. The tilt is a rendering
                    // choice (not a frame conversion), so it stays viz-local.
                    let cell = Isometry3d::new(p, Quat::from_rotation_x(FRAC_PI_2));
                    gizmos.rect(cell, Vec2::splat(*resolution as f32), MAP_COLOR);
                }
            }
        }
    }
}
