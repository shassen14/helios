use codspeed_criterion_compat::{criterion_group, criterion_main, Criterion};
use nalgebra::{Isometry3, Point2};

use helios_core::data::sensor::PointCloud2D;
use helios_core::mapping::{Mapper, OccupancyGridMapper};

// =========================================================================
// == Fixtures ==
// =========================================================================

fn make_mapper() -> OccupancyGridMapper {
    // 100×100 m at 1 m/cell = 100×100 cells.
    OccupancyGridMapper::new(1.0, 100.0, 100.0)
}

fn pose(x: f64, y: f64) -> Isometry3<f64> {
    Isometry3::translation(x, y, 0.0)
}

/// Synthesise a 360-beam scan (one ray per degree) from the robot origin outward
/// to `range_m`, all rays along the ground plane.
fn make_scan(range_m: f64) -> PointCloud2D {
    let points = (0..360)
        .map(|deg| {
            let angle = (deg as f64).to_radians();
            Point2::new(angle.cos() * range_m, angle.sin() * range_m)
        })
        .collect();
    PointCloud2D { points }
}

// =========================================================================
// == Benchmarks ==
// =========================================================================

/// Single 360-beam LiDAR scan applied to an empty mapper (exercises `raycast` only).
fn bench_raycast_single(c: &mut Criterion) {
    let scan = make_scan(30.0);
    let sensor_pose = pose(0.0, 0.0);

    c.bench_function("mapping/raycast_360_beams", |b| {
        b.iter(|| {
            let mut mapper = make_mapper();
            mapper.recenter(&pose(0.0, 0.0));
            mapper.integrate_scan_2d(&sensor_pose, &scan);
        });
    });
}

/// `rebuild_cache` cost after 100 raycasts — the dominant per-pose-update work.
fn bench_rebuild_cache(c: &mut Criterion) {
    let scan = make_scan(20.0);
    let sensor_pose = pose(0.0, 0.0);

    let mut mapper = make_mapper();
    mapper.recenter(&pose(0.0, 0.0));
    for _ in 0..100 {
        mapper.integrate_scan_2d(&sensor_pose, &scan);
    }

    c.bench_function("mapping/rebuild_cache_100x100", |b| {
        b.iter(|| {
            // Force a rebuild each iteration by issuing a pose update.
            mapper.recenter(&pose(0.0, 0.0));
            mapper.get_map();
        });
    });
}

/// Grid-shift cost when the robot moves far enough to trigger `recenter_on`.
fn bench_recenter(c: &mut Criterion) {
    c.bench_function("mapping/recenter_shift", |b| {
        b.iter(|| {
            let mut mapper = make_mapper();
            mapper.recenter(&pose(0.0, 0.0));
            // Move to x = 40 — beyond the 50 % guard zone (25 m), triggers shift.
            mapper.recenter(&pose(40.0, 0.0));
        });
    });
}

/// Realistic full tick: 360-beam scan followed by a pose-gated rebuild.
fn bench_full_scan(c: &mut Criterion) {
    let scan = make_scan(30.0);
    let sensor_pose = pose(0.0, 0.0);

    c.bench_function("mapping/full_tick_scan_plus_rebuild", |b| {
        b.iter(|| {
            let mut mapper = make_mapper();
            mapper.recenter(&pose(0.0, 0.0));
            mapper.integrate_scan_2d(&sensor_pose, &scan);
            // Pose update triggers rebuild.
            mapper.recenter(&pose(0.1, 0.0));
            mapper.get_map();
        });
    });
}

criterion_group!(
    benches,
    bench_raycast_single,
    bench_rebuild_cache,
    bench_recenter,
    bench_full_scan
);
criterion_main!(benches);
