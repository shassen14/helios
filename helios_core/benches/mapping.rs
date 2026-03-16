use codspeed_criterion_compat::{criterion_group, criterion_main, Criterion};
use nalgebra::{Isometry3, Point3};

use helios_core::estimation::FilterContext;
use helios_core::mapping::{Mapper, MapperPoseSource, OccupancyGridMapper};
use helios_core::messages::{MeasurementData, MeasurementMessage, ModuleInput, Point, PointCloud};
use helios_core::types::FrameHandle;

// =========================================================================
// == Fixtures ==
// =========================================================================

const AGENT: FrameHandle = FrameHandle(0);
const SENSOR: FrameHandle = FrameHandle(1);

fn make_mapper() -> OccupancyGridMapper {
    // 100×100 m at 1 m/cell = 100×100 cells.
    OccupancyGridMapper::new(1.0, 100.0, 100.0, AGENT, MapperPoseSource::Estimated)
}

fn ctx() -> FilterContext<'static> {
    FilterContext { tf: None }
}

/// Pose update at (x, y).
fn pose_update(x: f64, y: f64) -> ModuleInput<'static> {
    ModuleInput::PoseUpdate {
        pose: Isometry3::translation(x, y, 0.0),
    }
}

/// Synthesise a 360-beam scan (one ray per degree) from the robot origin outward
/// to `range_m`, all rays along the ground plane.
fn make_scan(range_m: f64) -> Vec<Point> {
    (0..360)
        .map(|deg| {
            let angle = (deg as f64).to_radians();
            Point {
                position: Point3::new(angle.cos() * range_m, angle.sin() * range_m, 0.0),
                intensity: None,
            }
        })
        .collect()
}

fn scan_message(points: Vec<Point>) -> MeasurementMessage {
    MeasurementMessage {
        agent_handle: AGENT,
        sensor_handle: SENSOR,
        timestamp: 0.0,
        data: MeasurementData::PointCloud(PointCloud {
            sensor_handle: SENSOR,
            timestamp: 0.0,
            points,
        }),
    }
}

// =========================================================================
// == Benchmarks ==
// =========================================================================

/// Single 360-beam LiDAR scan applied to an empty mapper (exercises `raycast` only).
fn bench_raycast_single(c: &mut Criterion) {
    let scan = make_scan(30.0);
    let msg = scan_message(scan);

    c.bench_function("mapping/raycast_360_beams", |b| {
        b.iter(|| {
            let mut mapper = make_mapper();
            // Prime the pose so Estimated path resolves.
            mapper.process(&pose_update(0.0, 0.0), &ctx());
            mapper.process(&ModuleInput::Measurement { message: &msg }, &ctx());
        });
    });
}

/// `rebuild_cache` cost after 100 raycasts — the dominant per-pose-update work.
fn bench_rebuild_cache(c: &mut Criterion) {
    // Pre-populate the mapper with 100 scans at origin.
    let scan = make_scan(20.0);
    let msg = scan_message(scan);

    let mut mapper = make_mapper();
    mapper.process(&pose_update(0.0, 0.0), &ctx());
    for _ in 0..100 {
        mapper.process(&ModuleInput::Measurement { message: &msg }, &ctx());
    }

    c.bench_function("mapping/rebuild_cache_100x100", |b| {
        b.iter(|| {
            // Force a rebuild each iteration by issuing a pose update.
            mapper.process(&pose_update(0.0, 0.0), &ctx());
        });
    });
}

/// Grid-shift cost when the robot moves far enough to trigger `recenter_on`.
fn bench_recenter(c: &mut Criterion) {
    c.bench_function("mapping/recenter_shift", |b| {
        b.iter(|| {
            let mut mapper = make_mapper();
            // Initial pose — sets robot_pose but origin is at (-50, -50) for 100×100.
            mapper.process(&pose_update(0.0, 0.0), &ctx());
            // Move to x = 40 — beyond the 50 % guard zone (25 m), triggers shift.
            mapper.process(&pose_update(40.0, 0.0), &ctx());
        });
    });
}

/// Realistic full tick: 360-beam scan followed by a pose-gated rebuild.
fn bench_full_scan(c: &mut Criterion) {
    let scan = make_scan(30.0);
    let msg = scan_message(scan);

    c.bench_function("mapping/full_tick_scan_plus_rebuild", |b| {
        b.iter(|| {
            let mut mapper = make_mapper();
            mapper.process(&pose_update(0.0, 0.0), &ctx());
            mapper.process(&ModuleInput::Measurement { message: &msg }, &ctx());
            // Pose update triggers rebuild.
            mapper.process(&pose_update(0.1, 0.0), &ctx());
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
