use codspeed_criterion_compat::{criterion_group, criterion_main, Criterion};
use nalgebra::{DMatrix, Isometry3, Vector2};

use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
use helios_core::mapping::MapData;
use helios_core::planning::astar::{AStarConfig, AStarPlanner};
use helios_core::planning::context::PlannerContext;
use helios_core::planning::types::PlannerGoal;
use helios_core::planning::Planner;

// =========================================================================
// == Fixtures ==
// =========================================================================

fn bench_config() -> AStarConfig {
    AStarConfig {
        rate_hz: 0.0, // Disable rate gate — replan every call.
        arrival_tolerance_m: 0.5,
        occupancy_threshold: 128,
        max_search_depth: 100_000,
        enable_path_smoothing: false,
        replan_on_path_deviation: false,
        deviation_tolerance_m: 1.0,
        level_key: "global".into(),
    }
}

fn make_state(x: f64, y: f64) -> FrameAwareState {
    let layout = vec![
        StateVariable::Px(FrameId::World),
        StateVariable::Py(FrameId::World),
        StateVariable::Pz(FrameId::World),
    ];
    let mut state = FrameAwareState::new(layout, 0.0, 0.0);
    state.vector[0] = x;
    state.vector[1] = y;
    state.vector[2] = 0.0;
    state
}

fn clear_map(nrows: usize, ncols: usize, resolution: f64) -> MapData {
    MapData::OccupancyGrid2D {
        origin: Isometry3::identity(),
        resolution,
        data: DMatrix::from_element(nrows, ncols, 0u8),
        version: 0,
    }
}

fn obstacle_map_100x100(resolution: f64) -> MapData {
    let mut data = DMatrix::from_element(100, 100, 0u8);
    // Diagonal band of obstacles from (10,40) to (90,60).
    for i in 10usize..90 {
        let j = 40 + (i - 10) / 5;
        if j < 100 {
            data[(i, j)] = 255;
        }
    }
    MapData::OccupancyGrid2D {
        origin: Isometry3::identity(),
        resolution,
        data,
        version: 0,
    }
}

fn ctx() -> PlannerContext<'static> {
    PlannerContext { tf: None, now: 0.0 }
}

// =========================================================================
// == Benchmarks ==
// =========================================================================

fn bench_astar(c: &mut Criterion) {
    let mut group = c.benchmark_group("astar");

    // --- 50×50 free grid, corner to corner ---
    let map_50 = clear_map(50, 50, 1.0);
    let start_50 = make_state(0.5, 0.5);
    let goal_50 = PlannerGoal::WorldPosition2D(Vector2::new(49.5, 49.5));

    group.bench_function("50x50_free", |b| {
        b.iter(|| {
            let mut planner = AStarPlanner::new(bench_config());
            planner.set_goal(goal_50.clone());
            planner.plan(&start_50, &map_50, &ctx())
        });
    });

    // --- 100×100 free grid, corner to corner ---
    let map_100 = clear_map(100, 100, 1.0);
    let start_100 = make_state(0.5, 0.5);
    let goal_100 = PlannerGoal::WorldPosition2D(Vector2::new(99.5, 99.5));

    group.bench_function("100x100_free", |b| {
        b.iter(|| {
            let mut planner = AStarPlanner::new(bench_config());
            planner.set_goal(goal_100.clone());
            planner.plan(&start_100, &map_100, &ctx())
        });
    });

    // --- 100×100 grid with diagonal obstacle band ---
    let map_100_obs = obstacle_map_100x100(1.0);

    group.bench_function("100x100_obstacles", |b| {
        b.iter(|| {
            let mut planner = AStarPlanner::new(bench_config());
            planner.set_goal(goal_100.clone());
            planner.plan(&start_100, &map_100_obs, &ctx())
        });
    });

    group.finish();
}

criterion_group!(benches, bench_astar);
criterion_main!(benches);
