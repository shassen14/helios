use codspeed_criterion_compat::{criterion_group, criterion_main, Criterion};
use nalgebra::{DMatrix, DVector, Isometry3};

use helios_core::data::primitives::FrameHandle;
use helios_core::estimation::filters::ekf::ExtendedKalmanFilter;
use helios_core::estimation::filters::ukf::{UkfParams, UnscentedKalmanFilter};
use helios_core::estimation::measurement::MeasurementModel;
use helios_core::estimation::{EstimatorInputs, GaussianStateEstimator};
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
use helios_core::ports::TfProvider;
use helios_core::prelude::EstimationDynamics;

// =========================================================================
// == Fixtures (duplicated from test modules — benches are separate) ==
// =========================================================================

struct IdentityTf;

impl TfProvider for IdentityTf {
    fn get_transform(&self, _from: FrameHandle, _to: FrameHandle) -> Option<Isometry3<f64>> {
        Some(Isometry3::identity())
    }
    fn world_pose(&self, _frame: FrameHandle) -> Option<Isometry3<f64>> {
        Some(Isometry3::identity())
    }
}

#[derive(Debug, Clone)]
struct ConstantVelocity2D;

impl EstimationDynamics for ConstantVelocity2D {
    fn get_control_dim(&self) -> usize {
        0
    }

    fn get_derivatives(&self, x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
        let mut xdot = DVector::zeros(4);
        xdot[0] = x[2];
        xdot[1] = x[3];
        xdot
    }

    fn calculate_jacobian(
        &self,
        _x: &DVector<f64>,
        _u: &DVector<f64>,
        _t: f64,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let mut a = DMatrix::zeros(4, 4);
        a[(0, 2)] = 1.0;
        a[(1, 3)] = 1.0;
        (a, DMatrix::zeros(4, 0))
    }
}

#[derive(Debug, Clone)]
struct Position2DMeasurement;

impl MeasurementModel for Position2DMeasurement {
    fn dim(&self) -> usize {
        2
    }

    fn predict_measurement(
        &self,
        state: &FrameAwareState,
        _tf: Option<&dyn TfProvider>,
    ) -> Option<DVector<f64>> {
        Some(DVector::from_row_slice(&[
            state.state.vector[0],
            state.state.vector[1],
        ]))
    }

    fn jacobian(&self, state: &FrameAwareState, _tf: Option<&dyn TfProvider>) -> DMatrix<f64> {
        let n = state.dim();
        let mut h = DMatrix::zeros(2, n);
        h[(0, 0)] = 1.0;
        h[(1, 1)] = 1.0;
        h
    }
}

fn make_state() -> FrameAwareState {
    let layout = vec![
        StateVariable::Px(FrameId::World),
        StateVariable::Py(FrameId::World),
        StateVariable::Vx(FrameId::World),
        StateVariable::Vy(FrameId::World),
    ];
    let mut state = FrameAwareState::new(layout, 1.0, 0.0);
    state.state.vector[2] = 1.0; // vx = 1.0 m/s
    state
}

fn make_ekf() -> ExtendedKalmanFilter {
    let state = make_state();
    let q = DMatrix::identity(4, 4) * 0.01;
    ExtendedKalmanFilter::new(state, q, Box::new(ConstantVelocity2D))
}

fn make_ukf() -> UnscentedKalmanFilter {
    let state = make_state();
    let q = DMatrix::identity(4, 4) * 0.01;
    let params = UkfParams {
        alpha: 1e-3,
        beta: 2.0,
        kappa: 0.0,
    };
    UnscentedKalmanFilter::new(state, q, Box::new(ConstantVelocity2D), params)
}

fn gps_z(x: f64, y: f64) -> DVector<f64> {
    DVector::from_row_slice(&[x, y])
}

fn gps_r() -> DMatrix<f64> {
    DMatrix::identity(2, 2) * 0.1
}

// =========================================================================
// == Benchmarks ==
// =========================================================================

fn bench_ekf(c: &mut Criterion) {
    let u = DVector::zeros(0);
    let tf = IdentityTf;
    let inputs = EstimatorInputs { control: u };
    let z = gps_z(1.0, 0.0);
    let r = gps_r();
    let model = Position2DMeasurement;

    let mut group = c.benchmark_group("ekf");

    group.bench_function("predict", |b| {
        let mut ekf = make_ekf();
        b.iter(|| ekf.predict(0.1, &inputs));
    });

    group.bench_function("update", |b| {
        let mut ekf = make_ekf();
        b.iter(|| ekf.update(&z, &model, &r, Some(&tf)));
    });

    group.finish();
}

fn bench_ukf(c: &mut Criterion) {
    let u = DVector::zeros(0);
    let tf = IdentityTf;
    let inputs = EstimatorInputs { control: u };
    let z = gps_z(1.0, 0.0);
    let r = gps_r();
    let model = Position2DMeasurement;

    let mut group = c.benchmark_group("ukf");

    group.bench_function("predict", |b| {
        let mut ukf = make_ukf();
        b.iter(|| ukf.predict(0.1, &inputs));
    });

    group.bench_function("update", |b| {
        let mut ukf = make_ukf();
        b.iter(|| ukf.update(&z, &model, &r, Some(&tf)));
    });

    group.finish();
}

criterion_group!(benches, bench_ekf, bench_ukf);
criterion_main!(benches);
