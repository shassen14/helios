use std::sync::Arc;

use codspeed_criterion_compat::{criterion_group, criterion_main, Criterion};
use helios_core::state::{Component, Quantity};
use nalgebra::{DMatrix, DVector, Isometry3};

use helios_core::data::ports::TfProvider;
use helios_core::data::MonotonicTime;
use helios_core::estimation::filters::ekf::ExtendedKalmanFilter;
use helios_core::estimation::filters::ukf::{UkfParams, UnscentedKalmanFilter};
use helios_core::estimation::measurement::MeasurementModel;
use helios_core::estimation::schema::{SchemaBlock, StateSchema};
use helios_core::estimation::{EstimatorInputs, GaussianStateEstimator};
use helios_core::frames::transforms::{Convention, ErasedTransform};
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
use helios_core::prelude::EstimationDynamics;

// =========================================================================
// == Fixtures (duplicated from test modules — benches are separate) ==
// =========================================================================

const AT: MonotonicTime = MonotonicTime(0.0);

struct IdentityTf;

impl TfProvider for IdentityTf {
    fn get_transform(
        &self,
        _from: FrameId,
        _to: FrameId,
        _at: MonotonicTime,
    ) -> Option<ErasedTransform> {
        Some(ErasedTransform::from_parts(
            Isometry3::identity(),
            Convention::Flu,
            Convention::Flu,
        ))
    }
}

#[derive(Debug, Clone)]
struct ConstantVelocity3D;

impl EstimationDynamics for ConstantVelocity3D {
    fn get_control_dim(&self) -> usize {
        0
    }

    fn schema(&self) -> Arc<StateSchema> {
        Arc::new(StateSchema::compose(vec![
            SchemaBlock::new(
                Quantity::Position(FrameId::World),
                None,
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            SchemaBlock::new(
                Quantity::Velocity(FrameId::World),
                None,
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
        ]))
    }

    fn derivatives(&self, x: &DVector<f64>, _u: &DVector<f64>, _t: f64) -> DVector<f64> {
        let mut xdot = DVector::zeros(6);
        xdot[0] = x[3];
        xdot[1] = x[4];
        xdot[2] = x[5];
        xdot
    }

    fn jacobian(
        &self,
        _x: &DVector<f64>,
        _u: &DVector<f64>,
        _t: f64,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let mut a = DMatrix::zeros(6, 6);
        a[(0, 3)] = 1.0;
        a[(1, 4)] = 1.0;
        a[(2, 5)] = 1.0;
        (a, DMatrix::zeros(6, 0))
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
        _at: MonotonicTime,
    ) -> Option<DVector<f64>> {
        Some(DVector::from_row_slice(&[state.mean[0], state.mean[1]]))
    }

    fn jacobian(
        &self,
        state: &FrameAwareState,
        _tf: Option<&dyn TfProvider>,
        _at: MonotonicTime,
    ) -> DMatrix<f64> {
        let n = state.tangent_dim();
        let mut h = DMatrix::zeros(2, n);
        h[(0, 0)] = 1.0;
        h[(1, 1)] = 1.0;
        h
    }
}

fn make_state() -> FrameAwareState {
    // Layout is [px, py, pz, vx, vy, vz]; Vx is index 3.
    let mut state = FrameAwareState::from_schema(ConstantVelocity3D.schema(), 0.0);
    state.set_variable(
        &StateVariable::new(Quantity::Velocity(FrameId::World), Component::X),
        1.0,
    ); // vx = 1.0 m/s
    state
}

fn make_ekf() -> ExtendedKalmanFilter {
    let state = make_state();
    let q = DMatrix::identity(6, 6) * 0.01;
    ExtendedKalmanFilter::new(state, q, Box::new(ConstantVelocity3D))
}

fn make_ukf() -> UnscentedKalmanFilter {
    let state = make_state();
    let q = DMatrix::identity(6, 6) * 0.01;
    let params = UkfParams {
        alpha: 1e-3,
        beta: 2.0,
        kappa: 0.0,
    };
    UnscentedKalmanFilter::new(state, q, Box::new(ConstantVelocity3D), params)
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
        b.iter(|| ekf.update(&z, &model, &r, Some(&tf), AT));
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
        b.iter(|| ukf.update(&z, &model, &r, Some(&tf), AT));
    });

    group.finish();
}

criterion_group!(benches, bench_ekf, bench_ukf);
criterion_main!(benches);
