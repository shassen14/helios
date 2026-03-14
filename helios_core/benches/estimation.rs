use std::any::Any;
use std::collections::HashMap;

use criterion::{criterion_group, criterion_main, Criterion};
use nalgebra::{DMatrix, DVector, Isometry3, Vector3};

use helios_core::estimation::filters::ekf::ExtendedKalmanFilter;
use helios_core::estimation::filters::ukf::{UkfParams, UnscentedKalmanFilter};
use helios_core::estimation::{FilterContext, StateEstimator};
use helios_core::frames::{FrameAwareState, FrameId, StateVariable};
use helios_core::messages::{MeasurementData, MeasurementMessage};
use helios_core::models::estimation::measurement::Measurement;
use helios_core::prelude::EstimationDynamics;
use helios_core::types::{FrameHandle, TfProvider};

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

    fn get_control_from_measurement(&self, _data: &MeasurementData) -> Option<DVector<f64>> {
        None
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
struct Position2DMeasurement {
    r: DMatrix<f64>,
}

impl Measurement for Position2DMeasurement {
    fn get_measurement_layout(&self) -> Vec<StateVariable> {
        vec![StateVariable::Px(FrameId::World), StateVariable::Py(FrameId::World)]
    }

    fn get_measurement_vector(&self, data: &MeasurementData) -> Option<DVector<f64>> {
        if let MeasurementData::GpsPosition(v) = data {
            Some(DVector::from_row_slice(&[v[0], v[1]]))
        } else {
            None
        }
    }

    fn predict_measurement(
        &self,
        state: &FrameAwareState,
        message: &MeasurementMessage,
        _tf: &dyn TfProvider,
    ) -> Option<DVector<f64>> {
        if !matches!(&message.data, MeasurementData::GpsPosition(_)) {
            return None;
        }
        Some(DVector::from_row_slice(&[state.vector[0], state.vector[1]]))
    }

    fn calculate_jacobian(&self, state: &FrameAwareState, _tf: &dyn TfProvider) -> DMatrix<f64> {
        let n = state.dim();
        let mut h = DMatrix::zeros(2, n);
        h[(0, 0)] = 1.0;
        h[(1, 1)] = 1.0;
        h
    }

    fn get_r(&self) -> &DMatrix<f64> {
        &self.r
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

const SENSOR: FrameHandle = FrameHandle(1);

fn make_state() -> FrameAwareState {
    let layout = vec![
        StateVariable::Px(FrameId::World),
        StateVariable::Py(FrameId::World),
        StateVariable::Vx(FrameId::World),
        StateVariable::Vy(FrameId::World),
    ];
    let mut state = FrameAwareState::new(layout, 1.0, 0.0);
    state.vector[2] = 1.0; // vx = 1.0 m/s
    state
}

fn make_ekf() -> ExtendedKalmanFilter {
    let state = make_state();
    let q = DMatrix::identity(4, 4) * 0.01;
    let r = DMatrix::identity(2, 2) * 0.1;
    let mut models: HashMap<FrameHandle, Box<dyn Measurement>> = HashMap::new();
    models.insert(SENSOR, Box::new(Position2DMeasurement { r }));
    ExtendedKalmanFilter::new(state, q, Box::new(ConstantVelocity2D), models)
}

fn make_ukf() -> UnscentedKalmanFilter {
    let state = make_state();
    let q = DMatrix::identity(4, 4) * 0.01;
    let r = DMatrix::identity(2, 2) * 0.1;
    let mut models: HashMap<FrameHandle, Box<dyn Measurement>> = HashMap::new();
    models.insert(SENSOR, Box::new(Position2DMeasurement { r }));
    let params = UkfParams { alpha: 1e-3, beta: 2.0, kappa: 0.0 };
    UnscentedKalmanFilter::new(state, q, Box::new(ConstantVelocity2D), models, params)
}

fn gps_message(x: f64, y: f64) -> MeasurementMessage {
    MeasurementMessage {
        agent_handle: FrameHandle::default(),
        sensor_handle: SENSOR,
        timestamp: 0.1,
        data: MeasurementData::GpsPosition(Vector3::new(x, y, 0.0)),
    }
}

// =========================================================================
// == Benchmarks ==
// =========================================================================

fn bench_ekf(c: &mut Criterion) {
    let u = DVector::zeros(0);
    let tf = IdentityTf;
    let ctx_with_tf = FilterContext { tf: Some(&tf) };
    let ctx_no_tf = FilterContext::default();
    let msg = gps_message(1.0, 0.0);

    let mut group = c.benchmark_group("ekf");

    group.bench_function("predict", |b| {
        let mut ekf = make_ekf();
        b.iter(|| ekf.predict(0.1, &u, &ctx_no_tf));
    });

    group.bench_function("update", |b| {
        let mut ekf = make_ekf();
        b.iter(|| ekf.update(&msg, &ctx_with_tf));
    });

    group.finish();
}

fn bench_ukf(c: &mut Criterion) {
    let u = DVector::zeros(0);
    let tf = IdentityTf;
    let ctx_with_tf = FilterContext { tf: Some(&tf) };
    let ctx_no_tf = FilterContext::default();
    let msg = gps_message(1.0, 0.0);

    let mut group = c.benchmark_group("ukf");

    group.bench_function("predict", |b| {
        let mut ukf = make_ukf();
        b.iter(|| ukf.predict(0.1, &u, &ctx_no_tf));
    });

    group.bench_function("update", |b| {
        let mut ukf = make_ukf();
        b.iter(|| ukf.update(&msg, &ctx_with_tf));
    });

    group.finish();
}

criterion_group!(benches, bench_ekf, bench_ukf);
criterion_main!(benches);
