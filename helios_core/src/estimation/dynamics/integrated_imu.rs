use std::sync::Arc;

use crate::data::primitives::{Control, FrameHandle, State};
use crate::estimation::dynamics::EstimationDynamics;
use crate::estimation::schema::{SchemaBlock, StateSchema};
use crate::frames::{FrameId, StateVariable};
use crate::manifold::euclidean::EuclideanBlock;
use crate::manifold::quaternion::QuaternionBlock;
use crate::manifold::TangentNoise;
use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};

/// A dynamics model that integrates raw IMU measurements (as control inputs)
/// to propagate a 16-state inertial navigation system (INS) state vector.
///
/// This model correctly handles 3D rotations and estimates IMU biases.
#[derive(Debug, Clone)]
pub struct IntegratedImuModel {
    /// The handle of the agent this model describes.
    pub agent_handle: FrameHandle,
    /// The filter's assumed gravity in the world (ENU) frame, m/s², subtracted
    /// from IMU-integrated acceleration during prediction. This is a model
    /// parameter — what the estimator *believes* gravity to be — not ground
    /// truth; the bias states absorb the residual when it is slightly off.
    ///
    /// A vector rather than a magnitude, so any fixed direction is expressible:
    /// Earth-down `[0, 0, -9.81]`, a tilted local frame, or a different body's
    /// surface. It is a **uniform-field** approximation, valid near a surface
    /// where gravity is effectively constant over the trajectory. It is the
    /// degenerate case of a position-dependent field `g(r)`; an orbital regime
    /// (gravity pointing at the body, magnitude ∝ 1/r²) would replace this
    /// constant with such a function and is deliberately out of scope here.
    pub gravity_world: Vector3<f64>,

    schema: Arc<StateSchema>,
    pos_off: usize,
    vel_off: usize,
    quat_off: usize,
    accel_bias_off: usize,
    gyro_bias_off: usize,
}

/// Process-noise (`Q`) variances for the INS dynamics, one per driven block.
/// Each is a variance — the square of the corresponding IMU noise or bias-
/// instability standard deviation.
#[derive(Debug, Clone, Copy)]
pub struct ImuProcessNoise {
    /// Accelerometer white-noise variance, driving the velocity block.
    pub accel_noise_var: f64,
    /// Gyroscope white-noise variance, driving the orientation block.
    pub gyro_noise_var: f64,
    /// Accelerometer bias-instability variance, driving the accel-bias block.
    pub accel_bias_var: f64,
    /// Gyroscope bias-instability variance, driving the gyro-bias block.
    pub gyro_bias_var: f64,
}

/// Initial-covariance (`P₀`) variances for the INS state, one per block. Each is
/// a variance — the square of a standard deviation in that block's units.
///
/// Every block is stated explicitly; there is no inherited fallback. A block left
/// at a meaningless prior (e.g. a gyro-bias std of 1 rad/s) seeds an error far
/// outside the filter's linear regime, so each value is the caller's to choose
/// per sensor grade.
#[derive(Debug, Clone, Copy)]
pub struct ImuInitialUncertainty {
    /// Position variance (m²).
    pub pos_var: f64,
    /// Velocity variance ((m/s)²).
    pub vel_var: f64,
    /// Orientation variance (rad²), in the 3-DOF tangent.
    pub ori_var: f64,
    /// Accelerometer-bias variance ((m/s²)²).
    pub accel_bias_var: f64,
    /// Gyroscope-bias variance ((rad/s)²).
    pub gyro_bias_var: f64,
}

impl IntegratedImuModel {
    pub fn new(
        agent_handle: FrameHandle,
        gravity_world: Vector3<f64>,
        noise: ImuProcessNoise,
        initial_uncertainty: ImuInitialUncertainty,
    ) -> Self {
        let schema = Arc::new(compose_ins_schema(agent_handle, noise, initial_uncertainty));

        let off = |v: &StateVariable| {
            schema
                .storage_offset_of(v)
                .expect("variable is in the schema we just built")
        };

        let (body, world) = (FrameId::Body(agent_handle), FrameId::World);
        let pos_off = off(&StateVariable::Px(world.clone()));
        let vel_off = off(&StateVariable::Vx(world.clone()));
        let quat_off = off(&StateVariable::Qx(body.clone(), world.clone()));
        let accel_bias_off = off(&StateVariable::Ax(body.clone()));
        let gyro_bias_off = off(&StateVariable::Wx(body.clone()));

        Self {
            agent_handle,
            gravity_world,
            schema,
            pos_off,
            vel_off,
            quat_off,
            accel_bias_off,
            gyro_bias_off,
        }
    }
}

/// Returns the standard 16-dimensional state vector layout used for high-fidelity
/// inertial navigation system (INS) filters.
///
/// The state is composed of:
/// - Position (3) in World Frame
/// - Velocity (3) in World Frame
/// - Orientation (4, Quaternion) from World to Body Frame
/// - Accelerometer Bias (3) in Body Frame
/// - Gyroscope Bias (3) in Body Frame
///
/// # Arguments
/// * `agent_handle`: The unique handle for the agent (body) whose state is being defined.
pub fn ins_state_layout(agent_handle: FrameHandle) -> Vec<StateVariable> {
    let body = FrameId::Body(agent_handle);
    let world = FrameId::World;

    let mut layout = position_vars(&world);
    layout.extend(velocity_vars(&world));
    layout.extend(orientation_vars(&body, &world));
    layout.extend(accel_bias_vars(&body));
    layout.extend(gyro_bias_vars(&body));
    layout
}

fn compose_ins_schema(
    agent_handle: FrameHandle,
    noise: ImuProcessNoise,
    initial: ImuInitialUncertainty,
) -> StateSchema {
    let body = FrameId::Body(agent_handle);
    let world = FrameId::World;

    let noise_block = |var: f64, n: usize| {
        TangentNoise::from_variances(DVector::from_element(n, var))
            .expect("positive variance is positive-definite")
    };
    let p0 = |var: f64, n: usize| DMatrix::identity(n, n) * var;

    let blocks = vec![
        // 1. Position (World) — no process noise; P₀ from config.
        SchemaBlock {
            block: Arc::new(EuclideanBlock::without_noise(
                DVector::zeros(3),
                p0(initial.pos_var, 3),
            )),
            variables: position_vars(&world),
            sensor: None,
        },
        // 2. Velocity (World) — Q from accel white noise.
        SchemaBlock {
            block: Arc::new(EuclideanBlock::new(
                noise_block(noise.accel_noise_var, 3),
                DVector::zeros(3),
                p0(initial.vel_var, 3),
            )),
            variables: velocity_vars(&world),
            sensor: None,
        },
        // 3. Orientation (Body from World) — 4/3 quaternion
        //    block. Identity quaternion is [x, y, z, w] = [0, 0, 0, 1].
        SchemaBlock {
            block: Arc::new(QuaternionBlock::new(
                noise_block(noise.gyro_noise_var, 3),
                DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
                p0(initial.ori_var, 3),
            )),
            variables: orientation_vars(&body, &world),
            sensor: None,
        },
        // 4. Accel bias (Body) — Q from bias instability.
        SchemaBlock {
            block: Arc::new(EuclideanBlock::new(
                noise_block(noise.accel_bias_var, 3),
                DVector::zeros(3),
                p0(initial.accel_bias_var, 3),
            )),
            variables: accel_bias_vars(&body),
            sensor: None,
        },
        // 5. Gyro bias (Body) — Q from bias instability.
        SchemaBlock {
            block: Arc::new(EuclideanBlock::new(
                noise_block(noise.gyro_bias_var, 3),
                DVector::zeros(3),
                p0(initial.gyro_bias_var, 3),
            )),
            variables: gyro_bias_vars(&body),
            sensor: None,
        },
    ];

    StateSchema::compose(blocks)
}

fn position_vars(world: &FrameId) -> Vec<StateVariable> {
    vec![
        StateVariable::Px(world.clone()),
        StateVariable::Py(world.clone()),
        StateVariable::Pz(world.clone()),
    ]
}

fn velocity_vars(world: &FrameId) -> Vec<StateVariable> {
    vec![
        StateVariable::Vx(world.clone()),
        StateVariable::Vy(world.clone()),
        StateVariable::Vz(world.clone()),
    ]
}

fn orientation_vars(body: &FrameId, world: &FrameId) -> Vec<StateVariable> {
    vec![
        StateVariable::Qx(body.clone(), world.clone()),
        StateVariable::Qy(body.clone(), world.clone()),
        StateVariable::Qz(body.clone(), world.clone()),
        StateVariable::Qw(body.clone(), world.clone()),
    ]
}

fn accel_bias_vars(body: &FrameId) -> Vec<StateVariable> {
    vec![
        StateVariable::Ax(body.clone()),
        StateVariable::Ay(body.clone()),
        StateVariable::Az(body.clone()),
    ]
}

fn gyro_bias_vars(body: &FrameId) -> Vec<StateVariable> {
    vec![
        StateVariable::Wx(body.clone()),
        StateVariable::Wy(body.clone()),
        StateVariable::Wz(body.clone()),
    ]
}

impl EstimationDynamics for IntegratedImuModel {
    /// The control input `u` is a 6D vector: [ax, ay, az, wx, wy, wz] from the IMU.
    fn get_control_dim(&self) -> usize {
        6
    }

    fn schema(&self) -> Arc<StateSchema> {
        Arc::clone(&self.schema)
    }

    fn derivatives(&self, x: &State, u: &Control, _t: f64) -> State {
        let mut x_dot = DVector::zeros(x.nrows());

        // --- 1. Extract state variables and inputs  ---
        let q = self.quat_off;
        let velocity_world = x.fixed_rows::<3>(self.vel_off);
        let orientation_quat = UnitQuaternion::new_normalize(
            Quaternion::new(x[q + 3], x[q], x[q + 1], x[q + 2]), // w=Qw, i=Qx, j=Qy, k=Qz
        );
        let accel_bias = x.fixed_rows::<3>(self.accel_bias_off);
        let gyro_bias = x.fixed_rows::<3>(self.gyro_bias_off);

        let raw_accel_measurement = u.fixed_rows::<3>(0);
        let raw_gyro_measurement = u.fixed_rows::<3>(3);

        // --- 2. Correct the raw measurements  ---
        let corrected_accel_body = raw_accel_measurement - accel_bias;
        let corrected_gyro_body = raw_gyro_measurement - gyro_bias;

        // --- 3. Position and Velocity Derivatives  ---
        x_dot
            .fixed_rows_mut::<3>(self.pos_off)
            .copy_from(&velocity_world);
        let accel_world = orientation_quat * corrected_accel_body;
        x_dot
            .fixed_rows_mut::<3>(self.vel_off)
            .copy_from(&(accel_world + self.gravity_world));

        // --- 4. Orientation Derivative ---
        // Create the pure quaternion for the angular velocity.
        let omega_quat = nalgebra::Quaternion::from_imag(corrected_gyro_body);

        // Perform the quaternion multiplication first: `q * ω_q`.
        // The result is a `Quaternion<f64>`.
        let q_times_omega = *orientation_quat * omega_quat;

        // Now, scale the resulting quaternion by 0.5.
        let q_dot_quat = q_times_omega * 0.5;

        // Finally, extract the coordinates of the final result for the state vector derivative.
        x_dot
            .fixed_rows_mut::<4>(self.quat_off)
            .copy_from(&q_dot_quat.coords); // q_dot_quat.coords is [x, y, z, w]

        // --- 5. Bias Derivatives (this part is correct) ---
        // Derivatives remain zero, handled by process noise Q.

        x_dot
    }
}

// NOTE: The `propagate` method from the trait uses the above `derivatives`
// with an integrator like RK4, so it does not need to be reimplemented.

#[cfg(test)]
mod tests {
    //! Tests for [`IntegratedImuModel`].
    //!
    //! Properties validated:
    //! - Schema: 16 storage / 15 tangent dims, each block at its INS offset, and
    //!   Q / P₀ placed per config per block (position carries no process noise).
    //! - Derivatives: position rate = velocity, gravity correctly subtracts from
    //!   IMU acceleration, accel/gyro biases subtract from raw measurements.
    //! - Jacobian: correct 16×16 / 16×6 shape; velocity-to-position coupling ≈ 1.
    //! - Propagation: RK4 integration with gravity-compensating IMU keeps agent
    //!   stationary (position and velocity remain near zero).

    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::utils::integrators::RK4;
    use nalgebra::DVector;

    const AGENT: FrameHandle = FrameHandle(1);
    const G: f64 = 9.81;

    fn make_model() -> IntegratedImuModel {
        // Noise/uncertainty values are arbitrary here — these tests exercise the
        // derivatives, Jacobian shape, and propagation, none of which read Q or P₀.
        IntegratedImuModel::new(
            AGENT,
            Vector3::new(0.0, 0.0, -G),
            ImuProcessNoise {
                accel_noise_var: 0.1_f64.powi(2),
                gyro_noise_var: 0.01_f64.powi(2),
                accel_bias_var: 0.001_f64.powi(2),
                gyro_bias_var: 0.0001_f64.powi(2),
            },
            ImuInitialUncertainty {
                pos_var: 1.0,
                vel_var: 1.0,
                ori_var: 1.0,
                accel_bias_var: 1.0,
                gyro_bias_var: 1.0,
            },
        )
    }

    /// Builds a 16-element state vector with the identity quaternion at indices 6-9.
    ///
    /// Index layout mirrors [`ins_state_layout`]: 0-2 position, 3-5 velocity,
    /// 6-9 quaternion (Qx, Qy, Qz, Qw), 10-12 accel bias, 13-15 gyro bias.
    fn identity_state() -> DVector<f64> {
        let mut x = DVector::zeros(ins_state_layout(AGENT).len());
        x[9] = 1.0; // Qw = 1 → identity quaternion
        x
    }

    /// Returns a 6-element control vector with `az = g` to exactly counteract gravity.
    fn gravity_compensating_imu(g: f64) -> DVector<f64> {
        let mut u = DVector::zeros(6);
        u[2] = g;
        u
    }

    // ── Schema ───────────────────────────────────────────────────────────────

    #[test]
    fn schema_has_sixteen_storage_and_fifteen_tangent_dims() {
        // The orientation is a proper SO(3) error-state block: 4 stored quaternion
        // components but only 3 tangent DOF, so storage (16) outruns tangent (15).
        // Every covariance-space quantity lives in the 15-D tangent.
        let schema = make_model().schema();
        assert_eq!(schema.storage_dim(), 16);
        assert_eq!(schema.tangent_dim(), 15);
    }

    #[test]
    fn schema_layout_matches_ins_state_layout() {
        // Single-source guard: the composed schema's block ordering and the
        // standalone `ins_state_layout` are built from the same per-block groups
        // in the same order, so neither can drift from the other unnoticed.
        let schema = make_model().schema();
        assert_eq!(schema.layout(), ins_state_layout(AGENT).as_slice());
    }

    #[test]
    fn schema_places_each_block_at_its_ins_offset() {
        let schema = make_model().schema();
        let body = FrameId::Body(AGENT);
        let world = FrameId::World;

        let off = |v: &StateVariable| schema.storage_offset_of(v).unwrap();
        assert_eq!(off(&StateVariable::Px(world.clone())), 0);
        assert_eq!(off(&StateVariable::Vx(world.clone())), 3);
        assert_eq!(off(&StateVariable::Qx(body.clone(), world.clone())), 6);
        assert_eq!(off(&StateVariable::Ax(body.clone())), 10);
        assert_eq!(off(&StateVariable::Wx(body.clone())), 13);
    }

    #[test]
    fn schema_tangent_offsets_shift_below_the_orientation_block() {
        // The tangent twin of the storage-offset test, and the crux of the SO(3)
        // flip: storage and tangent agree up to and including orientation (both at
        // 6), then diverge — orientation spends 3 tangent rows against 4 stored, so
        // every block after it sits one row lower in tangent than in storage.
        let schema = make_model().schema();
        let body = FrameId::Body(AGENT);
        let world = FrameId::World;

        let off = |v: &StateVariable| schema.tangent_offset_of(v).unwrap();
        assert_eq!(off(&StateVariable::Px(world.clone())), 0); // agrees with storage
        assert_eq!(off(&StateVariable::Vx(world.clone())), 3); // agrees with storage
        assert_eq!(off(&StateVariable::Qx(body.clone(), world.clone())), 6); // last agreement
        assert_eq!(off(&StateVariable::Ax(body.clone())), 9); // storage 10 → tangent 9
        assert_eq!(off(&StateVariable::Wx(body.clone())), 12); // storage 13 → tangent 12
    }

    #[test]
    fn schema_process_noise_matches_config_per_block() {
        // Distinct variances so a mis-placed block can't hide behind a shared
        // value. Position carries no process noise; every other block's Q is the
        // variance passed to `new`.
        let model = IntegratedImuModel::new(
            AGENT,
            Vector3::new(0.0, 0.0, -G),
            ImuProcessNoise {
                accel_noise_var: 2.0, // → velocity block
                gyro_noise_var: 3.0,  // → orientation block
                accel_bias_var: 4.0,  // → accel-bias block
                gyro_bias_var: 5.0,   // → gyro-bias block
            },
            ImuInitialUncertainty {
                pos_var: 1.0, // P₀ only, irrelevant to this Q test
                vel_var: 1.0,
                ori_var: 1.0,
                accel_bias_var: 1.0,
                gyro_bias_var: 1.0,
            },
        );
        let schema = model.schema();
        let q = schema.process_noise();

        // Q is tangent-sized (15×15): the orientation block is 3 wide here, not 4,
        // so every block after it sits one row lower than its storage offset.
        let expected = [
            (0..3, 0.0),   // position: no process noise
            (3..6, 2.0),   // velocity
            (6..9, 3.0),   // orientation (3-wide tangent)
            (9..12, 4.0),  // accel bias
            (12..15, 5.0), // gyro bias
        ];
        for (range, value) in expected {
            for i in range {
                assert_eq!(q[(i, i)], value, "Q diagonal at {i}");
            }
        }
    }

    #[test]
    fn schema_initial_covariance_matches_config_per_block() {
        // Every block's P₀ is independently config-driven — there is no inherited
        // fallback. Distinct values per block so a misplaced or transposed P₀ can't
        // hide behind a shared number.
        let model = IntegratedImuModel::new(
            AGENT,
            Vector3::new(0.0, 0.0, -G),
            ImuProcessNoise {
                accel_noise_var: 0.1,
                gyro_noise_var: 0.1,
                accel_bias_var: 0.1,
                gyro_bias_var: 0.1, // noise values irrelevant to P₀
            },
            ImuInitialUncertainty {
                pos_var: 7.0,        // → position block
                vel_var: 6.0,        // → velocity block
                ori_var: 9.0,        // → orientation block
                accel_bias_var: 4.0, // → accel-bias block
                gyro_bias_var: 5.0,  // → gyro-bias block
            },
        );
        let schema = model.schema();
        let p0 = schema.initial_covariance();

        // P₀ is tangent-sized (15×15): orientation occupies 3 tangent rows (6..9),
        // so every block after it sits one row lower than its storage offset.
        let expected = [
            (0..3, 7.0),   // position
            (3..6, 6.0),   // velocity
            (6..9, 9.0),   // orientation (3-wide tangent)
            (9..12, 4.0),  // accel bias
            (12..15, 5.0), // gyro bias
        ];
        for (range, value) in expected {
            for i in range {
                assert_eq!(p0[(i, i)], value, "P₀ diagonal at {i}");
            }
        }
    }

    // ── Derivatives ──────────────────────────────────────────────────────────

    #[test]
    fn derivatives_position_rate_equals_velocity() {
        // With world velocity [2, 3, 1] and identity orientation, x_dot[0..3] = [2, 3, 1].
        let model = make_model();
        let mut x = identity_state();
        x[3] = 2.0; // vx
        x[4] = 3.0; // vy
        x[5] = 1.0; // vz
        let u = gravity_compensating_imu(G); // zero net acceleration
        let xdot = model.derivatives(&x, &u, 0.0);

        assert!((xdot[0] - 2.0).abs() < 1e-9, "px_dot should equal vx");
        assert!((xdot[1] - 3.0).abs() < 1e-9, "py_dot should equal vy");
        assert!((xdot[2] - 1.0).abs() < 1e-9, "pz_dot should equal vz");
    }

    #[test]
    fn derivatives_free_fall_gives_downward_acceleration() {
        // Zero IMU input with identity orientation: gravity alone drives velocity
        // downward at ~9.81 m/s². Horizontal components must stay zero.
        let model = make_model();
        let x = identity_state();
        let u = DVector::zeros(6);
        let xdot = model.derivatives(&x, &u, 0.0);

        assert!(
            xdot[5] < -9.0,
            "vz_dot should be strongly negative (gravity pull), got {}",
            xdot[5]
        );
        assert!(xdot[3].abs() < 1e-9, "vx_dot must be zero in free fall");
        assert!(xdot[4].abs() < 1e-9, "vy_dot must be zero in free fall");
    }

    #[test]
    fn derivatives_gravity_compensation_gives_zero_velocity_change() {
        // IMU reports [0, 0, 9.81] — exactly cancelling gravity in the world frame.
        // With identity orientation, velocity derivative must be ≈ zero.
        let model = make_model();
        let x = identity_state();
        let u = gravity_compensating_imu(G);
        let xdot = model.derivatives(&x, &u, 0.0);

        assert!(
            xdot[3].abs() < 1e-9,
            "vx_dot must be 0 with gravity compensation"
        );
        assert!(
            xdot[4].abs() < 1e-9,
            "vy_dot must be 0 with gravity compensation"
        );
        assert!(
            xdot[5].abs() < 1e-9,
            "vz_dot must be 0 with gravity compensation, got {}",
            xdot[5]
        );
    }

    #[test]
    fn derivatives_accel_bias_subtracts_from_raw_measurement() {
        // Accel bias z = 1.0 means corrected = raw - bias.
        // With raw_az = 9.81 and bias_az = 1.0: corrected = 8.81.
        // velocity_z derivative = 8.81 + (-9.81) = -1.0.
        let model = make_model();
        let mut x = identity_state();
        x[12] = 1.0; // accel_bias_z (index 12 in standard INS layout)
        let u = gravity_compensating_imu(G);
        let xdot = model.derivatives(&x, &u, 0.0);

        assert!(
            (xdot[5] - (-1.0)).abs() < 1e-6,
            "vz_dot with 1.0 z-bias should be -1.0, got {}",
            xdot[5]
        );
    }

    // ── Jacobian ─────────────────────────────────────────────────────────────

    #[test]
    fn jacobian_has_correct_dimensions() {
        // A is the state Jacobian (16×16), B is the control Jacobian (16×6).
        let model = make_model();
        let x = identity_state();
        let u = DVector::zeros(6);
        let (a_jac, b_jac) = model.jacobian(&x, &u, 0.0);

        let dim = ins_state_layout(AGENT).len();
        assert_eq!(a_jac.nrows(), dim, "A rows");
        assert_eq!(a_jac.ncols(), dim, "A cols");
        assert_eq!(b_jac.nrows(), dim, "B rows");
        assert_eq!(b_jac.ncols(), 6, "B cols = control_dim (ax,ay,az,wx,wy,wz)");
    }

    #[test]
    fn jacobian_velocity_to_position_coupling_is_unity() {
        // A(i, i+3) for i in 0..3 encodes d(pos)/d(vel) = 1.
        // This is the clearest structural property of the INS dynamics.
        let model = make_model();
        let x = identity_state();
        let u = DVector::zeros(6);
        let (a_jac, _) = model.jacobian(&x, &u, 0.0);

        for (pos_idx, vel_idx) in [(0, 3), (1, 4), (2, 5)] {
            assert!(
                (a_jac[(pos_idx, vel_idx)] - 1.0).abs() < 1e-4,
                "A({pos_idx},{vel_idx}) = ∂pos_dot/∂vel ≈ 1.0, got {}",
                a_jac[(pos_idx, vel_idx)]
            );
        }
    }

    // ── Propagation ──────────────────────────────────────────────────────────

    #[test]
    fn propagate_with_gravity_compensation_keeps_agent_stationary() {
        // At rest with gravity-compensating IMU input, RK4 integration over 0.1 s
        // must leave position and velocity near zero.
        let model = make_model();
        let x = identity_state();
        let u = gravity_compensating_imu(G);
        let x_next = model.propagate(&x, &u, 0.0, 0.1, &RK4);

        for idx in 0..6 {
            assert!(
                x_next[idx].abs() < 1e-9,
                "state[{idx}] should remain 0 at rest, got {}",
                x_next[idx]
            );
        }
    }
}
