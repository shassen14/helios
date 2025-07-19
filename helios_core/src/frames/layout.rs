// helios_core/src/frames/layout.rs
use crate::frames::{FrameId, StateVariable};
use crate::types::FrameHandle;

/// The standard dimension of the 16-state INS state vector.
pub const STANDARD_INS_STATE_DIM: usize = 16;

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
pub fn standard_ins_state_layout(agent_handle: FrameHandle) -> Vec<StateVariable> {
    let body_frame = FrameId::Body(agent_handle);
    let world_frame = FrameId::World;

    vec![
        // --- Position (World Frame) --- indices 0-2
        StateVariable::Px(world_frame.clone()),
        StateVariable::Py(world_frame.clone()),
        StateVariable::Pz(world_frame.clone()),
        // --- Velocity (World Frame) --- indices 3-5
        StateVariable::Vx(world_frame.clone()),
        StateVariable::Vy(world_frame.clone()),
        StateVariable::Vz(world_frame.clone()),
        // --- Orientation (Quaternion, Body from World) --- indices 6-9
        // Storing as [x, y, z, w] for contiguous memory, but nalgebra uses (w, i, j, k)
        StateVariable::Qx(body_frame.clone(), world_frame.clone()),
        StateVariable::Qy(body_frame.clone(), world_frame.clone()),
        StateVariable::Qz(body_frame.clone(), world_frame.clone()),
        StateVariable::Qw(body_frame.clone(), world_frame.clone()),
        // --- Accelerometer Bias (Body Frame) --- indices 10-12
        StateVariable::Ax(body_frame.clone()), // Represents bias_ax
        StateVariable::Ay(body_frame.clone()), // Represents bias_ay
        StateVariable::Az(body_frame.clone()), // Represents bias_az
        // --- Gyroscope Bias (Body Frame) --- indices 13-15
        StateVariable::Wx(body_frame.clone()), // Represents bias_wx
        StateVariable::Wy(body_frame.clone()), // Represents bias_wy
        StateVariable::Wz(body_frame.clone()), // Represents bias_wz
    ]
}
