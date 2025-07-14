// helios_core/src/frames.rs

use crate::types::FrameHandle;
use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};
use std::hash::Hash;

/// A unique, hashable identifier for any coordinate frame in the simulation.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum FrameId {
    /// The global ENU simulation frame. The ultimate source of truth.
    World,
    /// The origin of a rigid body, where dynamics are typically calculated.
    /// Identified by the agent's unique FrameHandle.
    Body(FrameHandle),
    /// The specific origin of a sensor component.
    /// Identified by the sensor's own unique FrameHandle.
    Sensor(FrameHandle),
}

/// An enum that defines every possible variable that can exist in a state vector.
/// The FrameId specifies which frame the variable is expressed in.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum StateVariable {
    // --- Cartesian Position ---
    Px(FrameId),
    Py(FrameId),
    Pz(FrameId),
    // --- Cartesian Velocity ---
    Vx(FrameId),
    Vy(FrameId),
    Vz(FrameId),
    // --- Cartesian Acceleration ---
    Ax(FrameId),
    Ay(FrameId),
    Az(FrameId),
    // --- Orientation (as a quaternion) ---
    // Represents the rotation FROM the first frame TO the second frame.
    // e.g., Qx(from: Body, to: World)
    Qx(FrameId, FrameId),
    Qy(FrameId, FrameId),
    Qz(FrameId, FrameId),
    Qw(FrameId, FrameId),
    // --- Angular Velocity ---
    Wx(FrameId),
    Wy(FrameId),
    Wz(FrameId),
    // --- Angular Acceleration --
    Alphax(FrameId),
    Alphay(FrameId),
    Alphaz(FrameId),
    // --- Magnetic Field ---
    MagX(FrameId),
    MagY(FrameId),
    MagZ(FrameId),
}

/// The "smart" state object used by filters. It bundles the state vector
/// with its schema (the layout), covariance, and timestamp.
#[derive(Debug, Clone)]
pub struct FrameAwareState {
    /// The ordered "schema" of the state vector.
    pub layout: Vec<StateVariable>,
    /// The actual numerical data vector `x`.
    pub vector: DVector<f64>,
    /// The covariance matrix `P`.
    pub covariance: DMatrix<f64>,
    /// The timestamp of the last update.
    pub last_update_timestamp: f64,
}

impl FrameAwareState {
    /// Creates a new state with a given layout, initializing the vector to zero
    /// (with a valid identity quaternion) and the covariance to a scaled identity matrix.
    pub fn new(layout: Vec<StateVariable>, initial_covariance_val: f64, timestamp: f64) -> Self {
        let dim = layout.len();
        let mut vector = DVector::zeros(dim);

        // Find the quaternion part of the state and initialize it to identity (0,0,0,1).
        // This is critical to prevent NaN values from an invalid zero quaternion.
        for (i, var) in layout.iter().enumerate() {
            // Using `matches!` is a clean way to check the enum variant without caring about its contents.
            if matches!(var, StateVariable::Qw(_, _)) {
                vector[i] = 1.0; // Set the 'w' component to 1.
                                 // Note: This assumes only one quaternion in the state. For more complex
                                 // states, this logic would need to be more robust.
                break;
            }
        }

        Self {
            layout,
            vector,
            covariance: DMatrix::identity(dim, dim) * initial_covariance_val,
            last_update_timestamp: timestamp,
        }
    }

    /// Returns the dimension (number of rows) of the state vector.
    pub fn dim(&self) -> usize {
        self.layout.len()
    }

    /// Finds the index of a specific `StateVariable` in the layout.
    pub fn find_idx(&self, var: &StateVariable) -> Option<usize> {
        self.layout.iter().position(|v| v == var)
    }

    /// Extracts a 3D vector from the state vector based on a starting `StateVariable`.
    ///
    /// For example, providing `StateVariable::Px(frame_id)` will attempt to find `Px`, `Py`,
    /// and `Pz` for that frame and return them as a `Vector3`.
    ///
    /// # Arguments
    /// * `start_variable`: The `StateVariable` representing the X-component of the desired vector.
    ///
    /// # Returns
    /// * `Some(Vector3<f64>)` if all three components (X, Y, Z) are found contiguously.
    /// * `None` if any of the components are missing or not in order.
    pub fn get_vector3(&self, start_variable: &StateVariable) -> Option<Vector3<f64>> {
        // Determine the expected Y and Z variables based on the provided X variable.
        let (expected_y, expected_z) = match start_variable {
            StateVariable::Px(id) => (StateVariable::Py(id.clone()), StateVariable::Pz(id.clone())),
            StateVariable::Vx(id) => (StateVariable::Vy(id.clone()), StateVariable::Vz(id.clone())),
            StateVariable::Ax(id) => (StateVariable::Ay(id.clone()), StateVariable::Az(id.clone())),
            StateVariable::Wx(id) => (StateVariable::Wy(id.clone()), StateVariable::Wz(id.clone())),
            StateVariable::Alphax(id) => (
                StateVariable::Alphay(id.clone()),
                StateVariable::Alphaz(id.clone()),
            ),
            StateVariable::MagX(id) => (
                StateVariable::MagY(id.clone()),
                StateVariable::MagZ(id.clone()),
            ),
            _ => return None, // Not a valid start of a 3D vector
        };

        // Find the starting index.
        let start_idx = self.find_idx(start_variable)?;

        // Check if the next two elements match the expected Y and Z variables.
        // Also ensures we don't read past the end of the layout vector.
        if self.layout.get(start_idx + 1) == Some(&expected_y)
            && self.layout.get(start_idx + 2) == Some(&expected_z)
        {
            // If they match, we can safely slice the state vector.
            // `fixed_rows` provides a view into the DVector without copying.
            let vec_slice = self.vector.fixed_rows::<3>(start_idx);
            Some(vec_slice.into()) // Convert the slice into an owned Vector3
        } else {
            None // The state layout is not as expected.
        }
    }

    /// Extracts the orientation quaternion from the state vector.
    ///
    /// It searches for the `Qw` component and assumes that `Qx`, `Qy`, and `Qz`
    /// precede it in the state layout.
    ///
    /// # Returns
    /// * `Some(UnitQuaternion<f64>)` if a valid quaternion is found.
    /// * `None` if the quaternion components are not found contiguously.
    pub fn get_orientation(&self) -> Option<UnitQuaternion<f64>> {
        // We can find any of the quaternion components to start, but searching for
        // Qw is often convenient as it's the last one.
        let mut qx_idx = None;
        let mut qy_idx = None;
        let mut qz_idx = None;
        let mut qw_idx = None;

        // Find the indices of all four components.
        for (i, var) in self.layout.iter().enumerate() {
            match var {
                StateVariable::Qx(_, _) => qx_idx = Some(i),
                StateVariable::Qy(_, _) => qy_idx = Some(i),
                StateVariable::Qz(_, _) => qz_idx = Some(i),
                StateVariable::Qw(_, _) => qw_idx = Some(i),
                _ => {}
            }
        }

        // Ensure all four were found.
        let (qx_idx, qy_idx, qz_idx, qw_idx) = (qx_idx?, qy_idx?, qz_idx?, qw_idx?);

        // A robust check to ensure they are contiguous (e.g., [..., Qx, Qy, Qz, Qw, ...])
        // This makes the layout more flexible than assuming a fixed order.
        if qy_idx == qx_idx + 1 && qz_idx == qy_idx + 1 && qw_idx == qz_idx + 1 {
            // All components are contiguous, extract them.
            // Note: nalgebra's `Quaternion` constructor is `(w, i, j, k)`.
            // The `UnitQuaternion` constructor from a `Quaternion` handles normalization.
            let quat = UnitQuaternion::from_quaternion(Quaternion::new(
                self.vector[qw_idx], // w
                self.vector[qx_idx], // x
                self.vector[qy_idx], // y
                self.vector[qz_idx], // z
            ));
            Some(quat)
        } else {
            // The quaternion components in the state layout are not contiguous.
            None
        }
    }
}
