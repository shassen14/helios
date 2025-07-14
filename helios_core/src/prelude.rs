// helios_core/src/prelude.rs

// --- Core Abstractions (The main contracts of the library) ---
pub use crate::messages::{MeasurementData, MeasurementMessage, ModuleInput};
pub use crate::models::dynamics::Dynamics;
pub use crate::models::measurement::Measurement;
pub use crate::types::{FrameHandle, TfProvider};

// --- Core Data Structures (The "nouns" of the library) ---
// pub use crate::context::KinematicContext;
pub use crate::frames::{FrameAwareState, FrameId, StateVariable};

// --- Estimation Algorithms ---
// pub use crate::estimation::ekf::{ekf_predict, ekf_update, EkfMeasurementParams, EkfUpdateParams};

// --- Concrete Model Implementations (Export common ones for convenience) ---
pub use crate::models::dynamics::ackermann::AckermannKinematics;
pub use crate::models::dynamics::generic::ConstantAccelerationModel;
pub use crate::models::measurement::imu::Imu6DofModel;
