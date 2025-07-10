// helios_core/src/prelude.rs

// --- Core Abstractions (The main contracts of the library) ---
pub use crate::abstractions::{Dynamics, Measurement, TfProvider};

// --- Core Data Structures (The "nouns" of the library) ---
pub use crate::context::KinematicContext;
pub use crate::frames::{FrameAwareState, FrameHandle, FrameId, MeasurementEvent, StateVariable};

// --- Estimation Algorithms ---
pub use crate::estimation::ekf::{ekf_predict, ekf_update, EkfMeasurementParams, EkfUpdateParams};

// --- Concrete Model Implementations (Export common ones for convenience) ---
pub use crate::models::ackermann::AckermannKinematics;
pub use crate::models::generic::ConstantAccelerationModel;
pub use crate::models::imu::Imu6DofModel;
