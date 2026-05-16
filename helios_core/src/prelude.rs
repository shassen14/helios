// helios_core/src/prelude.rs

// --- Core Abstractions (The main contracts of the library) ---
pub use crate::data::messages::{MeasurementData, MeasurementMessage, ModuleInput, Odometry};
pub use crate::data::primitives::{FrameHandle, MonotonicTime, TfProvider, TrajectoryPoint};
pub use crate::data::sensor::{SensorPayload, SensorReading};
pub use crate::data::sensor::{
    AngularVelocity3D, DepthImage, GpsPosition, GpsVelocity, LinearAcceleration3D, MagneticField3D,
    PointCloud2D, PointCloud3D, RgbImage,
};

// --- Core Data Structures (The "nouns" of the library) ---
pub use crate::frames::{FrameAwareState, FrameId, StateVariable};

// --- Control Abstractions ---
pub use crate::control::dynamics::ControlDynamics;
pub use crate::control::{ControlInputs, ControlOutput, Controller};

// --- Estimation ---
pub use crate::estimation::dynamics::EstimationDynamics;
pub use crate::estimation::measurement::MeasurementModel;
pub use crate::estimation::{EstimatorInputs, GaussianStateEstimator};

// --- Planning ---
pub use crate::planning::types::{Path, PlannerGoal, PlannerResult, PlannerStatus};
pub use crate::planning::SearchPlanner;
pub use crate::planning::SearchPlannerInputs;

// Path Following
pub use crate::path_following::{PathFollower, PathFollowerInputs, PathFollowerResult};

// --- Concrete Model Implementations (Export common ones for convenience) ---
pub use crate::estimation::measurement::{accelerometer, gps, gyroscope, magnetometer};
