// helios_core/src/prelude.rs

// --- Core Abstractions (The main contracts of the library) ---
pub use crate::messages::{MeasurementData, MeasurementMessage, ModuleInput};
pub use crate::models::estimation::dynamics::EstimationDynamics;
pub use crate::models::estimation::measurement::Measurement;
pub use crate::sensor_data::{
    AngularVelocity3D, DepthImage, GpsPosition, GpsVelocity, LinearAcceleration3D, MagneticField3D,
    PointCloud2D, PointCloud3D, RgbImage,
};
pub use crate::types::{FrameHandle, MonotonicTime, TfProvider, TrajectoryPoint};

// --- Core Data Structures (The "nouns" of the library) ---
pub use crate::frames::{FrameAwareState, FrameId, StateVariable};

// --- Control Abstractions ---
pub use crate::control::{ControlContext, ControlOutput, Controller};
pub use crate::models::controls::ControlDynamics;

// --- Planning & Tracking ---
pub use crate::planning::context::PlannerContext;
pub use crate::planning::types::{Path, PlannerGoal, PlannerResult, PlannerStatus};
pub use crate::planning::Planner;
pub use crate::tracking::{Track, Tracker};

// Path Following
pub use crate::path_following::{PathFollower, PathFollowerResult};

// --- Concrete Model Implementations (Export common ones for convenience) ---
pub use crate::models::estimation::measurement::{accelerometer, gps, gyroscope, magnetometer};
