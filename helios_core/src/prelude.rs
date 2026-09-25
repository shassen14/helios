// --- Core Abstractions (The main contracts of the library) ---
pub use crate::interchange::measurement::cloud::{
    AttributeColumns, LidarAttrs, LidarColumns, LidarColumnsBuilder, PointCloud, PointCloudBuilder,
};
pub use crate::interchange::measurement::envelope::SensorReading;
pub use crate::interchange::measurement::range_field::{
    DirectionModel, RangeField, RangeFieldBuildError, RangeFieldBuilder, ScanTiming,
    SphericalAngular,
};
pub use crate::interchange::motion::{Odometry, Twist};
pub use crate::spatial::tf::TfProvider;
pub use crate::spatial::primitives::{AgentId, MonotonicDuration, MonotonicTime};
pub use crate::interchange::measurement::sensor::SensorPayload;
pub use crate::interchange::measurement::sensor::{
    Acceleration, AngularRate, DepthImage, GpsPosition, GpsVelocity, MagneticField, RgbImage,
};

// --- Core Data Structures (The "nouns" of the library) ---
pub use crate::spatial::{FrameAwareState, FrameId, StateVariable};

// --- Control Abstractions ---
pub use crate::control::dynamics::ControlDynamics;
pub use crate::control::{ControlInputs, Controller};

// --- Estimation ---
pub use crate::estimation::dynamics::EstimationDynamics;
pub use crate::estimation::measurement::MeasurementModel;
pub use crate::estimation::{EstimatorInputs, GaussianStateEstimator};

// --- Planning ---
pub use crate::interchange::path::{Path, PlannerGoal};
pub use crate::planning::types::{PlannerResult, PlannerStatus};
pub use crate::planning::SearchPlanner;
pub use crate::planning::SearchPlannerInputs;

// Path Following
pub use crate::following::{PathFollower, PathFollowerInputs, PathFollowerResult};
