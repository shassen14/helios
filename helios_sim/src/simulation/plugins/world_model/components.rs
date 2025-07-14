// helios_sim/src/plugins/world_model/components.rs

use bevy::prelude::Component;
use helios_core::{estimation::StateEstimator, mapping::Mapper, slam::SlamSystem};

/// The single, unified component that holds the autonomy logic for an agent.
#[derive(Component)]
pub enum WorldModelComponent {
    /// Configuration for separate localization and mapping modules.
    Separate {
        estimator: Box<dyn StateEstimator>,
        // For now, we'll use a placeholder `None` mapper.
        mapper: Box<dyn Mapper>,
    },
    /// Configuration for a unified SLAM system.
    CombinedSlam { system: Box<dyn SlamSystem> },
}
