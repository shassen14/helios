use bevy::prelude::Component;

/// A marker component that enables debug visualizations for the entity it's attached to.
///
/// Systems like `draw_lidar_rays` will query for entities with this component
/// to decide whether to draw their gizmos.
#[derive(Component)]
pub struct ShowDebugGizmos;
