// helios_sim/src/simulation/plugins/isolation/mock_map.rs
//
// MockMapInjectorPlugin stub.
// Map injection will be restored once MapperNode is wired into AutonomyPipeline.

use bevy::prelude::*;

pub struct MockMapInjectorPlugin;

impl Plugin for MockMapInjectorPlugin {
    fn build(&self, _app: &mut App) {
        // Stub: map injection deferred to a later step.
    }
}
