// helios_sim/src/simulation/plugins/isolation/mock_path.rs
//
// MockPathInjectorPlugin stub.
// Path injection will be restored once PathFollowerNode is wired into AutonomyPipeline.

use bevy::prelude::*;

pub struct MockPathInjectorPlugin;

impl Plugin for MockPathInjectorPlugin {
    fn build(&self, _app: &mut App) {
        // Stub: path injection deferred to a later step.
    }
}
