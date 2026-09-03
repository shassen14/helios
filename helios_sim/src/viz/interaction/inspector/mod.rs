pub mod gather;
pub mod model;
pub mod render;

use bevy::prelude::*;

use crate::{
    prelude::AppState,
    viz::{
        interaction::inspector::{
            gather::{
                begin_inspection, gather_actuators, gather_controller, gather_estimator,
                gather_identity, gather_pose, gather_reference,
            },
            model::InspectorModel,
            render::{load_inspector_font, render_inspection},
        },
        VizSet,
    },
};

pub struct InspectorPlugin;

impl Plugin for InspectorPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CurrentInspection>();

        // Load the panel font once, only when an asset backend is present. The real
        // host always has one (DefaultPlugins); the bare wiring-test app has none and
        // the renderer falls back to the default face.
        app.add_systems(
            Startup,
            load_inspector_font.run_if(resource_exists::<AssetServer>),
        );

        app.add_systems(
            Update,
            (
                begin_inspection,
                gather_identity,
                gather_pose,
                gather_estimator,
                gather_controller,
                gather_reference,
                gather_actuators,
                render_inspection,
            )
                .chain()
                .in_set(VizSet::Live)
                .run_if(in_state(AppState::Running)),
        );
    }
}

#[derive(Resource, Default)]
pub struct CurrentInspection(pub Option<InspectorModel>);

#[cfg(test)]
mod tests {
    use super::*;

    use crate::viz::interaction::{inspector::model::SubsystemPath, selection::Selected};

    use std::sync::Arc;

    /// Tier 3: the plugin must register `begin → gather_identity → gather_pose →
    /// render` as one ordered chain. Dropping a system from `add_systems` or losing
    /// the `.chain()` still compiles, so drive the whole plugin once against a
    /// selected subject and assert the sections land, in order: identity first
    /// (proving `begin` seeded the shell before the gatherers), then pose. A missing
    /// or reordered gatherer changes this list.
    #[test]
    fn plugin_runs_the_gather_chain_in_order() {
        let mut app = App::new();
        app.add_plugins(bevy::state::app::StatesPlugin);
        app.insert_state(AppState::Running);
        app.init_resource::<Time>();
        app.add_plugins(InspectorPlugin);
        app.world_mut().spawn((
            Name::new("robot_3"),
            Selected,
            GlobalTransform::from_translation(Vec3::new(1.0, 2.0, 3.0)),
        ));

        app.update();

        let model = app
            .world()
            .resource::<CurrentInspection>()
            .0
            .clone()
            .expect("a selection drives the chain to a model");
        let ids: Vec<_> = model.sections.iter().map(|s| s.id.clone()).collect();
        assert_eq!(
            ids,
            vec![
                SubsystemPath(Arc::from("subject.robot_3")),
                SubsystemPath(Arc::from("subject.pose")),
            ],
        );
    }
}
