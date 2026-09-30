//! Collider view: Avian's wireframe of every collider, drawn over the scene
//! so a collider that is not where its mesh is shows at a glance. Off at
//! startup and flipped by the `viz.toggle_colliders` action. Only the collider
//! outlines are drawn; Avian's other debug shapes stay off.

use crate::viz::interaction::{
    actions::{
        handle::{ActionHandle, ActionId},
        registry::ActionRegistry,
    },
    sampling::ActionState,
};

use avian3d::prelude::PhysicsGizmos;
use bevy::prelude::*;
use serde::Deserialize;

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct ColliderViewTuningFile {
    pub color: Option<[f32; 3]>,
}

/// Look of the collider view: one operator preference for the session, the
/// same on every collider.
#[derive(Resource, Debug, Clone)]
pub struct ColliderViewTuning {
    /// Color of the collider wireframes. Distinct from the amber selection
    /// ring so a selected object's collider stays readable.
    pub color: Color,
}

impl Default for ColliderViewTuning {
    fn default() -> Self {
        Self {
            color: Color::srgb(1.0, 0.5, 0.0),
        }
    }
}

impl ColliderViewTuning {
    /// Overlays sparse overrides onto [`Default`], packing the `[r, g, b]`
    /// triple into an sRGB [`Color`].
    pub(crate) fn resolve(overrides: &ColliderViewTuningFile) -> Self {
        let mut t = Self::default();
        if let Some([r, g, b]) = overrides.color {
            t.color = Color::srgb(r, g, b);
        }
        t
    }
}

/// Writes the tuned color into Avian's collider drawing. Runs at startup,
/// after the tuning file is loaded, since the gizmo group is configured in
/// `Plugin::build`, before any file is read.
pub(crate) fn apply_collider_view_tuning(
    tuning: Res<ColliderViewTuning>,
    mut store: ResMut<GizmoConfigStore>,
) {
    let (_, gizmos) = store.config_mut::<PhysicsGizmos>();
    gizmos.collider_color = Some(tuning.color);
}

pub(crate) fn toggle_colliders(
    registry: Res<ActionRegistry>,
    state: Res<ActionState>,
    mut store: ResMut<GizmoConfigStore>,
    mut handle: Local<Option<ActionHandle>>,
) {
    let h = *handle.get_or_insert_with(|| {
        registry
            .handle(ActionId("viz.toggle_colliders"))
            .expect("registered")
    });

    if state.is_active(h) {
        let (config, _) = store.config_mut::<PhysicsGizmos>();
        config.enabled = !config.enabled;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viz::interaction::actions::handle::{ActionMetadata, InputKind};

    /// A store holding the collider group as `VizPlugin` configures it:
    /// nothing drawn, and switched off.
    fn collider_store() -> GizmoConfigStore {
        let mut store = GizmoConfigStore::default();
        store.insert(
            GizmoConfig {
                enabled: false,
                ..default()
            },
            PhysicsGizmos::none(),
        );
        store
    }

    fn collider_view_enabled(app: &mut App) -> bool {
        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        store.config_mut::<PhysicsGizmos>().0.enabled
    }

    /// Tier 2: each firing of the action flips the view, on then off.
    #[test]
    fn toggle_flips_the_collider_view_each_time_the_action_fires() {
        let mut registry = ActionRegistry::default();
        let handle = registry.register(
            ActionId("viz.toggle_colliders"),
            ActionMetadata {
                label: "Toggle colliders",
                group: "viz",
                kind: InputKind::Button,
                default_key: KeyCode::KeyC,
            },
        );

        let mut app = App::new();
        app.insert_resource(registry);
        app.insert_resource(ActionState::from_active([handle]));
        app.insert_resource(collider_store());
        app.add_systems(Update, toggle_colliders);

        app.update();
        assert!(collider_view_enabled(&mut app), "first press turns it on");
        app.update();
        assert!(
            !collider_view_enabled(&mut app),
            "second press turns it off"
        );
    }

    /// Tier 2: with no action firing, the view stays as it is.
    #[test]
    fn toggle_leaves_the_collider_view_alone_when_the_action_is_idle() {
        let mut registry = ActionRegistry::default();
        registry.register(
            ActionId("viz.toggle_colliders"),
            ActionMetadata {
                label: "Toggle colliders",
                group: "viz",
                kind: InputKind::Button,
                default_key: KeyCode::KeyC,
            },
        );

        let mut app = App::new();
        app.insert_resource(registry);
        app.insert_resource(ActionState::from_active([]));
        app.insert_resource(collider_store());
        app.add_systems(Update, toggle_colliders);

        app.update();
        assert!(!collider_view_enabled(&mut app));
    }

    /// Tier 2: the tuned color reaches Avian's collider drawing, and nothing
    /// else Avian can draw is switched on by it.
    #[test]
    fn tuning_sets_only_the_collider_color() {
        let color = Color::srgb(0.1, 0.8, 0.9);
        let mut app = App::new();
        app.insert_resource(ColliderViewTuning { color });
        app.insert_resource(collider_store());
        app.add_systems(Update, apply_collider_view_tuning);

        app.update();

        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        let (config, gizmos) = store.config_mut::<PhysicsGizmos>();
        assert_eq!(gizmos.collider_color, Some(color));
        assert_eq!(gizmos.aabb_color, None);
        assert_eq!(gizmos.axis_lengths, None);
        assert!(
            !config.enabled,
            "setting the color does not turn the view on"
        );
    }

    /// A file with no overrides resolves to the compiled-in color.
    #[test]
    fn empty_file_resolves_to_default_color() {
        let t = ColliderViewTuning::resolve(&ColliderViewTuningFile::default());
        assert_eq!(t.color, ColliderViewTuning::default().color);
    }

    /// The `[r, g, b]` triple packs into an sRGB color.
    #[test]
    fn color_override_packs_into_srgb() {
        let file = ColliderViewTuningFile {
            color: Some([0.2, 0.4, 0.6]),
        };
        assert_eq!(
            ColliderViewTuning::resolve(&file).color,
            Color::srgb(0.2, 0.4, 0.6)
        );
    }
}
