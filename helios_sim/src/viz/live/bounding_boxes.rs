//! Bounding-box view: each object's labelled box, drawn as a wire box that
//! turns and moves with the object. It shows the ground-truth label, not the
//! collider, so seen beside the collider view it shows where the two
//! disagree. Off at startup and flipped by the `viz.toggle_bounding_boxes`
//! action.

use crate::{
    core::components::BoundingBox3D,
    viz::interaction::{
        actions::{
            handle::{ActionHandle, ActionId},
            registry::ActionRegistry,
        },
        sampling::ActionState,
    },
};

use bevy::prelude::*;
use serde::Deserialize;

/// Gizmo group for the box lines, so they have their own on/off switch.
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct BoundingBoxGizmos;

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct BoundingBoxViewTuningFile {
    pub color: Option<[f32; 3]>,
}

/// Look of the bounding-box view: one operator preference for the session,
/// the same on every box.
#[derive(Resource, Debug, Clone)]
pub struct BoundingBoxViewTuning {
    /// Color of the box lines. Distinct from the collider wireframes, which
    /// coincide with the box on box-shaped objects, so either view reads
    /// with both on.
    pub color: Color,
}

impl Default for BoundingBoxViewTuning {
    fn default() -> Self {
        Self {
            color: Color::srgb(0.0, 0.8, 1.0),
        }
    }
}

impl BoundingBoxViewTuning {
    /// Overlays sparse overrides onto [`Default`], packing the `[r, g, b]`
    /// triple into an sRGB [`Color`].
    pub(crate) fn resolve(overrides: &BoundingBoxViewTuningFile) -> Self {
        let mut t = Self::default();
        if let Some([r, g, b]) = overrides.color {
            t.color = Color::srgb(r, g, b);
        }
        t
    }
}

/// Where a unit cube must go to cover `bb` on an object posed at `object`:
/// moved to the box centre and stretched to the full box size, both in the
/// object's own axes, then carried by the object's pose.
pub(crate) fn box_transform(object: &GlobalTransform, bb: &BoundingBox3D) -> GlobalTransform {
    *object * Transform::from_translation(bb.centre).with_scale(bb.half_extents * 2.0)
}

pub(crate) fn draw_bounding_boxes(
    boxes: Query<(&GlobalTransform, &BoundingBox3D)>,
    tuning: Res<BoundingBoxViewTuning>,
    mut gizmos: Gizmos<BoundingBoxGizmos>,
) {
    for (object, bb) in &boxes {
        gizmos.cube(box_transform(object, bb), tuning.color);
    }
}

pub(crate) fn toggle_bounding_boxes(
    registry: Res<ActionRegistry>,
    state: Res<ActionState>,
    mut store: ResMut<GizmoConfigStore>,
    mut handle: Local<Option<ActionHandle>>,
) {
    let h = *handle.get_or_insert_with(|| {
        registry
            .handle(ActionId("viz.toggle_bounding_boxes"))
            .expect("registered")
    });

    if state.is_active(h) {
        let (config, _) = store.config_mut::<BoundingBoxGizmos>();
        config.enabled = !config.enabled;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viz::interaction::actions::handle::{ActionMetadata, InputKind};

    use std::f32::consts::FRAC_PI_2;

    const TOLERANCE: f32 = 1e-5;

    /// A store holding the box group as `VizPlugin` configures it: switched
    /// off.
    fn box_store() -> GizmoConfigStore {
        let mut store = GizmoConfigStore::default();
        store.insert(
            GizmoConfig {
                enabled: false,
                ..default()
            },
            BoundingBoxGizmos,
        );
        store
    }

    fn box_view_enabled(app: &mut App) -> bool {
        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        store.config_mut::<BoundingBoxGizmos>().0.enabled
    }

    fn toggle_app(active: bool) -> App {
        let mut registry = ActionRegistry::default();
        let handle = registry.register(
            ActionId("viz.toggle_bounding_boxes"),
            ActionMetadata {
                label: "Toggle bounding boxes",
                group: "viz",
                kind: InputKind::Button,
                default_key: KeyCode::KeyB,
            },
        );
        let firing = if active { vec![handle] } else { Vec::new() };

        let mut app = App::new();
        app.insert_resource(registry);
        app.insert_resource(ActionState::from_active(firing));
        app.insert_resource(box_store());
        app.add_systems(Update, toggle_bounding_boxes);
        app
    }

    /// Tier 2: each firing of the action flips the view, on then off.
    #[test]
    fn toggle_flips_the_box_view_each_time_the_action_fires() {
        let mut app = toggle_app(true);

        app.update();
        assert!(box_view_enabled(&mut app), "first press turns it on");
        app.update();
        assert!(!box_view_enabled(&mut app), "second press turns it off");
    }

    /// Tier 2: with no action firing, the view stays as it is.
    #[test]
    fn toggle_leaves_the_box_view_alone_when_the_action_is_idle() {
        let mut app = toggle_app(false);

        app.update();
        assert!(!box_view_enabled(&mut app));
    }

    /// The box follows the object's rotation: a 2 × 1 × 4 box whose centre
    /// sits 0.5 up from the origin, on an object at (10, 0, 0) turned a
    /// quarter turn about up. Its unit cube's corners must land where the
    /// box's own corners go under that pose, which a box placed in world
    /// axes, or offset before rotating, would miss.
    #[test]
    fn box_turns_and_moves_with_the_object() {
        let object = GlobalTransform::from(
            Transform::from_xyz(10.0, 0.0, 0.0).with_rotation(Quat::from_rotation_y(FRAC_PI_2)),
        );
        let bb = BoundingBox3D {
            centre: Vec3::new(0.0, 0.5, 0.0),
            half_extents: Vec3::new(1.0, 0.5, 2.0),
        };

        let cube = box_transform(&object, &bb);

        // The cube's (+½, +½, +½) corner is the box's (1, 1, 2) corner in
        // object axes; a quarter turn about y takes (x, z) to (z, −x).
        let corner = cube.transform_point(Vec3::splat(0.5));
        assert!(
            corner.abs_diff_eq(Vec3::new(12.0, 1.0, -1.0), TOLERANCE),
            "corner at {corner}"
        );
        let centre = cube.transform_point(Vec3::ZERO);
        assert!(
            centre.abs_diff_eq(Vec3::new(10.0, 0.5, 0.0), TOLERANCE),
            "centre at {centre}"
        );
    }

    /// A file with no overrides resolves to the compiled-in color.
    #[test]
    fn empty_file_resolves_to_default_color() {
        let t = BoundingBoxViewTuning::resolve(&BoundingBoxViewTuningFile::default());
        assert_eq!(t.color, BoundingBoxViewTuning::default().color);
    }

    /// The `[r, g, b]` triple packs into an sRGB color.
    #[test]
    fn color_override_packs_into_srgb() {
        let file = BoundingBoxViewTuningFile {
            color: Some([0.2, 0.4, 0.6]),
        };
        assert_eq!(
            BoundingBoxViewTuning::resolve(&file).color,
            Color::srgb(0.2, 0.4, 0.6)
        );
    }
}
