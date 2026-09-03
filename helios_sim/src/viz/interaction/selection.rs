//! General object selection: click any discrete world object to mark it
//! [`Selected`], then let independent consumers react to that one marker.
//!
//! Selection is deliberately *general*, not agent-only — a tree, a sign, or an
//! agent are all selectable; the ground is not. [`Selectable`] marks the identity
//! roots (added by [`ensure_selectable`]), and a single global observer,
//! [`on_click_select`], resolves a raw pointer hit up to the nearest `Selectable`
//! ancestor and moves the lone [`Selected`] marker onto it.
//!
//! Nothing here knows what selection is *for*: consumers intersect [`Selected`]
//! with the components they own — the camera follows it (`retarget_on_selection`),
//! the map toggle filters on it, and [`highlight_selection`] rings it. A consumer
//! that owns no relevant component simply never matches, so no per-type enum is
//! needed. Added only by `helios_play`, never the headless host.

use crate::{
    prelude::{AppState, AutonomyPipelineComponent, BoundingBox3D, WorldObjectType},
    viz::{
        interaction::{
            actions::{
                handle::{ActionHandle, ActionId, ActionMetadata, InputKind},
                registry::ActionRegistry,
            },
            sampling::ActionState,
            tuning::{require_positive, InteractionTuningError},
            InteractionSet,
        },
        VizSet,
    },
};

use bevy::prelude::*;
use serde::Deserialize;
use std::f32::consts::FRAC_PI_2;

/// Wires selection into the app: the mesh-picking backend, the click observer,
/// the deselect action, and the per-frame marker consumers this crate owns.
pub struct SelectionPlugin;

impl Plugin for SelectionPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MeshPickingPlugin);

        app.add_observer(on_click_select);

        app.add_systems(
            Startup,
            register_selection_actions.in_set(InteractionSet::Registration),
        );

        app.add_systems(
            Update,
            (ensure_selectable, deselect_on_escape, highlight_selection)
                .in_set(VizSet::Live)
                .run_if(in_state(AppState::Running)),
        );
    }
}

/// Registers the `selection.deselect` action (Escape by default) so
/// [`deselect_on_escape`] can resolve its handle at runtime.
pub(crate) fn register_selection_actions(mut registry: ResMut<ActionRegistry>) {
    registry.register(
        ActionId("selection.deselect"),
        ActionMetadata {
            label: "Deselect",
            group: "selection",
            kind: InputKind::Button,
            default_key: KeyCode::Escape,
        },
    );
}

/// Marks the one currently-selected entity.
///
/// At most one exists at a time — [`on_click_select`] clears the previous marker
/// before setting a new one. This is the shared vocabulary every consumer reads.
#[derive(Component)]
pub struct Selected;

/// Marks an entity as a valid selection *root*: the identity-bearing parent a raw
/// mesh hit is resolved up to. Added by [`ensure_selectable`] to agents and world
/// objects, never to terrain, so a ground click resolves to nothing.
#[derive(Component)]
pub struct Selectable;

/// Global observer: resolves a left-click to the nearest [`Selectable`] and moves
/// [`Selected`] onto it.
///
/// The ray hits a *mesh*, which for a glTF prop is a descendant of the entity that
/// carries the identity, so this rises from the hit through its ancestors to the
/// first `Selectable`. Propagation is stopped up front so the walk runs once. A
/// click that resolves to no `Selectable` (the ground, empty space) leaves the
/// current selection untouched.
pub fn on_click_select(
    mut click: On<Pointer<Click>>,
    parents: Query<&ChildOf>,
    selectable: Query<(), With<Selectable>>,
    selected: Query<Entity, With<Selected>>,
    mut commands: Commands,
) {
    click.propagate(false);

    // didn't get a left click
    if click.event.button != PointerButton::Primary {
        return;
    }

    let hit = click.original_event_target();
    let Some(root) = std::iter::once(hit)
        .chain(parents.iter_ancestors(hit))
        .find(|&e| selectable.contains(e))
    else {
        return;
    };

    for previous in &selected {
        commands.entity(previous).remove::<Selected>();
    }

    commands.entity(root).insert(Selected);
}

/// Clears the selection when the `selection.deselect` action fires.
pub fn deselect_on_escape(
    state: Res<ActionState>,
    registry: Res<ActionRegistry>,
    selected: Query<Entity, With<Selected>>,
    mut commands: Commands,
    mut handle: Local<Option<ActionHandle>>,
) {
    // TODO: we have a bunch of hardcoded &str in ActionId to reference to
    // I would like to have a file or maybe multiple per dir that would be
    // the vocabulary for such actions. this way we can reference them
    // later without possible typing errors
    let h = *handle.get_or_insert_with(|| {
        registry
            .handle(ActionId("selection.deselect"))
            .expect("selection.deselect registered at startup")
    });

    if state.is_active(h) {
        for e in &selected {
            commands.entity(e).remove::<Selected>();
        }
    }
}

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct SelectionTuningFile {
    pub highlight_color: Option<[f32; 3]>,
    pub highlight_radius: Option<f32>,
    pub highlight_margin: Option<f32>,
}

/// Look of the selection ring drawn by [`highlight_selection`]. A `Resource`, not
/// per-entity state: this is one operator preference for the whole session — the
/// ring looks the same on every selectable — whereas *which* entity is ringed is
/// the per-entity [`Selected`] marker. Defaults reproduce the values that were
/// compiled in before the tuning surface existed.
#[derive(Resource, Debug, Clone)]
pub struct SelectionTuning {
    /// Color of the ground ring.
    pub highlight_color: Color,
    /// Ring radius (meters) for a selected object with no bounding box to size it.
    pub highlight_radius: f32,
    /// Fraction the ring sits outside a bounded object's footprint — `1.15` rings
    /// it 15% wider than its half-extent so the outline reads as around, not on,
    /// the object. Unitless; multiplies the bounding-box extent.
    pub highlight_margin: f32,
}

impl Default for SelectionTuning {
    fn default() -> Self {
        Self {
            highlight_color: Color::srgb(1.0, 0.85, 0.2),
            highlight_radius: 1.5,
            highlight_margin: 1.15,
        }
    }
}

impl SelectionTuning {
    /// Overlays sparse overrides onto [`Default`], packing the `[r, g, b]` triple
    /// into an sRGB [`Color`], and rejects a non-positive radius or a margin below
    /// `1.0` (which would ring the object *inside* its own footprint).
    pub(crate) fn resolve(overrides: &SelectionTuningFile) -> Result<Self, InteractionTuningError> {
        let mut t = Self::default();
        if let Some([r, g, b]) = overrides.highlight_color {
            t.highlight_color = Color::srgb(r, g, b);
        }
        if let Some(v) = overrides.highlight_radius {
            t.highlight_radius = v;
        }
        if let Some(v) = overrides.highlight_margin {
            t.highlight_margin = v;
        }

        require_positive("selection.highlight_radius", t.highlight_radius)?;
        if t.highlight_margin < 1.0 {
            return Err(InteractionTuningError::MarginTooSmall {
                value: t.highlight_margin,
            });
        }
        Ok(t)
    }
}

/// Draws a flat ground ring beneath the selected object, sized to its bounding
/// box when it has one, so the current selection is visible in the scene.
fn highlight_selection(
    tuning: Res<SelectionTuning>,
    selected: Query<(&GlobalTransform, Option<&BoundingBox3D>), With<Selected>>,
    mut gizmos: Gizmos,
) {
    for (transform, bbox) in &selected {
        let radius = match bbox {
            Some(bb) => bb.half_extents.x.max(bb.half_extents.z) * tuning.highlight_margin,
            None => tuning.highlight_radius,
        };

        let ring = Isometry3d::new(transform.translation(), Quat::from_rotation_x(FRAC_PI_2));
        gizmos.circle(ring, radius, tuning.highlight_color);
    }
}

/// Tags agents and world objects with [`Selectable`] exactly once.
///
/// Keeps selection state out of the headless-shared scene build: the agents and
/// props are spawned by the shared host, so this marker is backfilled here from a
/// windowed-only system. `Without<Selectable>` makes it idempotent — each entity
/// is tagged once, then drops out of the query. Mirrors `ensure_map_visible`.
#[allow(clippy::type_complexity)]
fn ensure_selectable(
    query: Query<
        Entity,
        (
            Without<Selectable>,
            Or<(With<WorldObjectType>, With<AutonomyPipelineComponent>)>,
        ),
    >,
    mut commands: Commands,
) {
    for e in &query {
        commands.entity(e).insert(Selectable);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tier 3 wiring guard: the deselect action must be declared at startup, or
    /// `deselect_on_escape`'s handle lookup panics on the first frame. Compiles
    /// clean when broken (the `add_systems` line simply drops), so it needs a test.
    #[test]
    fn register_selection_actions_declares_deselect() {
        let mut app = App::new();
        app.init_resource::<ActionRegistry>();
        app.add_systems(Startup, register_selection_actions);

        app.update();

        let registry = app.world().resource::<ActionRegistry>();
        assert!(
            registry.handle(ActionId("selection.deselect")).is_some(),
            "register_selection_actions must declare selection.deselect at startup",
        );
    }

    /// Tier 2: firing the deselect action strips `Selected` off the entity.
    #[test]
    fn deselect_removes_selected_when_the_action_is_active() {
        let mut registry = ActionRegistry::default();
        let handle = registry.register(
            ActionId("selection.deselect"),
            ActionMetadata {
                label: "Deselect",
                group: "selection",
                kind: InputKind::Button,
                default_key: KeyCode::Escape,
            },
        );

        let mut app = App::new();
        app.insert_resource(registry);
        app.insert_resource(ActionState::from_active([handle]));
        let entity = app.world_mut().spawn(Selected).id();

        app.add_systems(Update, deselect_on_escape);
        app.update();

        assert!(app.world().get::<Selected>(entity).is_none());
    }

    /// Tier 2: a world object gets tagged `Selectable`, and a second run is a
    /// no-op — the `Without<Selectable>` filter keeps it idempotent.
    #[test]
    fn ensure_selectable_tags_world_objects_idempotently() {
        let mut app = App::new();
        let entity = app.world_mut().spawn(WorldObjectType("tree".into())).id();

        app.add_systems(Update, ensure_selectable);

        app.update();
        assert!(app.world().get::<Selectable>(entity).is_some());

        // Second pass must not double-insert or panic.
        app.update();
        assert!(app.world().get::<Selectable>(entity).is_some());
    }

    /// A file with no overrides resolves to exactly the compiled-in defaults.
    #[test]
    fn empty_file_resolves_to_defaults() {
        let t = SelectionTuning::resolve(&SelectionTuningFile::default()).unwrap();
        let d = SelectionTuning::default();
        assert_eq!(t.highlight_color, d.highlight_color);
        assert_eq!(t.highlight_radius, d.highlight_radius);
        assert_eq!(t.highlight_margin, d.highlight_margin);
    }

    /// The `[r, g, b]` triple packs into an sRGB color; the numeric fields pass
    /// through unchanged.
    #[test]
    fn overrides_pack_color_and_pass_through_numbers() {
        let file = SelectionTuningFile {
            highlight_color: Some([0.1, 0.2, 0.3]),
            highlight_radius: Some(4.0),
            highlight_margin: Some(1.5),
        };
        let t = SelectionTuning::resolve(&file).unwrap();
        assert_eq!(t.highlight_color, Color::srgb(0.1, 0.2, 0.3));
        assert_eq!(t.highlight_radius, 4.0);
        assert_eq!(t.highlight_margin, 1.5);
    }

    /// A margin below 1.0 would draw the ring inside the object's footprint and is
    /// rejected.
    #[test]
    fn margin_below_one_is_rejected() {
        let file = SelectionTuningFile {
            highlight_margin: Some(0.5),
            ..Default::default()
        };
        let err = SelectionTuning::resolve(&file).unwrap_err();
        assert!(matches!(err, InteractionTuningError::MarginTooSmall { .. }));
    }

    /// A non-positive fallback radius is rejected.
    #[test]
    fn non_positive_radius_is_rejected() {
        let file = SelectionTuningFile {
            highlight_radius: Some(0.0),
            ..Default::default()
        };
        let err = SelectionTuning::resolve(&file).unwrap_err();
        assert!(matches!(err, InteractionTuningError::NonPositive { .. }));
    }
}
