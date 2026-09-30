//! General object selection: click any discrete world object to mark it
//! [`Selected`], then let independent consumers react to that one marker.
//!
//! Selection is deliberately *general*, not agent-only — a tree, a sign, or an
//! agent are all selectable; the ground is not. [`Selectable`] marks the identity
//! roots (added by [`ensure_selectable`] to every agent and world object whose
//! class is not listed as unselectable), and a single global observer,
//! [`on_click_select`], resolves a raw pointer hit up to the nearest `Selectable`
//! ancestor and moves the lone [`Selected`] marker onto it.
//!
//! Nothing here knows what selection is *for*: consumers intersect [`Selected`]
//! with the components they own — the camera follows it (`retarget_on_selection`),
//! the map toggle filters on it, and [`highlight_selection`] rings it. A consumer
//! that owns no relevant component simply never matches, so no per-type enum is
//! needed. Added only by `helios_play`, never the headless host.

use crate::{
    core::components::ObjectClass,
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
    world::ResolvedWorldLayout,
};

use helios_core::interchange::perception::semantic_class::{SemanticClass, SemanticTaxonomy};

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
/// objects, never to one whose class is unselectable (the ground), so a ground
/// click resolves to nothing.
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
    pub unselectable_classes: Option<Vec<String>>,
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
    /// Semantic class names that are never selected: surfaces such as the
    /// ground, which a click lands on whenever it misses every prop. Names,
    /// not IDs, because the class catalog is chosen per scenario.
    pub unselectable_classes: Vec<String>,
}

impl Default for SelectionTuning {
    fn default() -> Self {
        Self {
            highlight_color: Color::srgb(1.0, 0.85, 0.2),
            highlight_radius: 1.5,
            highlight_margin: 1.15,
            unselectable_classes: Vec::new(),
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
        if let Some(v) = &overrides.unselectable_classes {
            t.unselectable_classes = v.clone();
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
        let (base, radius) = match bbox {
            // Centred under the box's base, which is off the origin whenever
            // the object's origin is not its centre.
            Some(bb) => (
                transform.transform_point(bb.centre - Vec3::Y * bb.half_extents.y),
                bb.half_extents.x.max(bb.half_extents.z) * tuning.highlight_margin,
            ),
            None => (transform.translation(), tuning.highlight_radius),
        };

        let ring = Isometry3d::new(base, Quat::from_rotation_x(FRAC_PI_2));
        gizmos.circle(ring, radius, tuning.highlight_color);
    }
}

/// Tags agents and world objects with [`Selectable`] exactly once, skipping
/// world objects whose class is unselectable.
///
/// Keeps selection state out of the headless-shared scene build: the agents and
/// props are spawned by the shared host, so this marker is backfilled here from a
/// windowed-only system. `Without<Selectable>` makes it idempotent — each entity
/// is tagged once, then drops out of the query. A skipped object stays in the
/// query and is rechecked each frame, which is cheap while surfaces are a few
/// large objects. Mirrors `ensure_map_visible`.
///
/// The unselectable names are turned into classes once, on the first run, since
/// the scene's class catalog exists only once the layout has loaded.
#[allow(clippy::type_complexity)]
fn ensure_selectable(
    query: Query<
        (Entity, Option<&ObjectClass>),
        (
            Without<Selectable>,
            Or<(With<WorldObjectType>, With<AutonomyPipelineComponent>)>,
        ),
    >,
    tuning: Res<SelectionTuning>,
    layout: Option<Res<ResolvedWorldLayout>>,
    mut unselectable: Local<Option<Vec<SemanticClass>>>,
    mut commands: Commands,
) {
    let unselectable = unselectable.get_or_insert_with(|| match &layout {
        Some(layout) => unselectable_classes(&tuning.unselectable_classes, layout.taxonomy()),
        // Without a layout no object carries a class, so there is nothing to skip.
        None => Vec::new(),
    });

    for (e, class) in &query {
        if class.is_some_and(|class| unselectable.contains(&class.0)) {
            continue;
        }
        commands.entity(e).insert(Selectable);
    }
}

/// The classes `names` refer to in `taxonomy`. A name the catalog does not
/// have is warned about and dropped: the objects it meant would stay
/// selectable, and the operator file is shared by scenarios whose catalogs
/// differ, so it cannot fail the run.
fn unselectable_classes(names: &[String], taxonomy: &SemanticTaxonomy) -> Vec<SemanticClass> {
    names
        .iter()
        .filter_map(|name| match taxonomy.class(name) {
            Ok(class) => Some(class),
            Err(unknown) => {
                warn!("selection.unselectable_classes: {unknown}");
                None
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::structs::WorldLayout;
    use crate::config::LoadedWorldLayout;

    use figment::{
        providers::{Format, Toml},
        Figment,
    };
    use std::collections::BTreeMap;

    /// A layout with no objects, carrying a catalog of `terrain` and `crate`.
    fn layout_with_classes() -> ResolvedWorldLayout {
        let taxonomy: SemanticTaxonomy = Figment::new()
            .merge(Toml::string(
                "[[class]]\nname = \"unlabeled\"\nid = 0\n\
                 [[class]]\nname = \"terrain\"\nid = 3\n\
                 [[class]]\nname = \"crate\"\nid = 7\n",
            ))
            .extract()
            .expect("test catalog is valid");
        let loaded = LoadedWorldLayout {
            layout: WorldLayout {
                name: "yard".to_string(),
                objects: Vec::new(),
            },
            prefabs: BTreeMap::new(),
            taxonomy,
        };
        ResolvedWorldLayout::resolve(&loaded).expect("test layout resolves")
    }

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
        app.init_resource::<SelectionTuning>();
        let entity = app.world_mut().spawn(WorldObjectType("tree".into())).id();

        app.add_systems(Update, ensure_selectable);

        app.update();
        assert!(app.world().get::<Selectable>(entity).is_some());

        // Second pass must not double-insert or panic.
        app.update();
        assert!(app.world().get::<Selectable>(entity).is_some());
    }

    /// Tier 2: an object whose class is listed unselectable is never tagged,
    /// while one of another class is, and a listed name the catalog lacks is
    /// dropped without affecting the rest.
    #[test]
    fn ensure_selectable_skips_unselectable_classes() {
        let layout = layout_with_classes();
        let terrain = layout.taxonomy().class("terrain").expect("in catalog");
        let crate_class = layout.taxonomy().class("crate").expect("in catalog");

        let mut app = App::new();
        app.insert_resource(SelectionTuning {
            unselectable_classes: vec!["terrain".into(), "sidewalk".into()],
            ..Default::default()
        });
        app.insert_resource(layout);
        let ground = app
            .world_mut()
            .spawn((WorldObjectType("ground".into()), ObjectClass(terrain)))
            .id();
        let crate_a = app
            .world_mut()
            .spawn((WorldObjectType("crate".into()), ObjectClass(crate_class)))
            .id();

        app.add_systems(Update, ensure_selectable);
        app.update();

        assert!(app.world().get::<Selectable>(ground).is_none());
        assert!(app.world().get::<Selectable>(crate_a).is_some());
    }

    /// A file with no overrides resolves to exactly the compiled-in defaults.
    #[test]
    fn empty_file_resolves_to_defaults() {
        let t = SelectionTuning::resolve(&SelectionTuningFile::default()).unwrap();
        let d = SelectionTuning::default();
        assert_eq!(t.highlight_color, d.highlight_color);
        assert_eq!(t.highlight_radius, d.highlight_radius);
        assert_eq!(t.highlight_margin, d.highlight_margin);
        assert_eq!(t.unselectable_classes, d.unselectable_classes);
    }

    /// The `[r, g, b]` triple packs into an sRGB color; the numeric fields pass
    /// through unchanged.
    #[test]
    fn overrides_pack_color_and_pass_through_numbers() {
        let file = SelectionTuningFile {
            highlight_color: Some([0.1, 0.2, 0.3]),
            highlight_radius: Some(4.0),
            highlight_margin: Some(1.5),
            unselectable_classes: Some(vec!["terrain".into()]),
        };
        let t = SelectionTuning::resolve(&file).unwrap();
        assert_eq!(t.highlight_color, Color::srgb(0.1, 0.2, 0.3));
        assert_eq!(t.highlight_radius, 4.0);
        assert_eq!(t.highlight_margin, 1.5);
        assert_eq!(t.unselectable_classes, ["terrain"]);
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
