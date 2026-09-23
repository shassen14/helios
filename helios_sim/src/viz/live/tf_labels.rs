//! Frame-name labels for the tf-tree overlay — a screen-space text tag at each
//! frame `tf.rs` draws a triad for.
//!
//! Unlike the triads and connectors in `tf.rs`, which are immediate-mode gizmos
//! redrawn from nothing each frame, a text label is a retained UI entity. So
//! this surface manages a lifecycle rather than issuing draw calls: one UI node
//! per frame, reconciled against the set of frames currently drawn — spawn a
//! label when a frame appears, despawn it when the frame is gone.
//!
//! There is no side table mapping frames to label entities. Each label carries a
//! [`FrameLabel`] holding its [`FrameId`], so the set of live labels is just a
//! query, and the reconcile diffs that query against the desired frame set. An
//! empty desired set (labels toggled off, or no agent selected) collapses to the
//! same diff, which despawns every label — there is no separate teardown path.
//!
//! Each label is placed by projecting its frame's world position to the screen
//! (`Camera::world_to_viewport`), so the text stays a constant, readable size
//! and faces the viewer for free, at the cost of not being occluded by geometry
//! — the right tradeoff for a diagnostic overlay. The projected position is the
//! same one `tf.rs` draws the triad at, so label and triad coincide.

use crate::prelude::{AgentIdComponent, TfServiceComponent};
use crate::viz::interaction::selection::Selected;
use crate::viz::interaction::tuning::{require_positive, InteractionTuningError};
use crate::viz::live::tf::{frame_origin_in_root, TfOverlayVisible};

use helios_core::frames::FrameId;

use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use serde::Deserialize;

/// Marks a UI text node as the label for one tf frame, tagging it with the frame
/// it names. Because every label carries this, the set of live labels *is* a
/// query over `FrameLabel` — there is no separate frame→entity map to keep in
/// sync, and the reconcile reads existing labels straight from the world.
#[derive(Component)]
pub struct FrameLabel(pub FrameId);

/// Sparse TOML overrides for frame-name label styling. Every field is optional;
/// anything omitted falls back to the compiled-in [`TfLabelTuning::default`].
#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct TfLabelTuningFile {
    pub font_size: Option<f32>,
    pub color: Option<[f32; 3]>,
    pub screen_offset: Option<[f32; 2]>,
    pub line_spacing: Option<f32>,
    pub column_width: Option<f32>,
}

/// Resolved styling for the frame-name labels. A `Resource` — one operator
/// preference for the session — read by [`tf_label_system`]. Defaults reproduce
/// the values compiled in before the tuning surface existed.
#[derive(Resource, Debug, Clone)]
pub struct TfLabelTuning {
    /// Font size, in logical screen pixels.
    pub font_size: f32,
    /// Text color.
    pub color: Color,
    /// Screen-space nudge (logical pixels) from a frame's projected point, so the
    /// text floats just above the triad rather than over its origin.
    pub screen_offset: Vec2,
    /// Gap (logical pixels) added below the glyph when stacking decluttered
    /// labels — kept separate from `font_size` so [`line_height`](Self::line_height)
    /// derives from both and the two can't drift apart.
    pub line_spacing: f32,
    /// Horizontal bucket width (logical pixels) for detecting collisions: labels
    /// sharing a column-and-row bucket are stacked instead of drawn atop one
    /// another.
    pub column_width: f32,
}

impl Default for TfLabelTuning {
    fn default() -> Self {
        Self {
            font_size: 12.0,
            color: Color::srgb(0.85, 0.85, 0.85),
            screen_offset: Vec2::new(0.0, -14.0),
            line_spacing: 4.0,
            column_width: 90.0,
        }
    }
}

impl TfLabelTuning {
    /// Row height for stacking colliding labels: the glyph size plus the spacing
    /// gap, derived so line spacing and glyph size can't drift apart.
    pub fn line_height(&self) -> f32 {
        self.font_size + self.line_spacing
    }

    /// Overlay sparse overrides onto [`Default`], packing the `[r, g, b]` color
    /// triple into an sRGB [`Color`] and the `[x, y]` offset into a [`Vec2`], and
    /// reject a non-positive font size, column width, or derived line height.
    pub(crate) fn resolve(overrides: &TfLabelTuningFile) -> Result<Self, InteractionTuningError> {
        let mut t = Self::default();
        if let Some(v) = overrides.font_size {
            t.font_size = v;
        }
        if let Some([r, g, b]) = overrides.color {
            t.color = Color::srgb(r, g, b);
        }
        if let Some([x, y]) = overrides.screen_offset {
            t.screen_offset = Vec2::new(x, y);
        }
        if let Some(v) = overrides.line_spacing {
            t.line_spacing = v;
        }
        if let Some(v) = overrides.column_width {
            t.column_width = v;
        }

        require_positive("tf_labels.font_size", t.font_size)?;
        require_positive("tf_labels.column_width", t.column_width)?;
        require_positive("tf_labels.line_height", t.line_height())?;
        Ok(t)
    }
}

/// Keeps the retained frame-name labels in step with the frames the overlay
/// draws: place surviving labels, spawn ones for new frames, despawn ones whose
/// frame is gone. The desired set is every frame of the *selected* agent's
/// estimated tree while the overlay master is on; when the master is off or no
/// agent is selected, that set is empty and the same diff despawns everything.
///
/// It shares the overlay's master toggle and selection scope with `tf.rs`, but —
/// being retained rather than immediate-mode — it can't be `run_if`-gated off
/// like the gizmo system: it must keep running to reconcile its entities away.
/// So the master is read here as an ordinary condition on building the desired
/// set, not as a schedule gate.
pub(crate) fn tf_label_system(
    overlay_visible: Res<TfOverlayVisible>,
    tuning: Res<TfLabelTuning>,
    agents: Query<(&TfServiceComponent, &AgentIdComponent), With<Selected>>,
    camera: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    mut labels: Query<(Entity, &FrameLabel, &mut Node, &mut Visibility)>,
    mut commands: Commands,
) {
    let Ok((camera, camera_transform)) = camera.single() else {
        return;
    };

    // Project every drawn frame. On-screen frames feed the declutter pass;
    // off-screen / unresolvable ones are kept as hidden labels so they reappear
    // without a respawn when they return to view. When the master is off both
    // stay empty, so the reconcile below tears every label down.
    let mut on_screen: Vec<(FrameId, Vec2)> = Vec::new();
    let mut off_screen: Vec<FrameId> = Vec::new();
    if overlay_visible.0 {
        for (tf_service, agent_id) in &agents {
            let root = FrameId::odom(agent_id.0.clone());
            for (edge, _kind) in tf_service.0.buffer().edges() {
                match frame_origin_in_root(tf_service.0.buffer(), &edge.child, &root).and_then(
                    |origin| camera.world_to_viewport(camera_transform, origin).ok(),
                ) {
                    Some(viewport) => on_screen.push((edge.child, viewport + tuning.screen_offset)),
                    None => off_screen.push(edge.child),
                }
            }
        }
    }

    // Stack labels that would land on the same spot, then fold the off-screen
    // frames back in as hidden. `None` position = hidden this frame.
    let mut desired: HashMap<FrameId, Option<Vec2>> =
        declutter(on_screen, tuning.column_width, tuning.line_height())
            .into_iter()
            .map(|(frame, pos)| (frame, Some(pos)))
            .collect();
    for frame in off_screen {
        desired.entry(frame).or_insert(None);
    }

    // One pass over the retained labels: reposition each survivor and collect the
    // current label set for the diff.
    let mut existing = Vec::new();
    for (entity, label, mut node, mut visibility) in &mut labels {
        if let Some(screen) = desired.get(&label.0) {
            place_label(&mut node, &mut visibility, *screen);
        }
        existing.push((entity, label.0.clone()));
    }

    let desired_frames: HashSet<FrameId> = desired.keys().cloned().collect();
    let (to_spawn, to_despawn) = reconcile_labels(&desired_frames, &existing);

    for frame in to_spawn {
        let screen = desired.get(&frame).copied().flatten();
        let mut node = Node {
            position_type: PositionType::Absolute,
            ..default()
        };
        let mut visibility = Visibility::Inherited;
        place_label(&mut node, &mut visibility, screen);

        commands.spawn((
            FrameLabel(frame.clone()),
            Text::new(frame.leaf().as_str()),
            TextFont {
                font_size: FontSize::Px(tuning.font_size),
                ..default()
            },
            TextColor(tuning.color),
            node,
            visibility,
        ));
    }

    for entity in to_despawn {
        commands.entity(entity).despawn();
    }
}

/// Writes a label's screen position into its UI node, or hides it when the frame
/// has no on-screen position this frame.
fn place_label(node: &mut Node, visibility: &mut Visibility, screen: Option<Vec2>) {
    match screen {
        Some(pos) => {
            node.left = Val::Px(pos.x);
            node.top = Val::Px(pos.y);
            *visibility = Visibility::Visible;
        }
        None => *visibility = Visibility::Hidden,
    }
}

/// Spreads labels that would overlap. Frames whose anchors fall in the same
/// screen cell are stacked vertically a line height apart — coincident frames
/// (the spine and origin-mounted sensors all sitting at `base_link`) become a
/// readable column instead of one unreadable smear. Frames already apart keep
/// their position. Within a cell the order is by leaf name, so the stack is
/// stable frame-to-frame rather than reshuffling as the `HashMap` iterates.
fn declutter(
    positions: Vec<(FrameId, Vec2)>,
    column_width: f32,
    line_height: f32,
) -> HashMap<FrameId, Vec2> {
    let mut buckets: HashMap<(i32, i32), Vec<(FrameId, Vec2)>> = HashMap::new();
    for (frame, pos) in positions {
        let cell = (
            (pos.x / column_width).floor() as i32,
            (pos.y / line_height).floor() as i32,
        );
        buckets.entry(cell).or_default().push((frame, pos));
    }

    let mut placed = HashMap::new();
    for (_, mut group) in buckets {
        group.sort_by(|(a, _), (b, _)| a.leaf().as_str().cmp(b.leaf().as_str()));
        for (rank, (frame, pos)) in group.into_iter().enumerate() {
            let stacked = Vec2::new(pos.x, pos.y + rank as f32 * line_height);
            placed.insert(frame, stacked);
        }
    }
    placed
}

/// Diff the frames that should carry a label (`desired`) against the labels that
/// currently exist (`existing`, each an entity paired with the frame it names).
/// Returns the frames needing a new label and the entities whose frame is no
/// longer desired. Frames present on both sides are left untouched — the caller
/// repositions every surviving label regardless, so they need no mention here.
///
/// Pure and total: the empty-`desired` case (labels off / nothing selected)
/// returns every existing entity to despawn, which is how toggling off reuses
/// this one path instead of a separate teardown.
fn reconcile_labels(
    desired: &HashSet<FrameId>,
    existing: &[(Entity, FrameId)],
) -> (Vec<FrameId>, Vec<Entity>) {
    let live: HashSet<&FrameId> = existing.iter().map(|(_, f)| f).collect();

    let to_spawn = desired
        .iter()
        .filter(|f| !live.contains(*f))
        .cloned()
        .collect();

    let to_despawn = existing
        .iter()
        .filter(|(_, f)| !desired.contains(f))
        .map(|(e, _)| *e)
        .collect();

    (to_spawn, to_despawn)
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::data::AgentId;

    fn agent() -> AgentId {
        AgentId::new("test_agent")
    }

    fn entity(index: u32) -> Entity {
        Entity::from_raw_u32(index).expect("valid test entity index")
    }

    #[test]
    fn declutter_stacks_labels_that_share_a_screen_cell() {
        let a = agent();
        let base = FrameId::base_link(a.clone());
        let odom = FrameId::odom(a.clone());
        let anchor = Vec2::new(100.0, 100.0);
        let t = TfLabelTuning::default();

        let placed = declutter(
            vec![(base.clone(), anchor), (odom.clone(), anchor)],
            t.column_width,
            t.line_height(),
        );

        // Same column, separated by at least one line height so both are legible.
        assert_eq!(placed[&base].x, anchor.x);
        assert_eq!(placed[&odom].x, anchor.x);
        assert!((placed[&base].y - placed[&odom].y).abs() >= t.line_height() - f32::EPSILON);
    }

    #[test]
    fn declutter_leaves_separated_labels_where_they_are() {
        let a = agent();
        let base = FrameId::base_link(a.clone());
        let lidar = FrameId::sensor(a.clone(), "lidar");
        let near = Vec2::new(100.0, 100.0);
        let far = Vec2::new(500.0, 400.0);
        let t = TfLabelTuning::default();

        let placed = declutter(
            vec![(base.clone(), near), (lidar.clone(), far)],
            t.column_width,
            t.line_height(),
        );

        assert_eq!(placed[&base], near);
        assert_eq!(placed[&lidar], far);
    }

    #[test]
    fn spawns_desired_frames_that_have_no_label_yet() {
        let a = agent();
        let base = FrameId::base_link(a.clone());
        let lidar = FrameId::sensor(a.clone(), "lidar");
        let desired = HashSet::from([base.clone(), lidar.clone()]);

        let (to_spawn, to_despawn) = reconcile_labels(&desired, &[]);

        assert_eq!(to_spawn.len(), 2);
        assert!(to_spawn.contains(&base));
        assert!(to_spawn.contains(&lidar));
        assert!(to_despawn.is_empty());
    }

    #[test]
    fn despawns_labels_whose_frame_is_no_longer_desired() {
        let a = agent();
        let base = FrameId::base_link(a.clone());
        let lidar = FrameId::sensor(a.clone(), "lidar");
        let desired = HashSet::from([base.clone()]);
        let existing = [(entity(0), base.clone()), (entity(1), lidar.clone())];

        let (to_spawn, to_despawn) = reconcile_labels(&desired, &existing);

        assert!(to_spawn.is_empty());
        assert_eq!(to_despawn, vec![entity(1)]);
    }

    #[test]
    fn steady_state_spawns_and_despawns_nothing() {
        let a = agent();
        let base = FrameId::base_link(a.clone());
        let lidar = FrameId::sensor(a.clone(), "lidar");
        let desired = HashSet::from([base.clone(), lidar.clone()]);
        let existing = [(entity(0), base), (entity(1), lidar)];

        let (to_spawn, to_despawn) = reconcile_labels(&desired, &existing);

        assert!(to_spawn.is_empty());
        assert!(to_despawn.is_empty());
    }

    #[test]
    fn label_tuning_line_height_tracks_font_and_spacing() {
        let file = TfLabelTuningFile {
            font_size: Some(20.0),
            line_spacing: Some(6.0),
            ..Default::default()
        };

        let t = TfLabelTuning::resolve(&file).expect("valid overrides resolve");

        assert_eq!(t.line_height(), 26.0);
    }

    #[test]
    fn label_tuning_rejects_a_nonpositive_font_size() {
        let file = TfLabelTuningFile {
            font_size: Some(0.0),
            ..Default::default()
        };

        assert!(TfLabelTuning::resolve(&file).is_err());
    }

    #[test]
    fn empty_desired_despawns_every_label() {
        // The toggle-off / nothing-selected case: no frame is desired, so the
        // same diff tears every label down — one path, not a separate teardown.
        let a = agent();
        let existing = [
            (entity(0), FrameId::base_link(a.clone())),
            (entity(1), FrameId::sensor(a.clone(), "lidar")),
        ];

        let (to_spawn, to_despawn) = reconcile_labels(&HashSet::new(), &existing);

        assert!(to_spawn.is_empty());
        assert_eq!(to_despawn.len(), 2);
    }
}
