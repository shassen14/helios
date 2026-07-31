//! Layer 2 — the disposable renderer. Reads [`CurrentInspection`] each frame and
//! draws it as a `bevy_ui` panel. Sim-local, crosses no boundary, swappable without
//! touching the model. This is the one place unit *localization* happens
//! ([`format_value`]) — the model stays SI, degrees appear only here.

use super::model::Value;
use super::CurrentInspection;

use bevy::prelude::*;

// Presentation constants for the panel. A fixed panel width is what stops the whole
// (right-anchored) panel from sliding left and right as values gain or lose digits.
// Tunable; a future config surface — the same one the camera-rate consts await — can
// lift these out of source.
const PANEL_MARGIN: f32 = 10.0;
const PANEL_WIDTH: f32 = 480.0;
const SECTION_GAP: f32 = 8.0;
const ROW_INDENT: f32 = 12.0;
const TITLE_FONT_SIZE: f32 = 18.0;
const TITLE_COLOR: Color = Color::srgb(0.62, 0.80, 1.0);

/// Marks the panel's UI root so the previous frame's panel can be found and
/// despawned before the next is drawn.
#[derive(Component)]
pub struct InspectorPanelRoot;

/// Despawns last frame's panel and respawns it from the current model — the crude
/// v0 that matches rebuild-every-frame. Draws nothing when nothing is selected.
///
/// Despawn-and-respawn is intentional for the vertical slice; when the rebuild is
/// split from the per-frame value refresh, this becomes a diff against retained
/// nodes.
pub fn render_inspection(
    model: Res<CurrentInspection>,
    old: Query<Entity, With<InspectorPanelRoot>>,
    mut commands: Commands,
) {
    for e in &old {
        commands.entity(e).despawn();
    }
    let Some(m) = &model.0 else { return };

    commands
        .spawn((
            InspectorPanelRoot,
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(PANEL_MARGIN),
                top: Val::Px(PANEL_MARGIN),
                width: Val::Px(PANEL_WIDTH),
                flex_direction: FlexDirection::Column,
                ..default()
            },
        ))
        .with_children(|panel| {
            for section in &m.sections {
                panel
                    .spawn(Node {
                        flex_direction: FlexDirection::Column,
                        margin: UiRect::bottom(Val::Px(SECTION_GAP)),
                        ..default()
                    })
                    .with_children(|group| {
                        group.spawn((
                            Text::new(section.title.clone()),
                            TextFont {
                                font_size: FontSize::Px(TITLE_FONT_SIZE),
                                ..default()
                            },
                            TextColor(TITLE_COLOR),
                        ));
                        group
                            .spawn(Node {
                                flex_direction: FlexDirection::Column,
                                padding: UiRect::left(Val::Px(ROW_INDENT)),
                                ..default()
                            })
                            .with_children(|rows| {
                                for row in &section.rows {
                                    rows.spawn(Text::new(format!(
                                        "{}: {}",
                                        row.label,
                                        format_value(&row.value)
                                    )));
                                }
                            });
                    });
            }
        });
}

/// Turns an SI-canonical [`Value`] into a display string. The *only* place unit
/// localization lives: radians render as degrees, m/s carries its unit. The model
/// never holds a display unit, so this fn is where "no degrees to the pixel" is
/// actually enforced.
fn format_value(v: &Value) -> String {
    use super::model::Dimension::*;
    match v {
        Value::Scalar { value, kind: Angle } => format!("{}°", signed(value.to_degrees(), 1)),
        Value::Scalar {
            value,
            kind: Velocity,
        } => format!("{} m/s", signed(*value, 2)),
        Value::Scalar { value, .. } => signed(*value, 3),
        Value::Vector(fv) => format!(
            "[{}, {}, {}] {:?}",
            signed(fv.value[0], 2),
            signed(fv.value[1], 2),
            signed(fv.value[2], 2),
            fv.frame
        ),
        Value::Flag(b) => b.to_string(),
        Value::Label(s) => s.clone(),
    }
}

/// Formats a float with a **reserved sign column**: a non-negative value gets a leading
/// space where the minus would be, so a value flipping sign keeps the same width and
/// does not jostle the columns beside it. Rust's `format!` has no printf space-flag, so
/// the space is added by hand.
fn signed(value: f64, precision: usize) -> String {
    if value.is_sign_negative() {
        format!("{value:.precision$}")
    } else {
        format!(" {value:.precision$}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viz::interaction::inspector::model::{Dimension, Frame, FramedVec3};

    use std::f64::consts::PI;

    /// Tier 1: the load-bearing conversion — an `Angle` in radians must render in
    /// degrees, never radians. π → 180.0°. This is the concrete guard that a
    /// rad/deg slip never reaches a pixel. The leading space is the reserved sign
    /// column (see [`signed`]).
    #[test]
    fn angle_renders_in_degrees() {
        let v = Value::Scalar {
            value: PI,
            kind: Dimension::Angle,
        };
        assert_eq!(format_value(&v), " 180.0°");
    }

    /// Tier 1: a velocity keeps its SI unit on one line — the regression guard for
    /// the stray-newline bug that split `1.40` from `m/s`.
    #[test]
    fn velocity_renders_on_one_line_with_units() {
        let v = Value::Scalar {
            value: 1.4,
            kind: Dimension::Velocity,
        };
        assert_eq!(format_value(&v), " 1.40 m/s");
    }

    /// Tier 1: a dimension without a bespoke arm falls through to a plain scalar.
    #[test]
    fn other_scalars_render_plain() {
        let v = Value::Scalar {
            value: 2.5,
            kind: Dimension::Length,
        };
        assert_eq!(format_value(&v), " 2.500");
    }

    /// Tier 1: a vector shows all three components and names its frame, so ENU vs
    /// NED is never ambiguous on screen. Each component carries the reserved sign
    /// column.
    #[test]
    fn vector_shows_components_and_frame() {
        let v = Value::Vector(FramedVec3 {
            value: [1.0, 2.0, 3.0],
            frame: Frame::Enu,
        });
        assert_eq!(format_value(&v), "[ 1.00,  2.00,  3.00] Enu");
    }

    /// Tier 1: the sign column is what stops columns from jostling — a value and its
    /// negation render to the same width, the minus taking the reserved space rather
    /// than adding a character.
    #[test]
    fn sign_column_keeps_equal_width() {
        let pos = Value::Scalar {
            value: 1.54,
            kind: Dimension::Length,
        };
        let neg = Value::Scalar {
            value: -1.54,
            kind: Dimension::Length,
        };
        assert_eq!(format_value(&pos), " 1.540");
        assert_eq!(format_value(&neg), "-1.540");
        assert_eq!(format_value(&pos).len(), format_value(&neg).len());
    }

    /// Tier 1: flags and labels pass through as-is.
    #[test]
    fn flags_and_labels_pass_through() {
        assert_eq!(format_value(&Value::Flag(true)), "true");
        assert_eq!(format_value(&Value::Label("reverse".into())), "reverse");
    }
}
