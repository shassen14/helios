use crate::{
    cli::Cli,
    viz::{
        interaction::{
            camera::{
                keyboard::{CameraKeyboardTuning, CameraKeyboardTuningFile},
                mouse::{CameraMouseTuning, CameraMouseTuningFile},
                rig::{CameraRigTuning, CameraRigTuningFile},
            },
            selection::{SelectionTuning, SelectionTuningFile},
            tf_panel::{
                gather::{TfPanelHealthThresholds, TfPanelHealthTuningFile},
                geometry::{
                    PanelOrientation, TfPanelGraphTuning, TfPanelGraphTuningFile,
                    TfPanelHealthColors,
                },
                panel::{TfPanelDockTuning, TfPanelDockTuningFile},
            },
        },
        live::{
            tf::{TfOverlayTuning, TfOverlayTuningFile},
            tf_labels::{TfLabelTuning, TfLabelTuningFile},
        },
    },
};

use bevy::prelude::*;
use figment::{
    providers::{Format, Toml},
    Figment,
};
use serde::Deserialize;
use std::fmt::Display;

const INTERACTION_TUNING_FILE: &str = "sim/interaction/default.toml";

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
struct InteractionTuningFile {
    camera: CameraTuningFile,
    selection: SelectionTuningFile,
    tf_overlay: TfOverlayTuningFile,
    tf_labels: TfLabelTuningFile,
    tf_panel: TfPanelTuningFile,
}

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
struct CameraTuningFile {
    keyboard: CameraKeyboardTuningFile,
    mouse: CameraMouseTuningFile,
    rig: CameraRigTuningFile,
}

/// The `[tf_panel.*]` sections, grouped by concept: the screen-space `dock`, the
/// `graph` geometry and node chrome, and edge `health` (thresholds + verdict colours).
/// Each sub-file is owned by the subsystem that consumes it; this only nests them for
/// one parse, the way [`CameraTuningFile`] groups the camera sub-files.
#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
struct TfPanelTuningFile {
    dock: TfPanelDockTuningFile,
    graph: TfPanelGraphTuningFile,
    health: TfPanelHealthTuningFile,
}

/// Every runtime tuning resource produced from one parse of the interaction TOML.
/// `resolve_all` fills it and `load_interaction_tuning` inserts each field. Naming the
/// fields keeps the resource set — which grows as subsystems (camera, selection, the tf
/// overlay and panel, …) are added — off tuple position, so a new resource can't be
/// silently inserted under the wrong resource type.
struct ResolvedInteractionTuning {
    camera_keyboard: CameraKeyboardTuning,
    camera_mouse: CameraMouseTuning,
    camera_rig: CameraRigTuning,
    selection: SelectionTuning,
    tf_overlay: TfOverlayTuning,
    tf_labels: TfLabelTuning,
    tf_panel_dock: TfPanelDockTuning,
    /// The panel's starting orientation, resolved out of the graph section because it
    /// is a separate live-toggled resource rather than a styling field.
    tf_panel_orientation: PanelOrientation,
    tf_panel_graph: TfPanelGraphTuning,
    tf_panel_health_thresholds: TfPanelHealthThresholds,
    tf_panel_health_colors: TfPanelHealthColors,
}

pub(crate) fn load_interaction_tuning(cli: Res<Cli>, mut commands: Commands) {
    let path = cli.config_root.join(INTERACTION_TUNING_FILE);

    let file: InteractionTuningFile = Figment::new()
        .merge(Toml::file(&path))
        .extract()
        .expect("interaction tuning TOML failed to parse");

    match resolve_all(&file) {
        Ok(resolved) => {
            commands.insert_resource(resolved.camera_keyboard);
            commands.insert_resource(resolved.camera_mouse);
            commands.insert_resource(resolved.camera_rig);
            commands.insert_resource(resolved.selection);
            commands.insert_resource(resolved.tf_overlay);
            commands.insert_resource(resolved.tf_labels);
            commands.insert_resource(resolved.tf_panel_dock);
            commands.insert_resource(resolved.tf_panel_orientation);
            commands.insert_resource(resolved.tf_panel_graph);
            commands.insert_resource(resolved.tf_panel_health_thresholds);
            commands.insert_resource(resolved.tf_panel_health_colors);
        }
        Err(e) => panic!("interaction tuning config: {e}"),
    }
}

fn resolve_all(
    file: &InteractionTuningFile,
) -> Result<ResolvedInteractionTuning, InteractionTuningError> {
    // The graph and health sections each fan into more than one resource, so they are
    // resolved into locals before the struct is assembled.
    let (tf_panel_orientation, tf_panel_graph) =
        TfPanelGraphTuning::resolve(&file.tf_panel.graph)?;
    let (tf_panel_health_thresholds, tf_panel_health_colors) =
        TfPanelHealthThresholds::resolve(&file.tf_panel.health)?;

    Ok(ResolvedInteractionTuning {
        camera_keyboard: CameraKeyboardTuning::resolve(&file.camera.keyboard)?,
        camera_mouse: CameraMouseTuning::resolve(&file.camera.mouse)?,
        camera_rig: CameraRigTuning::resolve(&file.camera.rig)?,
        selection: SelectionTuning::resolve(&file.selection)?,
        tf_overlay: TfOverlayTuning::resolve(&file.tf_overlay)?,
        tf_labels: TfLabelTuning::resolve(&file.tf_labels)?,
        tf_panel_dock: TfPanelDockTuning::resolve(&file.tf_panel.dock)?,
        tf_panel_orientation,
        tf_panel_graph,
        tf_panel_health_thresholds,
        tf_panel_health_colors,
    })
}

#[derive(Debug)]
pub enum InteractionTuningError {
    /// A gain, rate, or length that must be positive was zero or negative.
    NonPositive { field: &'static str, value: f32 },
    /// The rig's orbit-distance bounds don't form a non-empty, above-zero range.
    DistanceRange { min: f32, max: f32 },
    /// The rig's pitch margin isn't in `(0, π/2)`, so it can't hold the camera off
    /// the vertical singularity.
    PitchEps { value_rad: f32 },
    /// The selection ring margin is below `1.0`, which would draw the ring inside
    /// the object's own footprint.
    MarginTooSmall { value: f32 },
    /// The tf panel's `default_orientation` string matched no known orientation.
    UnknownOrientation { value: String },
    /// The tf panel's `dead_after` is not strictly greater than `stale_after`, so the
    /// staleness buckets would be misordered.
    HealthThresholdOrder { stale: f64, dead: f64 },
}

impl Display for InteractionTuningError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonPositive { field, value } => {
                write!(f, "{field} must be positive, got {value}")
            }
            Self::DistanceRange { min, max } => write!(
                f,
                "min_distance ({min}) must be > 0 and < max_distance ({max})"
            ),
            Self::PitchEps { value_rad } => {
                write!(f, "pitch_eps ({value_rad} rad) must be in (0, π/2)")
            }
            Self::MarginTooSmall { value } => {
                write!(f, "highlight_margin ({value}) must be >= 1.0")
            }
            Self::UnknownOrientation { value } => write!(
                f,
                "tf_panel default_orientation ('{value}') must be 'sideways' or 'top_down'"
            ),
            Self::HealthThresholdOrder { stale, dead } => write!(
                f,
                "tf_panel dead_after ({dead}) must be greater than stale_after ({stale})"
            ),
        }
    }
}

/// Rejects a value that must be strictly positive. Shared by the subsystem `resolve`
/// fns for their rate / sensitivity / length checks.
pub(crate) fn require_positive(
    field: &'static str,
    value: f32,
) -> Result<(), InteractionTuningError> {
    if value > 0.0 {
        Ok(())
    } else {
        Err(InteractionTuningError::NonPositive { field, value })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The aggregate seam: the `[tf_panel.*]` section names and nesting must match the
    /// file structs, and `resolve_all` must fan the graph and health sections out into
    /// their several resources. Per-subsystem `resolve` tests cover the value logic;
    /// this covers the wiring between the TOML sections and the resolved resources,
    /// which would otherwise only fail at runtime (a mistyped section is swallowed by
    /// `#[serde(default)]`, a fan-out mistake by nothing).
    #[test]
    fn tf_panel_sections_deserialize_and_fan_out() {
        let toml = r#"
            [tf_panel.dock]
            width = 500.0

            [tf_panel.graph]
            default_orientation = "top_down"
            col_pitch = 200.0

            [tf_panel.health]
            stale_after = 1.0
            dead_after = 3.0
        "#;

        let file: InteractionTuningFile = Figment::new()
            .merge(Toml::string(toml))
            .extract()
            .expect("tf_panel sections parse against the file structs");
        let resolved = resolve_all(&file).expect("valid overrides resolve");

        assert_eq!(resolved.tf_panel_dock.width, 500.0);
        assert_eq!(resolved.tf_panel_orientation, PanelOrientation::TopDown);
        assert_eq!(resolved.tf_panel_graph.col_pitch, 200.0);
        assert_eq!(resolved.tf_panel_health_thresholds.stale_after, 1.0);
        assert_eq!(resolved.tf_panel_health_thresholds.dead_after, 3.0);
    }

    /// An unknown key inside a `[tf_panel.*]` section is rejected rather than silently
    /// ignored — the `deny_unknown_fields` guard that catches a typo'd override.
    #[test]
    fn tf_panel_unknown_key_is_rejected() {
        let toml = r#"
            [tf_panel.dock]
            widht = 500.0
        "#;

        let parsed = Figment::new()
            .merge(Toml::string(toml))
            .extract::<InteractionTuningFile>();
        assert!(parsed.is_err(), "a misspelled key must not be silently dropped");
    }
}
