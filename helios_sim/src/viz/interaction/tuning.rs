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
}

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
struct CameraTuningFile {
    keyboard: CameraKeyboardTuningFile,
    mouse: CameraMouseTuningFile,
    rig: CameraRigTuningFile,
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
        }
        Err(e) => panic!("interaction tuning config: {e}"),
    }
}

fn resolve_all(
    file: &InteractionTuningFile,
) -> Result<ResolvedInteractionTuning, InteractionTuningError> {
    Ok(ResolvedInteractionTuning {
        camera_keyboard: CameraKeyboardTuning::resolve(&file.camera.keyboard)?,
        camera_mouse: CameraMouseTuning::resolve(&file.camera.mouse)?,
        camera_rig: CameraRigTuning::resolve(&file.camera.rig)?,
        selection: SelectionTuning::resolve(&file.selection)?,
        tf_overlay: TfOverlayTuning::resolve(&file.tf_overlay)?,
        tf_labels: TfLabelTuning::resolve(&file.tf_labels)?,
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
