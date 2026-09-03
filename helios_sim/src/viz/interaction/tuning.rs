use crate::{
    cli::Cli,
    viz::interaction::{
        camera::{
            keyboard::{CameraKeyboardTuning, CameraKeyboardTuningFile},
            mouse::{CameraMouseTuning, CameraMouseTuningFile},
            rig::{CameraRigTuning, CameraRigTuningFile},
        },
        selection::{SelectionTuning, SelectionTuningFile},
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
}

#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
struct CameraTuningFile {
    keyboard: CameraKeyboardTuningFile,
    mouse: CameraMouseTuningFile,
    rig: CameraRigTuningFile,
}

pub(crate) fn load_interaction_tuning(cli: Res<Cli>, mut commands: Commands) {
    let path = cli.config_root.join(INTERACTION_TUNING_FILE);

    let file: InteractionTuningFile = Figment::new()
        .merge(Toml::file(&path))
        .extract()
        .expect("interaction tuning TOML failed to parse");

    match resolve_all(&file) {
        Ok((keyboard, mouse, rig, selection)) => {
            commands.insert_resource(keyboard);
            commands.insert_resource(mouse);
            commands.insert_resource(rig);
            commands.insert_resource(selection);
        }
        Err(e) => panic!("interaction tuning config: {e}"),
    }
}

fn resolve_all(
    file: &InteractionTuningFile,
) -> Result<
    (
        CameraKeyboardTuning,
        CameraMouseTuning,
        CameraRigTuning,
        SelectionTuning,
    ),
    InteractionTuningError,
> {
    Ok((
        CameraKeyboardTuning::resolve(&file.camera.keyboard)?,
        CameraMouseTuning::resolve(&file.camera.mouse)?,
        CameraRigTuning::resolve(&file.camera.rig)?,
        SelectionTuning::resolve(&file.selection)?,
    ))
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
