use helios_core::control::actuation_model::ActuationModel;

use bevy::prelude::*;

/// The vehicle's resolved actuation contract, held on the body entity.
///
/// A plant builder deposits it from the `[actuation]` config; the plant's apply
/// system reads it each tick to `resolve` the pipeline's `ActuatorCommand`
/// (clamp to limits, apply sign, substitute fail-safes) before touching physics.
/// It is the actuation axis and is morphology-agnostic — every vehicle family
/// carries one, so it lives here rather than in any one family's module.
#[derive(Component, Clone)]
pub struct ActuationModelComponent(pub ActuationModel);
