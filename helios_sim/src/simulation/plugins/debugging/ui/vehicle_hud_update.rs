// helios_sim/src/simulation/plugins/debugging/ui/vehicle_hud_update.rs
//
// Runtime update systems for the vehicle HUD.

use bevy::prelude::*;

use helios_core::control::ControlOutput;
use helios_core::frames::{FrameId, StateVariable};

use crate::simulation::core::components::{
    ControlOutputComponent, ControllerStateSource, GroundTruthState,
};
use crate::simulation::plugins::autonomy::EstimatorComponent;
use crate::simulation::plugins::debugging::components::DebugVisualizationConfig;
use crate::simulation::plugins::debugging::ui::vehicle_hud::VehicleHudEntities;
use crate::simulation::plugins::vehicles::ackermann::components::{
    AckermannActuator, AckermannCommand,
};

/// Reads agent components each frame and updates HUD text/gauges.
/// No-ops if `VehicleHudEntities` resource is absent (HUD was not spawned).
#[allow(clippy::too_many_arguments)]
pub fn update_vehicle_hud(
    entities: Option<Res<VehicleHudEntities>>,
    config: Res<DebugVisualizationConfig>,
    mut node_q: Query<&mut Node>,
    mut transform_q: Query<&mut Transform>,
    mut text_q: Query<&mut Text>,
    mut bg_q: Query<&mut BackgroundColor>,
    mut vis_q: Query<&mut Visibility>,
    agent_q: Query<(
        &GroundTruthState,
        &AckermannActuator,
        &ControllerStateSource,
        Option<&EstimatorComponent>,
        Option<&ControlOutputComponent>,
        Option<&AckermannCommand>,
    )>,
) {
    let Some(entities) = entities else {
        return;
    };

    if let Ok(mut vis) = vis_q.get_mut(entities.root) {
        *vis = if config.show_vehicle_hud {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }

    if !config.show_vehicle_hud {
        return;
    }

    let Some((gt_state, actuator, state_source, est_opt, ctrl_opt, cmd_opt)) =
        agent_q.iter().next()
    else {
        return;
    };

    let max_speed = actuator.max_speed;
    let gt_speed = gt_state.linear_velocity.norm() as f32;

    let est_speed: Option<f32> = est_opt.and_then(|est| {
        est.0
            .get_state()
            .and_then(|s| s.get_vector3(&StateVariable::Vx(FrameId::World)))
            .map(|v| v.norm() as f32)
    });

    let active_speed = match state_source {
        ControllerStateSource::GroundTruth => gt_speed,
        ControllerStateSource::Estimated => est_speed.unwrap_or(gt_speed),
    };

    let desired_speed: Option<f32> = ctrl_opt.map(|ctrl| match &ctrl.0 {
        ControlOutput::BodyVelocity { linear, .. } => linear.x as f32,
        ControlOutput::Raw(u) if !u.is_empty() => u[0] as f32 * max_speed,
        _ => 0.0,
    });

    let throttle_norm: Option<f32> = cmd_opt.map(|c| c.throttle_norm);
    let steering_norm: Option<f32> = cmd_opt.map(|c| c.steering_torque_norm);
    let show_ctrl_marker = est_speed.is_some();

    if let Ok(mut text) = text_q.get_mut(entities.desired_speed_text) {
        text.0 = match desired_speed {
            Some(s) => format!("Des: {:.1} m/s", s),
            None => "Des: --".to_string(),
        };
    }

    if let Ok(mut text) = text_q.get_mut(entities.gt_speed_text) {
        let marker = if show_ctrl_marker && *state_source == ControllerStateSource::GroundTruth {
            " [CTRL]"
        } else {
            ""
        };
        text.0 = format!("GT:  {:.1} m/s{}", gt_speed, marker);
    }

    if let Ok(mut text) = text_q.get_mut(entities.est_speed_text) {
        text.0 = match est_speed {
            Some(spd) => {
                let marker = if *state_source == ControllerStateSource::Estimated {
                    " [CTRL]"
                } else {
                    ""
                };
                format!("Est: {:.1} m/s{}", spd, marker)
            }
            None => "Est: --".to_string(),
        };
    }

    if let Ok(mut text) = text_q.get_mut(entities.throttle_text) {
        text.0 = match throttle_norm {
            Some(t) => format!("{:+.0}%", t * 100.0),
            None => "--".to_string(),
        };
    }

    if let Ok(mut text) = text_q.get_mut(entities.steering_text) {
        text.0 = match steering_norm {
            Some(s) => format!("{:+.0}%", s * 100.0),
            None => "--".to_string(),
        };
    }

    if let Ok(mut node) = node_q.get_mut(entities.speed_fill) {
        let pct = if max_speed > 0.0 {
            (active_speed / max_speed * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };
        node.width = Val::Percent(pct);
    }

    if let Ok(mut node) = node_q.get_mut(entities.throttle_fill) {
        let pct = throttle_norm.map(|t| t.abs() * 100.0).unwrap_or(0.0);
        node.height = Val::Percent(pct);
    }

    if let Ok(mut bg) = bg_q.get_mut(entities.throttle_fill) {
        let color = match throttle_norm {
            Some(t) if t < 0.0 => Color::srgb(0.8, 0.1, 0.1),
            _ => Color::srgb(0.0, 0.8, 0.0),
        };
        *bg = BackgroundColor(color);
    }

    if let Ok(mut transform) = transform_q.get_mut(entities.steering_spoke) {
        let angle = steering_norm
            .map(|s| s * 135.0_f32.to_radians())
            .unwrap_or(0.0);
        transform.rotation = Quat::from_rotation_z(angle);
    }
}

/// Flips `ControllerStateSource` on all agent entities when T is pressed.
pub fn toggle_state_source(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut source_q: Query<&mut ControllerStateSource>,
) {
    if !keyboard.just_pressed(KeyCode::KeyT) {
        return;
    }
    for mut source in &mut source_q {
        *source = match *source {
            ControllerStateSource::Estimated => {
                info!("[Debug] Controller state source → GroundTruth");
                ControllerStateSource::GroundTruth
            }
            ControllerStateSource::GroundTruth => {
                info!("[Debug] Controller state source → Estimated");
                ControllerStateSource::Estimated
            }
        };
    }
}
