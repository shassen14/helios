//! Interactive teleop: reads the bound drive keys and publishes a device- and
//! morphology-agnostic [`TwistIntent`](helios_core::control::commands::TwistIntent)
//! onto the piloted agent's pipeline bus. The runtime's `TwistTeleopNode` scales
//! that intent into a body command — the host does device I/O only and knows
//! nothing about which axes a body drives.
//!
//! Added by the `helios_play` bin, never `HeliosSimulationPlugin`: the headless
//! test bin shares that body and must stay input-free, the same rule the camera
//! follows.
//!
//! The mapping runs in one stage — [`publish_teleop_intent`] folds the held keys
//! into a [`TwistIntent`] and publishes it. Unlike the camera, there is no shared
//! intent buffer: the value crosses the host↔brain boundary immediately, and the
//! per-DOF tuning and axis selection live downstream in the runtime node.

pub mod intent;

use crate::prelude::AppState;
use crate::viz::interaction::{
    actions::{
        handle::{ActionHandle, ActionId, ActionMetadata, InputKind},
        registry::ActionRegistry,
    },
    sampling::ActionState,
    teleop::intent::publish_teleop_intent,
    InteractionSet,
};

use bevy::prelude::*;

/// Schedule anchor for the teleop publish, ordered after `InteractionSet::Sampling`
/// so it reads this frame's action state rather than last frame's.
#[derive(SystemSet, Clone, PartialEq, Eq, Hash, Debug)]
pub enum TeleopSet {
    Publish,
}

pub struct TeleopPlugin;

impl Plugin for TeleopPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Startup,
            register_teleop_actions.in_set(InteractionSet::Registration),
        );

        // Teleop consumes the sampled action state, so it declares its own
        // dependency on the producer set — the input infra stays ignorant of it,
        // matching how the camera and viz layers order themselves.
        app.configure_sets(Update, TeleopSet::Publish.after(InteractionSet::Sampling));

        app.add_systems(
            Update,
            publish_teleop_intent
                .in_set(TeleopSet::Publish)
                .run_if(in_state(AppState::Running)),
        );
    }
}

/// The `(negative, positive)` action pair driving one intent DOF. [`deflection`]
/// collapses the two held states to a signed axis value — the whole of the
/// per-axis logic, so the intent builder never special-cases a body.
///
/// [`deflection`]: AxisPair::deflection
pub(crate) struct AxisPair {
    negative: ActionHandle,
    positive: ActionHandle,
}

impl AxisPair {
    /// This axis's signed deflection this frame: `+1` positive held, `-1` negative
    /// held, `0` for neither or both. The `[-1, 1]` contract falls out of the two
    /// booleans; the runtime mapper re-clamps defensively regardless.
    fn deflection(&self, state: &ActionState) -> f64 {
        f64::from(state.is_active(self.positive)) - f64::from(state.is_active(self.negative))
    }
}

/// Which DOF the current scheme binds to keys — one optional [`AxisPair`] per FLU
/// motion axis. A `None` axis has no keys, and the host never claims the body
/// drives it; the runtime `[teleop]` scale independently decides which axes a body
/// *can* drive, so an axis bound here but unscaled downstream simply produces no
/// motion.
#[derive(Resource, Default)]
pub(crate) struct TeleopActions {
    surge: Option<AxisPair>,
    sway: Option<AxisPair>,
    heave: Option<AxisPair>,
    roll: Option<AxisPair>,
    pitch: Option<AxisPair>,
    yaw: Option<AxisPair>,
}

/// Register the one teleop keyboard scheme and cache its handles in
/// [`TeleopActions`].
///
/// This is a **single global scheme**: the arrow keys drive surge and yaw, the
/// two DOF a car steers. It is deliberately minimal — the only place a concrete
/// key→axis choice is made, so extending to another morphology is additive (bind
/// another [`AxisPair`], add a `[teleop]` scale for it) rather than a rewrite of
/// the publish path. A body that drives other axes — a holonomic platform's sway,
/// a drone's heave — and the richer scheme where the *same* key maps to a
/// different DOF per piloted agent are not modelled here: the binding is one table
/// shared by whatever agent is piloted.
///
/// Runs once at `Startup` in `InteractionSet::Registration`, a sibling of the
/// camera's registration. Each motion is an [`InputKind::Axis`], sampled while its
/// key is held — what continuous driving needs.
pub(crate) fn register_teleop_actions(
    mut registry: ResMut<ActionRegistry>,
    mut commands: Commands,
) {
    let surge = AxisPair {
        negative: registry.register(
            ActionId("teleop.surge_back"),
            axis("Drive back", KeyCode::ArrowDown),
        ),
        positive: registry.register(
            ActionId("teleop.surge_forward"),
            axis("Drive forward", KeyCode::ArrowUp),
        ),
    };
    // Yaw is left-positive (+Z, FLU), so the left key is the positive handle.
    let yaw = AxisPair {
        negative: registry.register(
            ActionId("teleop.yaw_right"),
            axis("Turn right", KeyCode::ArrowRight),
        ),
        positive: registry.register(
            ActionId("teleop.yaw_left"),
            axis("Turn left", KeyCode::ArrowLeft),
        ),
    };

    commands.insert_resource(TeleopActions {
        surge: Some(surge),
        yaw: Some(yaw),
        ..default()
    });
}

/// Metadata for an `Axis`-kind teleop action — sampled while its key is held.
fn axis(label: &'static str, default_key: KeyCode) -> ActionMetadata {
    ActionMetadata {
        label,
        group: "teleop",
        kind: InputKind::Axis,
        default_key,
    }
}

#[cfg(test)]
mod tests {
    use super::{register_teleop_actions, TeleopActions};

    use crate::viz::interaction::actions::{handle::ActionId, registry::ActionRegistry};

    use bevy::prelude::*;

    /// Tier-3 wiring guard: catches someone dropping the `add_systems(Startup,
    /// register_teleop_actions…)` line or the resource insert — the code still
    /// compiles, but no teleop action is declared and `publish_teleop_intent`
    /// finds no `TeleopActions`. Stands up only the registry and this one system,
    /// not the whole `TeleopPlugin`, which would drag in the sampler and a window.
    #[test]
    fn register_teleop_actions_declares_the_scheme_at_startup() {
        let mut app = App::new();
        app.init_resource::<ActionRegistry>();
        app.add_systems(Startup, register_teleop_actions);

        // The first `update()` runs the `Startup` schedule exactly once.
        app.update();

        let registry = app.world().resource::<ActionRegistry>();
        for id in [
            "teleop.surge_forward",
            "teleop.surge_back",
            "teleop.yaw_left",
            "teleop.yaw_right",
        ] {
            assert!(
                registry.handle(ActionId(id)).is_some(),
                "register_teleop_actions must declare {id} at Startup",
            );
        }

        assert!(
            app.world().get_resource::<TeleopActions>().is_some(),
            "register_teleop_actions must insert the TeleopActions handle cache",
        );
    }
}
