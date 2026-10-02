//! Teleop's action vocabulary: one id per drive key, the registration that
//! binds them into the one keyboard scheme, and the per-axis handle cache the
//! publish system folds into an intent.

use crate::viz::interaction::{
    actions::{
        handle::{ActionHandle, ActionId, ActionMetadata, InputKind},
        registry::ActionRegistry,
    },
    sampling::ActionState,
};

use bevy::prelude::*;

/// Teleop action ids, a negative and a positive key per driven axis.
pub const SURGE_BACK: ActionId = ActionId("teleop.surge_back");
pub const SURGE_FORWARD: ActionId = ActionId("teleop.surge_forward");
pub const YAW_RIGHT: ActionId = ActionId("teleop.yaw_right");
pub const YAW_LEFT: ActionId = ActionId("teleop.yaw_left");

/// The group the teleop actions are listed under.
pub const TELEOP_GROUP: &str = "teleop";

/// The `(negative, positive)` action pair driving one intent DOF. [`deflection`]
/// collapses the two held states to a signed axis value — the whole of the
/// per-axis logic, so the intent builder never special-cases a body.
///
/// [`deflection`]: AxisPair::deflection
pub(crate) struct AxisPair {
    pub(super) negative: ActionHandle,
    pub(super) positive: ActionHandle,
}

impl AxisPair {
    /// This axis's signed deflection this frame: `+1` positive held, `-1` negative
    /// held, `0` for neither or both. The `[-1, 1]` contract falls out of the two
    /// booleans; the runtime mapper re-clamps defensively regardless.
    pub(super) fn deflection(&self, state: &ActionState) -> f64 {
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
    pub(super) surge: Option<AxisPair>,
    pub(super) sway: Option<AxisPair>,
    pub(super) heave: Option<AxisPair>,
    pub(super) roll: Option<AxisPair>,
    pub(super) pitch: Option<AxisPair>,
    pub(super) yaw: Option<AxisPair>,
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
        negative: registry.register(SURGE_BACK, axis("Drive back", KeyCode::ArrowDown)),
        positive: registry.register(SURGE_FORWARD, axis("Drive forward", KeyCode::ArrowUp)),
    };
    // Yaw is left-positive (+Z, FLU), so the left key is the positive handle.
    let yaw = AxisPair {
        negative: registry.register(YAW_RIGHT, axis("Turn right", KeyCode::ArrowRight)),
        positive: registry.register(YAW_LEFT, axis("Turn left", KeyCode::ArrowLeft)),
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
        group: TELEOP_GROUP,
        kind: InputKind::Axis,
        default_key,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        for id in [SURGE_FORWARD, SURGE_BACK, YAW_LEFT, YAW_RIGHT] {
            assert!(
                registry.handle(id).is_some(),
                "register_teleop_actions must declare {} at Startup",
                id.0,
            );
        }

        assert!(
            app.world().get_resource::<TeleopActions>().is_some(),
            "register_teleop_actions must insert the TeleopActions handle cache",
        );
    }
}
