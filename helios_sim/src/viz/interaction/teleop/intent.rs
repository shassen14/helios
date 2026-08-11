//! Build a [`TwistIntent`] from the held drive keys and publish it onto the
//! piloted agent's bus — but only on active input, so releasing the keys lets the
//! teleop command go stale and the arbiter falls back to autonomy. The builder
//! folds over whatever axes are bound; it has no notion of a car.

use crate::brain_bridge::components::TeleopControlled;
use crate::prelude::HostInputPublisher;
use crate::viz::interaction::{
    sampling::ActionState,
    teleop::{AxisPair, TeleopActions},
};

use helios_core::{control::commands::TwistIntent, data::MonotonicTime};
use helios_runtime::channels::control;

use bevy::prelude::*;

/// Fold each bound axis to its signed deflection; an unbound axis (`None`) stays
/// neutral. Pure over its inputs — the Tier-1 seam, and morphology-blind: it
/// visits all six DOF and lets the [`AxisPair`] presence, not any car assumption,
/// decide which carry a value.
pub(crate) fn teleop_intent(state: &ActionState, actions: &TeleopActions) -> TwistIntent {
    let axis = |pair: &Option<AxisPair>| pair.as_ref().map_or(0.0, |p| p.deflection(state));

    TwistIntent {
        surge: axis(&actions.surge),
        sway: axis(&actions.sway),
        heave: axis(&actions.heave),
        roll: axis(&actions.roll),
        pitch: axis(&actions.pitch),
        yaw: axis(&actions.yaw),
    }
}

/// Publish this frame's intent to every teleop-piloted agent's `control::intent`
/// channel, stamped at the current clock.
///
/// A neutral intent — no drive key held — publishes nothing. That is the freshness
/// handoff: with no fresh intent on the bus the mapper's last output ages out, and
/// the arbiter yields to autonomy rather than latching a stale command. The
/// channel is named from `helios_runtime`'s own `control::intent` role, so the
/// reserved string is never spelled here.
pub(crate) fn publish_teleop_intent(
    time: Res<Time>,
    state: Res<ActionState>,
    actions: Res<TeleopActions>,
    agents: Query<Entity, With<TeleopControlled>>,
    mut publisher: HostInputPublisher,
) {
    let intent = teleop_intent(&state, &actions);
    if intent == TwistIntent::neutral() {
        return;
    }

    let now = MonotonicTime(time.elapsed_secs_f64());

    for agent in &agents {
        publisher.publish(agent, control::intent::<TwistIntent>(), intent, now);
    }
}

#[cfg(test)]
mod tests {
    use super::{teleop_intent, AxisPair, TeleopActions};

    use crate::viz::interaction::actions::{
        handle::{ActionId, ActionMetadata, InputKind},
        registry::ActionRegistry,
    };
    use crate::viz::interaction::sampling::ActionState;

    use bevy::prelude::*;

    fn axis_meta() -> ActionMetadata {
        ActionMetadata {
            label: "t",
            group: "teleop",
            kind: InputKind::Axis,
            default_key: KeyCode::ArrowUp,
        }
    }

    /// Mint a real `(negative, positive)` pair. Handles can only come from the
    /// registry (their index is crate-private), so the test registers throwaway
    /// actions to obtain them.
    fn axis_pair(
        registry: &mut ActionRegistry,
        neg_id: &'static str,
        pos_id: &'static str,
    ) -> AxisPair {
        AxisPair {
            negative: registry.register(ActionId(neg_id), axis_meta()),
            positive: registry.register(ActionId(pos_id), axis_meta()),
        }
    }

    #[test]
    fn positive_held_deflects_the_axis_fully() {
        let mut reg = ActionRegistry::default();
        let surge = axis_pair(&mut reg, "t.surge_neg", "t.surge_pos");
        let held = surge.positive;
        let actions = TeleopActions {
            surge: Some(surge),
            ..default()
        };

        let state = ActionState::from_active([held]);
        let intent = teleop_intent(&state, &actions);

        assert_eq!(intent.surge, 1.0);
    }

    #[test]
    fn negative_held_deflects_the_axis_negatively() {
        let mut reg = ActionRegistry::default();
        let yaw = axis_pair(&mut reg, "t.yaw_neg", "t.yaw_pos");
        let held = yaw.negative;
        let actions = TeleopActions {
            yaw: Some(yaw),
            ..default()
        };

        let state = ActionState::from_active([held]);
        let intent = teleop_intent(&state, &actions);

        assert_eq!(intent.yaw, -1.0);
    }

    #[test]
    fn opposing_inputs_cancel_to_zero() {
        let mut reg = ActionRegistry::default();
        let surge = axis_pair(&mut reg, "t.surge_neg", "t.surge_pos");
        let (neg, pos) = (surge.negative, surge.positive);
        let actions = TeleopActions {
            surge: Some(surge),
            ..default()
        };

        let state = ActionState::from_active([neg, pos]);
        let intent = teleop_intent(&state, &actions);

        assert_eq!(intent.surge, 0.0);
    }

    #[test]
    fn unbound_axes_stay_neutral() {
        // Only surge is bound; every other DOF is `None` and must read zero even
        // though the builder visits all six.
        let mut reg = ActionRegistry::default();
        let surge = axis_pair(&mut reg, "t.surge_neg", "t.surge_pos");
        let held = surge.positive;
        let actions = TeleopActions {
            surge: Some(surge),
            ..default()
        };

        let state = ActionState::from_active([held]);
        let intent = teleop_intent(&state, &actions);

        assert_eq!(
            [
                intent.sway,
                intent.heave,
                intent.roll,
                intent.pitch,
                intent.yaw
            ],
            [0.0; 5],
        );
    }
}
