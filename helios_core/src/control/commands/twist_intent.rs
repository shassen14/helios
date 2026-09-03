//! Normalised operator drive intent for the twist command family.

use serde::{Deserialize, Serialize};

/// Normalised operator drive intent: a dimensionless deflection in `[-1, 1]` on
/// each of the six body FLU motion DOF, before any scaling into physical units
/// or framing into a command space.
///
/// Intent carries **no units and no gains**. `1.0` means "full deflection on
/// this axis," not a velocity; turning that into a physical command — and
/// deciding which axes a given body actually drives — is the mapper's job, so
/// the same intent serves every velocity-commanded body. A body that cannot move
/// on an axis (a car has no `sway`) simply leaves it zero.
///
/// Continuous motion only. Discrete operator actions — taking or releasing
/// control, mode switches — are a separate concern and do not belong here.
///
/// Fields: `surge` (+X forward), `sway` (+Y left), `heave` (+Z up); `roll` (+X),
/// `pitch` (+Y), `yaw` (+Z, left-positive). Each contracted to `[-1, 1]`; call
/// [`clamped`](Self::clamped) to enforce it.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TwistIntent {
    pub surge: f64,
    pub sway: f64,
    pub heave: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl TwistIntent {
    /// The resting intent: no deflection on any axis. The host publishes this
    /// when no operator input is present, so a released control surrenders to
    /// arbitration rather than latching its last command.
    pub fn neutral() -> Self {
        Self {
            surge: 0.0,
            sway: 0.0,
            heave: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }

    /// This intent with every axis clamped into the contracted `[-1, 1]` range.
    /// The host applies it before publishing so a miscalibrated device cannot
    /// drive an out-of-range deflection downstream.
    pub fn clamped(self) -> Self {
        Self {
            surge: self.surge.clamp(-1.0, 1.0),
            sway: self.sway.clamp(-1.0, 1.0),
            heave: self.heave.clamp(-1.0, 1.0),
            roll: self.roll.clamp(-1.0, 1.0),
            pitch: self.pitch.clamp(-1.0, 1.0),
            yaw: self.yaw.clamp(-1.0, 1.0),
        }
    }
}

impl Default for TwistIntent {
    fn default() -> Self {
        Self::neutral()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn neutral_is_zero_on_every_axis() {
        let n = TwistIntent::neutral();
        assert_eq!([n.surge, n.sway, n.heave, n.roll, n.pitch, n.yaw], [0.0; 6]);
    }

    #[test]
    fn default_is_neutral() {
        assert_eq!(TwistIntent::default(), TwistIntent::neutral());
    }

    #[test]
    fn clamped_bounds_each_axis_to_the_unit_range() {
        let wild = TwistIntent {
            surge: 2.5,
            sway: -3.0,
            heave: 0.5,
            roll: -0.5,
            pitch: 10.0,
            yaw: -1.0,
        };
        let c = wild.clamped();
        assert_eq!(
            [c.surge, c.sway, c.heave, c.roll, c.pitch, c.yaw],
            [1.0, -1.0, 0.5, -0.5, 1.0, -1.0]
        );
    }

    #[test]
    fn clamped_leaves_in_range_intent_untouched() {
        let ok = TwistIntent {
            surge: 0.5,
            sway: -1.0,
            heave: 0.0,
            roll: 1.0,
            pitch: -0.25,
            yaw: 0.75,
        };
        assert_eq!(ok.clamped(), ok);
    }
}
