//! The body's frame names, checked at load. A topology mount and a sensor
//! installation each name a frame `agent.<agent>.<name>` beside the spine, so
//! they share one namespace. Only host config can tell two installations under
//! one name from two channels on one mount (which is legitimate), so the check
//! runs here rather than at the pipeline build.

use crate::core::ground_truth::TRUTH_RIG_UNIT;

use helios_core::kernel::identifier::is_level;
use helios_core::spatial::id::{BASE_LINK, MAP, ODOM};

use std::collections::BTreeMap;

/// The frames the estimator and the body root own; no body part may take one.
const SPINE: [&str; 3] = [MAP, ODOM, BASE_LINK];

/// What claims a name, as the errors say it.
const TOPOLOGY_MOUNT: &str = "topology mount";
const SENSOR_INSTALLATION: &str = "sensor installation";

/// Every problem with the body's frame names, one message per problem, in
/// name order: a name that is not one level, a spine name, a sensor
/// installation named like the truth rig (two units under one name), and a
/// name claimed twice, by two topology mounts or by a mount and an
/// installation.
pub(super) fn check_body_frames<'a>(
    topology_mounts: impl IntoIterator<Item = &'a str>,
    sensor_installations: impl IntoIterator<Item = &'a str>,
) -> Vec<String> {
    let mut claims: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for name in topology_mounts {
        claims.entry(name).or_default().push(TOPOLOGY_MOUNT);
    }
    for name in sensor_installations {
        claims.entry(name).or_default().push(SENSOR_INSTALLATION);
    }

    let mut errors = Vec::new();
    for (name, claimants) in &claims {
        let by = claimants.join(" and ");
        if !is_level(name) {
            errors.push(format!(
                "{by} `{name}`: a body frame name is one level (ASCII letters, digits and `_`)"
            ));
        }
        if SPINE.contains(name) {
            errors.push(format!(
                "{by} `{name}`: {SPINE:?} are spine frames, not body parts"
            ));
        }
        if *name == TRUTH_RIG_UNIT && claimants.contains(&SENSOR_INSTALLATION) {
            errors.push(format!(
                "{by} `{name}`: the truth rig publishes under that unit name"
            ));
        }
        if claimants.len() > 1 {
            errors.push(format!(
                "`{name}` is claimed more than once ({by}); each would be the frame \
                 `agent.<agent>.{name}`"
            ));
        }
    }
    errors
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check(mounts: &[&str], installations: &[&str]) -> Vec<String> {
        check_body_frames(mounts.iter().copied(), installations.iter().copied())
    }

    #[test]
    fn distinct_well_formed_names_pass() {
        assert!(check(&["wheel_fl", "wheel_fr"], &["imu", "gps", "rear_lidar"]).is_empty());
    }

    /// A wheel mount and a sensor sharing a name would silently become one
    /// frame.
    #[test]
    fn a_mount_and_an_installation_sharing_a_name_are_refused() {
        let errors = check(&["wheel_fl"], &["wheel_fl"]);

        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains("wheel_fl"), "{errors:?}");
        assert!(errors[0].contains(TOPOLOGY_MOUNT), "{errors:?}");
        assert!(errors[0].contains(SENSOR_INSTALLATION), "{errors:?}");
    }

    #[test]
    fn a_topology_mount_declared_twice_is_refused() {
        let errors = check(&["wheel_fl", "wheel_fl"], &[]);

        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains("wheel_fl"), "{errors:?}");
    }

    /// A body frame fills one level of `agent.<agent>.<name>`; a dotted name
    /// is kept for a device's own sub-frames under its installation.
    #[test]
    fn a_name_of_more_than_one_level_is_refused() {
        let errors = check(&[], &["imu.0", "front gps"]);

        assert_eq!(errors.len(), 2, "{errors:?}");
        assert!(errors.iter().any(|e| e.contains("imu.0")), "{errors:?}");
        assert!(errors.iter().any(|e| e.contains("front gps")), "{errors:?}");
    }

    #[test]
    fn a_spine_name_is_refused() {
        let errors = check(&[BASE_LINK], &[ODOM, MAP]);

        assert_eq!(errors.len(), 3, "{errors:?}");
    }

    #[test]
    fn a_sensor_named_like_the_truth_rig_is_refused() {
        let errors = check(&[], &[TRUTH_RIG_UNIT]);

        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(errors[0].contains("truth rig"), "{errors:?}");
    }
}
