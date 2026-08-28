//! Mount frames — the runtime side of a topology's named attachment points.
//!
//! Each `[[mount]]` a `TopologyConfig` declares becomes a child frame entity
//! under the body, carrying its FLU pose. `MountFrame` tags that entity with the
//! config `name` so a later builder can resolve an anchor or role reference
//! (`"wheel_fl"`) back to the entity holding the pose.

use bevy::prelude::*;

/// Marks a child frame entity as the named mount it was built from. The `name`
/// is config-authored data that crosses to the visual and plant axes, not a
/// reserved string constant; it is validated when a downstream builder resolves
/// it against the declared mounts, not here.
#[derive(Component, Debug, Clone)]
pub struct MountFrame(pub String);
