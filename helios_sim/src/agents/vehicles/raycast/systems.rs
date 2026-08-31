use super::plant::RaycastWheelPlantComponent;
use crate::agents::vehicles::actuation::{apply_body_wrench, resolve_command};
use crate::agents::vehicles::components::ActuationModelComponent;
use crate::core::components::{ActuatorCommandComponent, GroundTruthState};
use crate::core::transforms::{enu_twist_to_body_flu, EnuBodyPose, EnuVector, FluVector};

use helios_core::plant::WheelContact;

use avian3d::prelude::{SpatialQuery, SpatialQueryFilter};
use bevy::prelude::*;

/// Contact friction coefficient, one value for every surface the wheels touch.
///
/// Interim: a single dry-tarmac-like grip stands in until the environment carries
/// per-part friction, at which point μ is read from the struck entity's material —
/// so mud, ice, and water differ, and water offers no traction — and this constant
/// retires. That is the eventual home of the plant's per-contact grip seam.
// TODO: source μ from the struck surface's material instead of this constant.
const WHEEL_FRICTION: f64 = 0.9;

/// Applies the L1 raycast plant each tick: the host half of its two-phase map.
///
/// For every raycast car it resolves the pipeline's command, converts the
/// world-ENU ground truth into the body-FLU twist the plant computes in, casts one
/// downward suspension ray per wheel against the world, and folds the contacts, the
/// twist, and the command into a chassis wrench the shared tail applies. Disjoint by
/// query from the L0 shim's applier, so both share `SimulationSet::Actuation` with
/// no ordering between them.
pub(super) fn drive_raycast_cars(
    mut commands: Commands,
    spatial_query: SpatialQuery,
    query: Query<(
        Entity,
        &GlobalTransform,
        &GroundTruthState,
        &RaycastWheelPlantComponent,
        &ActuationModelComponent,
        Option<&ActuatorCommandComponent>,
    )>,
) {
    for (entity, body, truth, plant, model, command) in &query {
        let Some(resolved) = resolve_command(&model.0, command) else {
            continue;
        };

        // Ground truth reports velocity in world ENU; the plant computes in the
        // body FLU frame, so rotate the twist through the body pose first.
        let twist = enu_twist_to_body_flu(
            EnuBodyPose(truth.pose),
            EnuVector(truth.linear_velocity),
            EnuVector(truth.angular_velocity),
        );

        // A contact slice sized to the wheel count and indexed by wheel: `None`
        // where a ray finds no ground. It must stay index-aligned, not push-order —
        // `compute_wrench` zips it positionally against its wheels, so a hit written
        // anywhere but `ray.wheel` would apply that force to the wrong corner.
        let rays = plant.0.generate_rays();
        let filter = SpatialQueryFilter::from_excluded_entities([entity]);
        let mut contacts: Vec<Option<WheelContact>> = (0..rays.len()).map(|_| None).collect();

        for ray in &rays {
            // The origin is a body point (picks up the body's world translation);
            // the direction is a free vector (rotation only). Casting into Avian and
            // reading the hit are the one f64↔f32 crossing — core stays f64.
            let origin = body.transform_point(Vec3::from(FluVector(*ray.origin.raw())));
            let direction = body.rotation() * Vec3::from(FluVector(*ray.direction.raw()));
            let Ok(dir) = Dir3::new(direction) else {
                continue;
            };
            if let Some(hit) =
                spatial_query.cast_ray(origin, dir, ray.max_distance as f32, true, &filter)
            {
                contacts[ray.wheel] = Some(WheelContact::new(hit.distance as f64, WHEEL_FRICTION));
            }
        }

        let wrench = plant.0.compute_wrench(&contacts, twist, &resolved);
        apply_body_wrench(&mut commands, entity, body.rotation(), &wrench);
    }
}
