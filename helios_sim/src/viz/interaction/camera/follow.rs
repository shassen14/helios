//! Focus resolution: the half of the camera that turns a [`CameraTarget`] *mode*
//! into the rig's concrete focus point each frame.
//!
//! [`retarget_on_selection`] reacts to a new [`Selected`] by switching the camera
//! into [`CameraTarget::Follow`]; [`resolve_focus`] then copies the followed
//! entity's world position into [`CameraRig::focus`] every frame. Both run
//! upstream of `apply_camera_intent`, so the focus is already correct before the
//! orbit/zoom/pan deltas swing the rig around it.
//!
//! The copy is a plain `Vec3` assignment: the rig lives in Bevy Y-up and so does
//! `GlobalTransform`, so nothing here crosses the ENU↔Bevy boundary and no
//! `core/transforms` conversion belongs in this file.

use bevy::prelude::*;

use crate::viz::interaction::{
    camera::{rig::CameraRig, CameraTarget},
    selection::Selected,
};

/// Switches the camera to follow a newly [`Selected`] entity.
///
/// Fires only on the selection *edge* (`Added<Selected>`), so after locking on you
/// can still orbit and zoom freely without the focus being re-grabbed each frame.
/// A pan gesture releases the follow, handled downstream in `apply_camera_intent`.
pub(crate) fn retarget_on_selection(
    just_selected: Query<Entity, Added<Selected>>,
    mut cams: Query<&mut CameraTarget>,
) {
    for entity in &just_selected {
        for mut target in &mut cams {
            *target = CameraTarget::Follow(entity);
        }
    }
}

/// Copies the followed entity's world position into the rig's focus each frame,
/// and reverts to [`CameraTarget::Fixed`] if that entity has despawned.
///
/// `Fixed` is left untouched — its focus is under manual (pan) control. The
/// despawn revert is the load-bearing case: a follow target can vanish mid-run,
/// and without the fallback the camera would chase a dangling `Entity`.
pub(crate) fn resolve_focus(
    mut cams: Query<(&mut CameraRig, &mut CameraTarget)>,
    transforms: Query<&GlobalTransform>,
) {
    for (mut rig, mut target) in &mut cams {
        let CameraTarget::Follow(entity) = *target else {
            continue;
        };

        match transforms.get(entity) {
            Ok(gt) => rig.focus = gt.translation(),
            Err(_) => *target = CameraTarget::Fixed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A rig at a known, non-trivial pose so a focus overwrite is observable.
    fn test_rig() -> CameraRig {
        CameraRig {
            focus: Vec3::ZERO,
            distance: 10.0,
            yaw: 0.0,
            pitch: 0.3,
        }
    }

    /// Tier 2: a `Follow` camera copies its target's translation into the focus.
    #[test]
    fn following_copies_target_translation_into_focus() {
        let mut app = App::new();
        let target = app
            .world_mut()
            .spawn(GlobalTransform::from_translation(Vec3::new(4.0, 5.0, 6.0)))
            .id();
        let cam = app
            .world_mut()
            .spawn((test_rig(), CameraTarget::Follow(target)))
            .id();

        app.add_systems(Update, resolve_focus);
        app.update();

        assert_eq!(
            app.world().get::<CameraRig>(cam).unwrap().focus,
            Vec3::new(4.0, 5.0, 6.0),
        );
    }

    /// Tier 2: the load-bearing edge case — a despawned follow target drops the
    /// camera back to `Fixed` instead of chasing a dangling entity.
    #[test]
    fn a_despawned_target_reverts_to_fixed() {
        let mut app = App::new();
        let target = app
            .world_mut()
            .spawn(GlobalTransform::from_translation(Vec3::ONE))
            .id();
        let cam = app
            .world_mut()
            .spawn((test_rig(), CameraTarget::Follow(target)))
            .id();

        app.world_mut().despawn(target);

        app.add_systems(Update, resolve_focus);
        app.update();

        assert_eq!(
            *app.world().get::<CameraTarget>(cam).unwrap(),
            CameraTarget::Fixed,
        );
    }

    /// Tier 2: selecting an entity switches the camera into `Follow`.
    #[test]
    fn selecting_an_entity_makes_the_camera_follow_it() {
        let mut app = App::new();
        let cam = app.world_mut().spawn(CameraTarget::Fixed).id();
        let target = app.world_mut().spawn(Selected).id();

        app.add_systems(Update, retarget_on_selection);
        app.update();

        assert_eq!(
            *app.world().get::<CameraTarget>(cam).unwrap(),
            CameraTarget::Follow(target),
        );
    }
}
