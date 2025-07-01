// src/estimation/ekf_plugin.rs
use crate::simulation::components::DynamicsModel;
use crate::simulation::topics::{AppliedControl, EstimatedPose, ImuData};
use bevy::prelude::*; // From your old components file

pub struct EkfPlugin;

// Define a SystemSet for ordering EKF systems
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct EkfSystemSet;

impl Plugin for EkfPlugin {
    fn build(&self, app: &mut App) {
        // The EKF's private internal state is a component.
        // It gets added to an agent when the EKF is selected for it.
        app.register_type::<EkfState>();

        app.configure_sets(Update, EkfSystemSet.run_if(/* run at filter frequency */));

        app.add_systems(
            Update,
            (
                ekf_predict_system,
                ekf_update_from_imu.after(ekf_predict_system),
                ekf_write_output.after(ekf_update_from_imu),
            )
                .in_set(EkfSystemSet),
        );
    }
}

#[derive(Component, Default)]
pub struct EkfState {/* The filter's state vector, covariance, etc. */}

#[derive(Component)]
pub struct EKF; // A simple marker component

fn ekf_predict_system(
    mut query: Query<(&mut EkfState, &DynamicsModel), With<EKF>>,
    mut control_reader: EventReader<AppliedControl>,
) { /* ... */
}

fn ekf_update_from_imu(
    mut query: Query<&mut EkfState, With<EKF>>,
    mut imu_reader: EventReader<ImuData>,
) { /* ... */
}

fn ekf_write_output(mut query: Query<(&mut EstimatedPose, &EkfState), With<EKF>>) { /* ... */
}
