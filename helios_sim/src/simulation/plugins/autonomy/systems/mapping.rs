// helios_sim/src/simulation/plugins/autonomy/systems/mapping.rs
//
// Mapping system: forwards sensor mailbox to mappers, handles timer-gated pose updates.

use bevy::prelude::*;
use std::collections::HashMap;

use crate::simulation::core::components::SensorMailbox;
use crate::simulation::core::sim_runtime::SimRuntime;
use crate::simulation::core::transforms::TfTree;
use crate::simulation::plugins::autonomy::components::{MapperComponent, ModuleTimer, OdomFrameOf};

/// Forwards each agent's `SensorMailbox` to its mapper and handles timer-gated pose updates.
/// Runs in parallel with `run_estimation` — accesses `MapperComponent` only.
pub fn run_mapping(
    mut module_query: Query<(
        Entity,
        &mut MapperComponent,
        &mut ModuleTimer,
        &SensorMailbox,
    )>,
    odom_query: Query<(&OdomFrameOf, Entity)>,
    time: Res<Time>,
    tf_tree: Res<TfTree>,
) {
    let runtime = SimRuntime {
        tf: &*tf_tree,
        elapsed_secs: time.elapsed_secs_f64(),
    };

    let agent_to_odom_iso: HashMap<Entity, nalgebra::Isometry3<f64>> = odom_query
        .iter()
        .filter_map(|(odom_of, odom_entity)| {
            tf_tree
                .lookup_by_entity(odom_entity)
                .map(|iso| (odom_of.0, iso))
        })
        .collect();

    for (agent_entity, mut mapper, mut timer, mailbox) in &mut module_query {
        timer.0.tick(time.delta());

        let messages: Vec<_> = mailbox.entries.iter().map(|e| e.message.clone()).collect();
        mapper.0.process_messages(&messages, &runtime);

        if timer.0.just_finished() {
            if let Some(&odom_iso) = agent_to_odom_iso.get(&agent_entity) {
                mapper.0.process_pose_update(odom_iso);
            }
        }
    }
}
