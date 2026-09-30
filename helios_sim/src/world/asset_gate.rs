//! The one gate out of `AssetLoading`: the scene is built once every world
//! asset has loaded, and the run fails, naming each, when any cannot.

use super::layout::ObjectAssets;
use super::terrain::TerrainAssets;
use crate::core::app_state::AssetLoadSet;
use crate::prelude::*;

use bevy::asset::{LoadState, UntypedAssetId};

/// Moves to `SceneBuilding` when every terrain tile and object prefab has
/// loaded.
pub struct AssetGatePlugin;

impl Plugin for AssetGatePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            finish_asset_loading
                .in_set(AssetLoadSet::Check)
                .run_if(in_state(AppState::AssetLoading)),
        );
    }
}

/// Where loading stands across every tracked asset.
#[derive(Debug, PartialEq)]
enum LoadProgress {
    Waiting,
    Ready,
    /// Each failed asset's label and why it failed.
    Failed(Vec<String>),
}

/// A failed asset ends loading at once, since waiting cannot fix it; a world
/// with nothing to load is ready immediately.
fn load_progress(states: impl IntoIterator<Item = (String, Option<LoadState>)>) -> LoadProgress {
    let mut waiting = false;
    let mut failures = Vec::new();
    for (label, state) in states {
        match state {
            Some(LoadState::Loaded) => {}
            Some(LoadState::Failed(error)) => failures.push(format!("{label}: {error}")),
            Some(LoadState::NotLoaded | LoadState::Loading) | None => waiting = true,
        }
    }
    if !failures.is_empty() {
        LoadProgress::Failed(failures)
    } else if waiting {
        LoadProgress::Waiting
    } else {
        LoadProgress::Ready
    }
}

fn finish_asset_loading(
    mut next_state: ResMut<NextState<AppState>>,
    asset_server: Res<AssetServer>,
    terrain: Res<TerrainAssets>,
    objects: Res<ObjectAssets>,
) {
    let state = |(label, id): (String, UntypedAssetId)| (label, asset_server.get_load_state(id));
    let tracked = terrain.tracked().chain(objects.tracked()).map(state);

    match load_progress(tracked) {
        LoadProgress::Waiting => {}
        LoadProgress::Ready => {
            info!("[Assets] All world assets loaded. Transitioning to SceneBuilding.");
            next_state.set(AppState::SceneBuilding);
        }
        LoadProgress::Failed(failures) => {
            panic!("world assets failed to load:\n{}", failures.join("\n"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::asset::AssetLoadError;
    use std::sync::Arc;

    fn failed() -> LoadState {
        LoadState::Failed(Arc::new(AssetLoadError::CannotLoadIgnoredAsset {
            path: "objects/crate_1m.glb".into(),
        }))
    }

    #[test]
    fn nothing_to_load_is_ready() {
        // An objects-only or empty world must not wait for terrain.
        assert_eq!(load_progress([]), LoadProgress::Ready);
    }

    #[test]
    fn waits_while_any_asset_is_loading() {
        let states = [
            ("a".to_string(), Some(LoadState::Loaded)),
            ("b".to_string(), Some(LoadState::Loading)),
        ];
        assert_eq!(load_progress(states), LoadProgress::Waiting);
    }

    #[test]
    fn ready_when_every_asset_has_loaded() {
        let states = [
            ("a".to_string(), Some(LoadState::Loaded)),
            ("b".to_string(), Some(LoadState::Loaded)),
        ];
        assert_eq!(load_progress(states), LoadProgress::Ready);
    }

    #[test]
    fn a_failure_ends_loading_naming_every_failed_asset() {
        // Fails even while another asset is still loading: waiting for it
        // would only delay the report.
        let states = [
            ("prefab `a`".to_string(), Some(failed())),
            ("b".to_string(), Some(LoadState::Loading)),
            ("prefab `c`".to_string(), Some(failed())),
        ];
        let LoadProgress::Failed(failures) = load_progress(states) else {
            panic!("expected a failure");
        };
        assert_eq!(failures.len(), 2);
        assert!(failures[0].starts_with("prefab `a`: "), "{failures:?}");
        assert!(failures[1].starts_with("prefab `c`: "), "{failures:?}");
    }
}
