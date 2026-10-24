//! Loads each prefab's `.glb` once, keeping the glTF document beside Bevy's
//! scene so the geometry can be derived from exactly what was exported.

use super::resolved::ResolvedWorldLayout;

use bevy::asset::UntypedAssetId;
use bevy::gltf::convert_coordinates::GltfConvertCoordinates;
use bevy::gltf::{Gltf, GltfLoaderSettings};
use bevy::prelude::*;
use std::path::PathBuf;

/// One loading `.glb` per prefab, in the order of the layout's
/// [`prefabs`](ResolvedWorldLayout::prefabs).
#[derive(Resource, Default)]
pub struct ObjectAssets {
    entries: Vec<ObjectAsset>,
}

impl ObjectAssets {
    /// The loaded file of the prefab at `prefab_index` in the layout.
    pub fn gltf(&self, prefab_index: usize) -> Option<&Handle<Gltf>> {
        self.entries.get(prefab_index).map(|entry| &entry.gltf)
    }

    /// Every asset to wait for, labelled by prefab and file for a failed
    /// load's report.
    pub fn tracked(&self) -> impl Iterator<Item = (String, UntypedAssetId)> + '_ {
        self.entries.iter().map(|entry| {
            (
                format!("prefab `{}` ({})", entry.prefab, entry.mesh.display()),
                entry.gltf.id().untyped(),
            )
        })
    }
}

struct ObjectAsset {
    prefab: String,
    mesh: PathBuf,
    gltf: Handle<Gltf>,
}

/// Starts loading every prefab the layout places.
pub(super) fn load_object_assets(
    layout: Option<Res<ResolvedWorldLayout>>,
    asset_server: Res<AssetServer>,
    mut assets: ResMut<ObjectAssets>,
) {
    let Some(layout) = layout else {
        return;
    };
    assets.entries = layout
        .prefabs()
        .iter()
        .map(|prefab| ObjectAsset {
            prefab: prefab.key.clone(),
            mesh: prefab.mesh.clone(),
            gltf: asset_server
                .load_builder()
                .with_settings(loader_settings)
                .load(prefab.mesh.clone()),
        })
        .collect();
}

/// Keeps the glTF document for geometry derivation, and pins coordinate
/// conversion off so the rendered scene and the derived geometry share
/// glTF's axes whatever the app-wide default.
fn loader_settings(settings: &mut GltfLoaderSettings) {
    settings.include_source = true;
    settings.convert_coordinates = Some(GltfConvertCoordinates::default());
}
