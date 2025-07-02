use crate::simulation::core::config::SimulationConfig;
use avian3d::prelude::*;
use bevy::prelude::*;

pub struct WorldSpawnerPlugin;

impl Plugin for WorldSpawnerPlugin {
    fn build(&self, app: &mut App) {
        // The `chain()` isn't necessary here, but doesn't hurt.
        app.add_systems(Startup, (spawn_world, spawn_lighting_and_camera).chain())
            .add_systems(Update, on_mesh_load_add_collider);
    }
}

#[derive(Component)]
struct CreateColliderFromMesh(Handle<Mesh>);

fn spawn_world(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    // mut materials: ResMut<Assets<StandardMaterial>>, // For the visual
    config: Res<SimulationConfig>,
) {
    println!(
        "[SETUP] Commanding world mesh to load from '{}'...",
        config.world.map_file.display()
    );

    // 1. Start loading the specific mesh from the GLTF file.
    //    The `GltfAssetLabel` is a clean way to pick a specific part of the file.
    let mesh_handle: Handle<Mesh> =
        asset_server.load(GltfAssetLabel::Mesh(0).from_asset(config.world.map_file.clone()));

    // 2. Spawn an entity with everything needed for visuals AND physics,
    //    but WITHOUT the actual Collider component yet.
    commands.spawn((
        // The visual components
        SceneRoot(
            asset_server.load(GltfAssetLabel::Scene(0).from_asset(config.world.map_file.clone())),
        ),
        // The physics body
        RigidBody::Static,
        // The marker component that links this entity to the mesh handle
        CreateColliderFromMesh(mesh_handle),
        Name::new("WorldMesh"),
    ));
}

fn on_mesh_load_add_collider(
    mut commands: Commands,
    mut ev_asset: EventReader<AssetEvent<Mesh>>,
    meshes: Res<Assets<Mesh>>,
    // Query for all entities that are waiting for a collider.
    query: Query<(Entity, &CreateColliderFromMesh)>,
) {
    for ev in ev_asset.read() {
        // We only care about the event that signals a mesh has fully loaded.
        if let AssetEvent::LoadedWithDependencies { id } = ev {
            // Check if the loaded mesh is one we're waiting for.
            for (entity, marker) in &query {
                // `marker.0` is our Handle<Mesh>
                if marker.0.id() == *id {
                    println!(
                        "[WORLD] Mesh asset {:?} is loaded. Creating collider...",
                        id
                    );

                    // Now it is SAFE to get the mesh data.
                    if let Some(mesh) = meshes.get(*id) {
                        // Create the collider from the actual mesh data.
                        if let Some(collider) = Collider::trimesh_from_mesh(mesh) {
                            // Add the collider to the entity and remove the marker.
                            commands
                                .entity(entity)
                                .insert(collider)
                                .remove::<CreateColliderFromMesh>();

                            println!(
                                "[WORLD] Successfully attached collider to entity {:?}",
                                entity
                            );
                        }
                    }
                }
            }
        }
    }
}

/// **STARTUP SYSTEM 2:** Spawns lights and a camera. Kept separate for clarity.
fn spawn_lighting_and_camera(mut commands: Commands) {
    // --- Spawn Lighting ---
    commands.spawn(DirectionalLight {
        shadows_enabled: true,
        illuminance: 15_000.0,
        ..default()
    });

    // --- Spawn Camera ---
    // You would add a flycam or other camera controller here.
    // For now, a static camera.
    let camera_transform = Transform::from_xyz(-30.0, 25.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y);
    commands.spawn((
        Camera3d::default(),
        camera_transform,
        GlobalTransform::default(), // Usually added automatically, but good to be explicit
    ));
}
