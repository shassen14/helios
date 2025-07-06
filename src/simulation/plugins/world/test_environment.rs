use avian3d::prelude::*;
use bevy::prelude::*;

/// A simple plugin that spawns a basic environment for physics testing.
pub struct TestEnvironmentPlugin;

impl Plugin for TestEnvironmentPlugin {
    fn build(&self, app: &mut App) {
        // This single startup system creates our entire test scene.
        app.add_systems(Startup, spawn_test_environment);
    }
}

/// This system creates a ground plane, a light, and a camera.
fn spawn_test_environment(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    info!("[SETUP] Spawning minimal test environment.");

    // --- Ground Plane ---
    let (ground_length, ground_width) = (500.0, 500.0);
    let ground_mesh_handle =
        meshes.add(Plane3d::default().mesh().size(ground_length, ground_width));
    let ground_material_handle = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.5, 0.3),
        ..default()
    });
    commands.spawn((
        Mesh3d(ground_mesh_handle),
        MeshMaterial3d(ground_material_handle),
        Transform::from_xyz(0.0, 0.0, 0.0),
        RigidBody::Static,
        // CollisionLayers::new(
        //     Layer::WorldStatic,
        //     Layer::Character.to_bits() | Layer::GroundVehicle.to_bits(),
        // ),
        Collider::cuboid(ground_length, 0.01, ground_width),
        // CollisionLayers::new(Group::ENVIRONMENT, GROUP::ALL), // Define collision groups (optional but good practice)
        Name::new("Ground"),
    ));

    // --- Lighting ---
    // A strong directional light to illuminate the scene clearly.
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 15000.0,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.8, -0.5, 0.0)),
        Name::new("SunLight"),
    ));

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
