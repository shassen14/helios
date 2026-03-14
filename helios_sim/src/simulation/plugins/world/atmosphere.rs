use avian3d::prelude::Gravity;
use bevy::prelude::*;
use bevy_fly_camera::{FlyCamera, FlyCameraPlugin};

use crate::prelude::*;

pub struct AtmospherePlugin;

impl Plugin for AtmospherePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FlyCameraPlugin).add_systems(
            OnEnter(AppState::SceneBuilding),
            (configure_gravity, spawn_sun, spawn_camera),
        );
    }
}

fn configure_gravity(config: Res<ScenarioConfig>, mut gravity: ResMut<Gravity>) {
    let g = config.world.atmosphere.gravity;
    gravity.0 = Vec3::new(g[0], g[1], g[2]);
    info!("[Atmosphere] Gravity set to {:?}", gravity.0);
}

fn spawn_sun(mut commands: Commands, config: Res<ScenarioConfig>) {
    let atmos = &config.world.atmosphere;
    let light_transform = sun_transform(atmos.sun_elevation, atmos.sun_azimuth);
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: atmos.ambient_lux,
            ..default()
        },
        light_transform,
    ));
    info!(
        "[Atmosphere] Sun: elevation={:.1}°, azimuth={:.1}°, illuminance={:.0} lx",
        atmos.sun_elevation, atmos.sun_azimuth, atmos.ambient_lux
    );
}

fn spawn_camera(mut commands: Commands) {
    let transform = Transform::from_xyz(-30.0, 25.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y);
    let euler = transform.rotation.to_euler(EulerRot::YZX);
    commands
        .spawn((Camera3d::default(), transform))
        .insert(FlyCamera {
            pitch: euler.2,
            yaw: euler.0,
            ..Default::default()
        });
}

/// Builds a Bevy Transform for a directional light at the given sun position.
/// Elevation is degrees above horizon; azimuth is degrees clockwise from North.
fn sun_transform(elevation_deg: f32, azimuth_deg: f32) -> Transform {
    let elev = elevation_deg.to_radians();
    let az = azimuth_deg.to_radians();

    // Sun direction in ENU: east = sin(az)*cos(el), north = cos(az)*cos(el), up = sin(el)
    // Convert ENU → Bevy: east→+X, up→+Y, north→-Z
    let sun_pos = Vec3::new(az.sin() * elev.cos(), elev.sin(), -(az.cos() * elev.cos())) * 100.0;

    Transform::from_translation(sun_pos).looking_at(Vec3::ZERO, Vec3::Y)
}
