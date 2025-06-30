// src/simulation/collision_groups.rs (or src/simulation/mod.rs)
use avian3d::prelude::*;

#[derive(PhysicsLayer, Default)]
pub enum Layer {
    // --- Major Categories ---
    /// Ground, terrain, roads, static large structures like buildings, bridges.
    /// Typically collides with most dynamic things.
    #[default]
    WorldStatic,
    /// General dynamic obstacles: smaller static items (poles, fences, bushes if solid),
    /// or larger dynamic items not fitting other categories.
    DynamicObstacle,

    // --- Vehicle Categories (based on movement domain & general size/interaction) ---
    /// Cars, trucks, ground-based robots (humanoids, dogs on ground).
    GroundVehicle,
    /// Drones, planes.
    FlyingVehicle,
    /// Boats, surface ships.
    SurfaceMarineVehicle,
    /// Submarines, UUVs.
    UnderwaterVehicle,

    // --- Other Dynamic Entities ---
    /// Humanoids, animals (if they have distinct interaction patterns from vehicles).
    Character,
    /// Robotic arms, manipulators (might have very specific collision needs).
    Manipulator, // Could also be part of its parent GroundVehicle/Character

    // --- Special Purpose Layers ---
    /// For sensor rays (Lidar, Radar) to selectively ignore certain objects
    /// (e.g., other sensor rays, transparent objects, or self-collision for rays from same sensor).
    SensorRay,
    /// For "trigger" volumes or non-colliding detection zones.
    DetectionZone,
    /// For fluid volumes (water, lakes, oceans) if you plan to simulate buoyancy
    /// or different drag. This might not be a simple collision layer but a region query.
    FluidVolume,
    // --- Potentially Finer Grained Vehicle Categories (if needed later) ---
    // SmallGroundVehicle (e.g., RC cars, small delivery bots)
    // LargeGroundVehicle (e.g., 18-wheelers, construction equipment)
    // This level of detail might be overkill initially. Start broad.
}

// // --- Constants for Layer Groups (Bitmasks) ---
//
// /// Represents all layers that are typically considered "dynamic" and can move or be interacted with.
// pub const ALL_DYNAMIC_THINGS_MASK: u32 = Layer::DynamicObstacle.to_bits()
//     | Layer::GroundVehicle.to_bits()
//     | Layer::FlyingVehicle.to_bits()
//     | Layer::SurfaceMarineVehicle.to_bits()
//     | Layer::UnderwaterVehicle.to_bits()
//     | Layer::Character.to_bits()
//     | Layer::Manipulator.to_bits();
//
// /// Represents all layers that form the static environment.
// pub const ALL_WORLD_STATIC_MASK: u32 = Layer::WorldStatic.to_bits();
// // If you had other static categories like "StaticProps", you'd add them here.
//
// /// Represents all types of vehicles.
// pub const ALL_VEHICLES_MASK: u32 = Layer::GroundVehicle.to_bits()
//     | Layer::FlyingVehicle.to_bits()
//     | Layer::SurfaceMarineVehicle.to_bits()
//     | Layer::UnderwaterVehicle.to_bits();
//
// // --- Or using functions (can be more flexible if logic is involved) ---
// // pub fn all_dynamic_things_mask() -> u32 {
// //     Layer::DynamicObstacle.to_bits()
// //     | Layer::GroundVehicle.to_bits()
// //     // ... etc.
// // }
//
// // You can also define a constant for "everything physical"
// pub const ALL_PHYSICAL_MASK: u32 = ALL_DYNAMIC_THINGS_MASK | ALL_WORLD_STATIC_MASK;
// // Note: This might be too broad for many filters. Be specific where possible.
//
// // Avian also provides `Group::ALL` which is `u32::MAX`, but making your own
// // "all physical" can be useful if you have non-physical layers like SensorRay.
