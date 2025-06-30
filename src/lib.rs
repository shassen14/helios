// src/lib.rs
pub mod input_systems;
pub mod integrators;
pub mod models {
    pub mod ackermann_vehicle_model;
    pub mod bicycle_kinematic;
}
pub mod rendering {
    pub mod systems;
}
pub mod simulation {
    pub mod collision_groups;
    pub mod components;
    pub mod systems;
    pub mod traits;
    pub mod utils;
}
pub mod vehicles {
    pub mod car_setup;
    pub mod vehicle_params;
}
pub mod controllers {
    pub mod path_follower;
    pub mod simple_go_to;
}
pub mod path_planning {
    pub mod dijkstra;
    pub mod error;
    pub mod grid_planner;
    pub mod grid_utils;
}
