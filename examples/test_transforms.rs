use bevy::math::Quat as BevyQuat; // BevyQuat is f32 based, from bevy::math not prelude for standalone
use nalgebra::{Quaternion, UnitQuaternion, Vector3}; // Added Quaternion
use std::f64::consts::FRAC_PI_2;

thread_local! {
    // Define only one fundamental frame rotation quaternion
    pub static Q_BEVY_FRAME_FROM_ENU_FRAME: UnitQuaternion<f64> =
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -FRAC_PI_2);
}

pub fn enu_quat_to_bevy_quat(enu_quat: &UnitQuaternion<f64>) -> BevyQuat {
    let final_rot_f64 = Q_BEVY_FRAME_FROM_ENU_FRAME.with(|q_bf_ef| {
        let q_ef_bf = q_bf_ef.inverse(); // This is T_ENU_from_Bevy
        *q_bf_ef * enu_quat * q_ef_bf // T_B_E * q_E * T_E_B
    });

    BevyQuat::from_xyzw(
        final_rot_f64.coords.x as f32,
        final_rot_f64.coords.y as f32,
        final_rot_f64.coords.z as f32,
        final_rot_f64.coords.w as f32,
    )
}

pub fn bevy_quat_to_enu_quat(bevy_quat: &BevyQuat) -> UnitQuaternion<f64> {
    // Ensure BevyQuat fields are correctly mapped to nalgebra Quaternion constructor (w, x, y, z)
    let bevy_q_f64 = UnitQuaternion::from_quaternion(Quaternion::new(
        bevy_quat.w as f64, // w first for nalgebra::Quaternion::new
        bevy_quat.x as f64,
        bevy_quat.y as f64,
        bevy_quat.z as f64,
    ));

    let final_rot_f64 = Q_BEVY_FRAME_FROM_ENU_FRAME.with(|q_bf_ef| {
        let q_ef_bf = q_bf_ef.inverse(); // This is T_ENU_from_Bevy
        q_ef_bf * bevy_q_f64 * *q_bf_ef // T_E_B * q_B * T_B_E
    });
    final_rot_f64
}

fn main() {
    println!("--- Testing ENU Identity to Bevy ---");
    let enu_identity = UnitQuaternion::identity();
    let bevy_from_enu_identity = enu_quat_to_bevy_quat(&enu_identity);
    println!(
        "ENU Identity Quat Components (x,y,z,w): {:?}",
        enu_identity.coords
    );
    println!(
        "Bevy from ENU Identity (EXPECT Bevy IDENTITY): {:?}",
        bevy_from_enu_identity
    );

    println!("\n--- Testing Bevy Identity to ENU ---");
    let bevy_identity = BevyQuat::IDENTITY;
    let enu_from_bevy_identity = bevy_quat_to_enu_quat(&bevy_identity);
    println!("Bevy Identity: {:?}", bevy_identity);
    println!(
        "ENU from Bevy Identity (EXPECT ENU IDENTITY) Quat Components (x,y,z,w): {:?}",
        enu_from_bevy_identity.coords
    );

    println!("\n--- Testing ENU Yaw (90 deg around Z_up) to Bevy ---");
    let enu_yaw_90 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
    let bevy_from_enu_yaw_90 = enu_quat_to_bevy_quat(&enu_yaw_90);
    let expected_bevy_yaw_90 = BevyQuat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    println!(
        "ENU Yaw 90 deg Quat Components (x,y,z,w): {:?}",
        enu_yaw_90.coords
    );
    println!("Bevy from ENU Yaw 90 deg: {:?}", bevy_from_enu_yaw_90);
    println!("Expected Bevy Yaw 90 deg: {:?}", expected_bevy_yaw_90);

    println!("\n--- Testing Bevy Yaw (90 deg around Y_up) to ENU ---");
    let bevy_yaw_90_for_test = BevyQuat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    let enu_from_bevy_yaw_90 = bevy_quat_to_enu_quat(&bevy_yaw_90_for_test);
    let expected_enu_yaw_90 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
    println!("Bevy Yaw 90 deg: {:?}", bevy_yaw_90_for_test);
    println!(
        "ENU from Bevy Yaw 90 deg Quat Components (x,y,z,w): {:?}",
        enu_from_bevy_yaw_90.coords
    );
    println!(
        "Expected ENU Yaw 90 deg Quat Components (x,y,z,w): {:?}",
        expected_enu_yaw_90.coords
    );

    // Test the Q_BEVY_FRAME_FROM_ENU_FRAME application to basis vectors
    Q_BEVY_FRAME_FROM_ENU_FRAME.with(|r_be| {
        let enu_x = Vector3::x();
        let enu_y = Vector3::y();
        let enu_z = Vector3::z();

        // This is NOT conjugation. This is transforming a vector's coordinates.
        let bevy_coords_of_enu_x = r_be * enu_x; // Should be (1,0,0)
        let bevy_coords_of_enu_y = r_be * enu_y; // Should be (0,0,-1)
        let bevy_coords_of_enu_z = r_be * enu_z; // Should be (0,1,0)

        println!("\n--- Frame Transformation of Basis Vectors (Coordinates) ---");
        println!(
            "Bevy Coords of ENU_X (EXPECT Bevy X = (1,0,0)): {:?}",
            bevy_coords_of_enu_x
        );
        println!(
            "Bevy Coords of ENU_Y (EXPECT Bevy -Z = (0,0,-1)): {:?}",
            bevy_coords_of_enu_y
        );
        println!(
            "Bevy Coords of ENU_Z (EXPECT Bevy Y = (0,1,0)): {:?}",
            bevy_coords_of_enu_z
        );
    });
}

