extern crate relaxed_ik_lib;
use relaxed_ik_lib::relaxed_ik;
use nalgebra::{Vector3, Isometry3, UnitQuaternion};
use nalgebra::geometry::{Translation3};
use parry3d_f64::shape;

use std::{io, thread, time};
use crate::relaxed_ik_lib::utils_rust::file_utils::{*};

fn main() {
    // initilize relaxed ik
    let path_to_src = get_path_to_src();
    let default_path_to_setting = path_to_src +  "configs/settings.yaml";
    let mut relaxed_ik = relaxed_ik::RelaxedIK::load_settings(default_path_to_setting.as_str());

    let cube = shape::Cuboid { half_extents: Vector3::new(0.1, 0.1, 0.1) };
    relaxed_ik.vars.env_collision.obstacles.push(Box::new(cube) as Box<dyn shape::Shape>);

    relaxed_ik.vars.env_collision.obstacle_positions.push(Isometry3::from_parts(
        Translation3::new(10.0, 0.5, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0)
    ));


    for i in 0..50{
        for j in 0..relaxed_ik.vars.robot.num_chains {
            // gradually move along the y axis
            relaxed_ik.vars.goal_positions[j] += Vector3::new(0.0, 0.02, 0.0);
        }
        let x = relaxed_ik.solve();
        println!("Joint solutions: {:?}", x);
    }
}
