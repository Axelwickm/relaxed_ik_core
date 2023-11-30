use nalgebra::{Vector3, Isometry3, Point3};
use nalgebra::geometry::{Translation3, UnitQuaternion, Quaternion};
use parry3d_f64::{shape, query};

use std::collections::BTreeMap;

use crate::spacetime::robot::Robot;

#[derive(Clone, Debug)]
pub struct LinkData {
    pub is_link: bool,
    pub arm_idx: i32,
}

impl LinkData {
    pub fn new(is_link: bool, arm_idx: i32) -> LinkData {
        Self {
            is_link: is_link,
            arm_idx: arm_idx,
        }
    }
}

#[derive(Clone, Debug)]
pub struct CollisionObjectData {
    pub name: String,
    pub link_data: LinkData,
}

impl CollisionObjectData {
    pub fn new(name: String, link_data: LinkData) -> CollisionObjectData {
        Self {
            name: name,
            link_data: link_data,
        }
    }
}

pub struct RelaxedIKEnvCollision {
    pub obstacles: Vec<Box<dyn shape::Shape>>,
    pub obstacle_positions: Vec<Isometry3<f64>>,
    // pub world: CollisionWorld<f64, CollisionObjectData>,
    // pub link_radius: f64,
    // pub link_handles: Vec<Vec<CollisionObjectSlabHandle>>,
    // pub dyn_obstacle_handles: Vec<(CollisionObjectSlabHandle, String)>,
    // pub active_pairs: Vec<BTreeMap<CollisionObjectSlabHandle, Vec<CollisionObjectSlabHandle>>>,
    // pub active_obstacles: Vec<Vec<(Option<CollisionObjectSlabHandle>, f64)>>,
}

impl RelaxedIKEnvCollision {
    pub fn init_collision_world (
        robot: &Robot,
    ) -> Self {
        let mut obstacles = Vec::new();
        let mut obstacle_positions = Vec::new();

        let cube = shape::Cuboid { half_extents: Vector3::new(0.1, 0.1, 0.1) };
        obstacles.push(Box::new(cube) as Box<dyn shape::Shape>);

        obstacle_positions.push(Isometry3::from_parts(
            Translation3::new(10.0, 0.5, 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0)
        ));


        // let link_radius = 0.05;
        // let plane_obstacles = Vec::new();
        // let sphere_obstacles = Vec::new();
        // let pcd_obstacles = Vec::new();
        //
        // // The links are part of group 1 and can only interact with obstacles
        // let mut link_groups = ncollide3d::pipeline::object::CollisionGroups::new();
        // link_groups.set_membership(&[1]);
        // link_groups.set_blacklist(&[1]);
        // link_groups.set_whitelist(&[2]);
        //
        // // All the other objects are part of the group 2 and interact only with the links
        // let mut others_groups = ncollide3d::pipeline::object::CollisionGroups::new();
        // others_groups.set_membership(&[2]);
        // others_groups.set_blacklist(&[2]);
        // others_groups.set_whitelist(&[1]);
        //
        // let proximity_query = ncollide3d::pipeline::object::GeometricQueryType::Proximity(20.0 * link_radius);

        // let mut world = ncollide3d::pipeline::object::GeometricQueryType::CollisionWorld::new(0.0);
        // let mut link_handles: Vec<Vec<CollisionObjectSlabHandle>> = Vec::new();
        // let mut active_pairs: Vec<BTreeMap<CollisionObjectSlabHandle, Vec<CollisionObjectSlabHandle>>> = Vec::new();
        // let mut active_obstacles: Vec<Vec<(Option<CollisionObjectSlabHandle>, f64)>> = Vec::new();
        // for arm_idx in 0..frames.len() {
        //     let mut handles: Vec<CollisionObjectSlabHandle> = Vec::new();
        //     let mut obstacles: Vec<(Option<CollisionObjectSlabHandle>, f64)> = Vec::new();
        //     let pair: BTreeMap<CollisionObjectSlabHandle, Vec<CollisionObjectSlabHandle>> = BTreeMap::new();
        //     let last_elem = frames[arm_idx].0.len() - 1;
        //     for i in 0..last_elem {
        //         let start_pt = Point3::from(frames[arm_idx].0[i]);
        //         let end_pt = Point3::from(frames[arm_idx].0[i + 1]);
        //         let segment = ncollide3d::shape::ShapeHandle::new(Segment::new(start_pt, end_pt));
        //         let segment_pos = nalgebra::one();
        //         let link_data = CollisionObjectData::new(format!("Link {}", i), LinkData::new(true, arm_idx as i32));
        //         let handle = world.add(segment_pos, segment, link_groups, proximity_query, link_data);
        //         handles.push(handle.0);
        //         obstacles.push((None, 0.0));
        //     }
        //     link_handles.push(handles);
        //     active_pairs.push(pair);
        //     active_obstacles.push(obstacles);
        // }
        //
        // let mut dyn_obstacle_handles: Vec<(CollisionObjectSlabHandle, String)> = Vec::new();
        // for i in 0..plane_obstacles.len() {
        //     let plane_obs = &plane_obstacles[i];
        //     let half_extents = Vector3::new(plane_obs.x_halflength, plane_obs.y_halflength, plane_obs.z_halflength);
        //     let plane = ncollide3d::shape::ShapeHandle::new(Cuboid::new(half_extents));
        //     let plane_ts = Translation3::new(plane_obs.tx, plane_obs.ty, plane_obs.tz);
        //     let plane_rot = UnitQuaternion::from_euler_angles(plane_obs.rx, plane_obs.ry, plane_obs.rz);
        //     let plane_pos = Isometry3::from_parts(plane_ts, plane_rot);
        //     let plane_data = CollisionObjectData::new(plane_obs.name.clone(), LinkData::new(false, -1));
        //     let plane_handle = world.add(plane_pos, plane, others_groups, proximity_query, plane_data);
        //     if plane_obs.is_dynamic {
        //         dyn_obstacle_handles.push((plane_handle.0, plane_handle.1.data().name.clone()));
        //     }
        // }
        //
        // for i in 0..sphere_obstacles.len() {
        //     let sphere_obs = &sphere_obstacles[i];
        //     let sphere = ncollide3d::shape::ShapeHandle::new(Ball::new(sphere_obs.radius));
        //     let sphere_ts = Translation3::new(sphere_obs.tx, sphere_obs.ty, sphere_obs.tz);
        //     let sphere_rot = UnitQuaternion::identity();
        //     let sphere_pos = Isometry3::from_parts(sphere_ts, sphere_rot);
        //     let sphere_data = CollisionObjectData::new(sphere_obs.name.clone(), LinkData::new(false, -1));
        //     let sphere_handle = world.add(sphere_pos, sphere, others_groups, proximity_query, sphere_data);
        //     if sphere_obs.is_dynamic {
        //         dyn_obstacle_handles.push((sphere_handle.0, sphere_handle.1.data().name.clone()));
        //     }
        // }
        //
        // for i in 0..pcd_obstacles.len() {
        //     let pcd_obs = &pcd_obstacles[i];
        //     // let mut shapes: Vec<(Isometry3<f64>, ShapeHandle<f64>)> = Vec::new();
        //     // for sphere_obs in &pcd_obs.points {
        //     //     let sphere = ShapeHandle::new(Ball::new(sphere_obs.radius));
        //     //     let sphere_ts = Translation3::new(sphere_obs.tx, sphere_obs.ty, sphere_obs.tz);
        //     //     let sphere_rot = UnitQuaternion::identity();
        //     //     let sphere_pos = Isometry3::from_parts(sphere_ts, sphere_rot);
        //     //     shapes.push((sphere_pos, sphere));
        //     // }
        //     let mut points: Vec<Point3<f64>> = Vec::new();
        //     for sphere_obs in &pcd_obs.points {
        //         points.push(Point3::new(sphere_obs.tx, sphere_obs.ty, sphere_obs.tz));
        //     }
        //     // let pcd = ShapeHandle::new(Compound::new(shapes));
        //     let pcd = ncollide3d::shape::ShapeHandle::new(ncollide3d::shape::ConvexHull::try_from_points(&points).unwrap());
        //     let pcd_ts = Translation3::new(pcd_obs.tx, pcd_obs.ty, pcd_obs.tz);
        //     let pcd_rot = UnitQuaternion::from_euler_angles(pcd_obs.rx, pcd_obs.ry, pcd_obs.rz);
        //     let pcd_pos = Isometry3::from_parts(pcd_ts, pcd_rot);
        //     let pcd_data = CollisionObjectData::new(pcd_obs.name.clone(), LinkData::new(false, -1));
        //     let pcd_handle = world.add(pcd_pos, pcd, others_groups, proximity_query, pcd_data);
        //     if pcd_obs.is_dynamic {
        //         dyn_obstacle_handles.push((pcd_handle.0, pcd_handle.1.data().name.clone()));
        //     }
        // }
        return Self{obstacles, obstacle_positions}; //world, link_radius, link_handles, dyn_obstacle_handles, active_pairs, active_obstacles};
    }

    pub fn update_links(
        &mut self,
        frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>,
    ) {
        // for arm_idx in 0..frames.len() {
        //     let last_elem = frames[arm_idx].0.len() - 1;
        //     for i in 0..last_elem {
        //         let start_pt = Point3::from(frames[arm_idx].0[i]);
        //         let end_pt = Point3::from(frames[arm_idx].0[i + 1]);
        //         let segment = ncollide3d::shape::ShapeHandle::new(Segment::new(start_pt, end_pt));
        //         let link = self.world.objects.get_mut(self.link_handles[arm_idx][i]).unwrap();
        //         link.set_shape(segment);
        //     }
        // }
    }

    pub fn update_dynamic_obstacle(
        &mut self,
        name: &str,
        position: Isometry3<f64>,
    ) {
        // for (handle, id) in &self.dyn_obstacle_handles {
        //     if id == name {
        //         let co = self.world.objects.get_mut(*handle).unwrap();
        //         co.set_position(position);
        //         break;
        //     }
        // }
    }
}
