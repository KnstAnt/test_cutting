mod hydrostatics;
mod plane;
mod sliced_mesh;

pub use sliced_mesh::*;
pub use plane::*;
pub use hydrostatics::*;

use obj::{Obj, ObjData};
use parry3d_f64::glamx::DQuat;
use parry3d_f64::math::*;
use parry3d_f64::shape::{TriMesh, TriMeshFlags};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

use crate::tools::{DisplacementCache, LocalCache, Position, load_stl};


pub fn test_sofia() {
    let scale = 0.001f64;
    let path = "assets/Sofiya.stl";
    let mut mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let center = Vec3::new(65.25, 0., 0.);
    let heel = 20.;
    let trim = 20.;
    let draught = 8.;
    let isometry = position(&center, heel, trim, draught);
    mesh.transform_vertices(&isometry);
    //   let (center, normal) = normal(center, heel, trim, dz);
    let plane = Plane::from_point_and_normal(center, Vec3::new(0., 0., 1.));
    let sliced_mesh = plane.slice_mesh(&mesh);
    let hydrostatics = sliced_mesh.hydrostatics(&plane);
    let center_of_buoyancy = hydrostatics.center_of_buoyancy;
    let center_of_buoyancy = isometry.inverse_transform_point(center_of_buoyancy);
    println!(
        "{} {} {} center:({:.3}, {:.3}, {:.3})  v: {} {}",
        heel as i32,
        trim as i32,
        draught,
        center.x,
        center.y,
        center.z,
        hydrostatics.volume,
        center_of_buoyancy
    );
}

pub fn calculate_hydrostatic_old(mut mesh: TriMesh, dx: f64, heel: f64, trim: f64, draught: f64) -> (f64, Vec3) {
    let center = Vec3::new(dx, 0., 0.);
    let isometry = position(&center, heel, trim, draught);
    mesh.transform_vertices(&isometry);
    let plane = Plane::from_point_and_normal(center, Vec3::new(0., 0., 1.));
    let sliced_mesh = plane.slice_mesh(&mesh);
    let hydrostatics = sliced_mesh.hydrostatics_old(&plane);
    let center_of_buoyancy = hydrostatics.center_of_buoyancy;
    let center_of_buoyancy = isometry.inverse_transform_point(center_of_buoyancy);
    (hydrostatics.volume, center_of_buoyancy)
}

pub fn calculate_hydrostatic(mesh: &TriMesh, dx: f64, heel: f64, trim: f64, draught: f64) -> (f64, Vec3) {
    let center = Vec3::new(dx, 0., 0.);
    let isometry = position(&center, heel, trim, draught).inverse();
    let local_point = isometry.transform_point(Vec3::ZERO); 
    let local_normal = isometry.transform_vector(Vec3::Z).normalize(); 
    let plane = Plane::from_point_and_normal(local_point, local_normal);
    let sliced_mesh = plane.slice_mesh(mesh);
    let hydrostatics = sliced_mesh.hydrostatics(&plane);
    let center_of_buoyancy = hydrostatics.center_of_buoyancy;
    (hydrostatics.volume, center_of_buoyancy)
}


/*
fn test_sofia() -> usize {
    let scale = 0.001f64;
    let path = "assets/Sofiya.stl";
    let mesh = load_stl(Path::new(path)).scaled(&Vec3::new(scale, scale, scale));
    let mut qnt = 0;
    //  let heel_steps = vec![-20., -5., -2., 0., 2., 5., 20.];
    //  let trim_steps = vec![-10., -2., -1., 0., 1., 2., 10.];
    let center = Vec3::new(65.25, 0., 0.);
    let heel_steps = vec![-20., 0., 20.];
    let trim_steps = vec![-20., 0., 20.];
    for dz in 1..=2 {
        let dz = (dz * 7 + 1) as f64;
        for &heel in &heel_steps {
            for &trim in &trim_steps {
                let (center, normal) = normal(center, heel, trim, dz);
                let plane = Plane::from_point_and_normal(center, normal);
                let sliced_mesh = plane.slice_mesh(&mesh);
                let hydrostatics = sliced_mesh.hydrostatics(&plane);
                println!(
                    "{} {} {} center:({:.3}, {:.3}, {:.3}) normal:({:.3}, {:.3}, {:.3}) v: {} {}",
                    heel as i32,
                    trim as i32,
                    dz,
                    center.x,
                    center.y,
                    center.z,
                    normal.x,
                    normal.y,
                    normal.z,
                    hydrostatics.volume,
                    hydrostatics.center_of_buoyancy
                );
                qnt += 1;
            }
        }
    }
    qnt
}*/

/*
pub fn position(center: &Vec3, heel: f64, trim: f64, draught: f64) -> Isometry3<f64> {
    let heel_rad = -heel.to_radians();
    let trim_rad = trim.to_radians();
    let trim_rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), trim_rad);
    let transformed_x_axis = trim_rotation.transform_vector(&Vector3::x_axis());
    let transformed_x_axis = UnitVector3::new_normalize(transformed_x_axis);
    let heel_rotation = UnitQuaternion::from_axis_angle(&transformed_x_axis, heel_rad);
    let rotation = heel_rotation * trim_rotation;
    let mut center = center.clone();
    center.z += draught;
    let point = rotation.transform_point(&center);
    let translation = Translation3::new(-point.x, -point.y, -point.z);
    Isometry::from_parts(translation, rotation)
}*/
pub fn position(center: &Vec3, heel: f64, trim: f64, draught: f64) -> Pose3 {
    let heel_rad = -heel.to_radians();
    let trim_rad = trim.to_radians();

    // 1. Вращение по дифференту (trim) вокруг оси Y
    let trim_rotation = DQuat::from_axis_angle(Vec3::Y, trim_rad);

    // 2. Находим трансформированную ось X для крена (heel)
    let transformed_x_axis = trim_rotation * Vec3::X;
    
    // 3. Вращение по крену вокруг новой оси X
    let heel_rotation = DQuat::from_axis_angle(transformed_x_axis.normalize(), heel_rad);
    
    // Итоговое вращение
    let rotation = heel_rotation * trim_rotation;

    // 4. Смещение центра
    let mut center_offset = *center;
    center_offset.z += draught;

    // 5. Вычисляем позицию (в glam вращение точки делается через оператор *)
    let point = rotation * center_offset;

    // 6. Создаем Isometry (в Parry с фичей glam это структура с полями translation и rotation)
    Pose3::from_parts(-point, rotation)
}

/// Расчет нормали по крену и дифференту
/*
pub fn normal(heel: f64, trim: f64) -> Vector3<f64> {
    let heel_rad = heel.to_radians();
    let trim_rad = -trim.to_radians();
    let trim_rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), trim_rad);
    let transformed_x_axis = trim_rotation.transform_vector(&Vector3::x_axis());
    let transformed_x_axis = UnitVector3::new_normalize(transformed_x_axis);
    let heel_rotation = UnitQuaternion::from_axis_angle(&transformed_x_axis, heel_rad);
    let rotation = heel_rotation * trim_rotation;
    rotation.transform_vector(&Vec3::new(0., 0., 1.))
}
*/
pub fn normal(heel: f64, trim: f64) -> Vec3 {
    let heel_rad = heel.to_radians();
    let trim_rad = -trim.to_radians();
    
    // 1. Вращение по дифференту вокруг оси Y
    let trim_rotation = DQuat::from_axis_angle(Vec3::Y, trim_rad);
    
    // 2. Получаем трансформированную ось X
    let transformed_x_axis = trim_rotation * Vec3::X;
    
    // 3. Вращение по крену вокруг новой оси X
    let heel_rotation = DQuat::from_axis_angle(transformed_x_axis.normalize(), heel_rad);
    
    // 4. Итоговое вращение
    let rotation = heel_rotation * trim_rotation;
    
    // 5. Трансформируем вектор нормали (Z-up)
    rotation * Vec3::Z
}
