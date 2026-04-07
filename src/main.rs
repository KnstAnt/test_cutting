use nalgebra::*;
use obj::{Obj, ObjData};
use parry3d_f64::glamx::dvec3;
use parry3d_f64::math::Vec3;
use parry3d_f64::shape::{TriMesh, TriMeshFlags};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

use crate::tools::Plane;

#[cfg(test)]
mod tests;
mod tools;

fn main() {
/*    let scale = 0.001f64;
    for path in [
        "assets/ark.stl",
        "assets/Sofiya.stl",
        "assets/ark.stl",
        "assets/Sofiya.stl",
    ] {
        let mesh = load_stl(Path::new(path)).scaled(dvec3(scale, scale, scale));

        let plane = Plane::from_point_and_normal(Point3::new(0., 0., 4.), Vector3::new(0., 0., 1.));
        let t = Instant::now();
        let sliced_mesh = plane.slice_mesh(&mesh);
        let mut elapsed = vec![t.elapsed()];
        let t = Instant::now();
        let hydrostatics = sliced_mesh.hydrostatics(&plane);
        elapsed.push(t.elapsed());
        println!("\nTest model: {}", path);
        println!("Volume: {}", hydrostatics.volume);
        println!("Center of buoyancy: {}", hydrostatics.center_of_buoyancy);
        println!(
            "Elapsed slice {:?}, hydrostatics {:?}",
            elapsed[0], elapsed[1]
        );
    }*/
    let t = Instant::now();
    let qnt = test_sofia() as f64;
    println!("time {:?}, {}", t.elapsed(), t.elapsed().as_secs_f64() / qnt);
}

fn test_sofia() -> usize {
    let scale = 0.001f64;
    let path = "assets/Sofiya.stl";
    let mesh = load_stl(Path::new(path)).scaled(dvec3(scale, scale, scale));
    let mut qnt = 0;
    let heel_steps = vec![-20., -5., -2., 0., 2., 5., 20.];
    let trim_steps = vec![-10., -2., -1., 0., 1., 2., 10.];
    for dz in 1..=14 {
        for &heel in &heel_steps {
            for &trim in &trim_steps {
                let normal = normal(heel, trim);
                let plane = Plane::from_point_and_normal(Point3::new(65.25, 0., dz as f64), normal);
                let sliced_mesh = plane.slice_mesh(&mesh);
                let hydrostatics = sliced_mesh.hydrostatics(&plane);
                println!("{} {} {} normal:({:.3}, {:.3}, {:.3}) v: {} {}", 
                heel as i32, trim as i32, dz, 
                normal.x, normal.y, normal.z,
                hydrostatics.volume, hydrostatics.center_of_buoyancy);
                qnt += 1;
            }
        }
    }
    qnt
}

fn load_obj(path: &Path) -> TriMesh {
    let Obj {
        data: ObjData {
            position, objects, ..
        },
        ..
    } = Obj::load(path).unwrap();
    let vertices = position
        .iter()
        .map(|v| Vec3::new(v[0] as f64, v[1] as f64, v[2] as f64))
        .collect::<Vec<_>>();
    let indices = objects[0].groups[0]
        .polys
        .iter()
        .map(|p| [p.0[0].0 as u32, p.0[1].0 as u32, p.0[2].0 as u32])
        .collect::<Vec<_>>();
    TriMesh::with_flags(vertices, indices, TriMeshFlags::all()).unwrap()
}

fn load_stl(path: &Path) -> TriMesh {
    let file = std::fs::File::open(path).unwrap();
    let mut reader = std::io::BufReader::new(file);
    let stl = stl_io::read_stl(&mut reader).unwrap();
    let vertices = stl
        .vertices
        .into_iter()
        .map(|v| Vec3::new(v[0] as f64, v[1] as f64, v[2] as f64))
        .collect::<Vec<_>>();
    let indices = stl
        .faces
        .into_iter()
        .map(|f| {
            [
                f.vertices[0] as u32,
                f.vertices[1] as u32,
                f.vertices[2] as u32,
            ]
        })
        .collect::<Vec<_>>();
    TriMesh::with_flags(vertices, indices, TriMeshFlags::all()).unwrap()
}

pub fn write_stl(path: &PathBuf, mesh: &TriMesh) {
    let (result, empty_normals): (Vec<_>, Vec<_>) = mesh
        .triangles()
        .map(|t| (t.normal(), t))
        .partition(|(n, _)| n.is_some());
    if !empty_normals.is_empty() {
        panic!("{}", format!("calculate normal error, path:{:?}", path));
    }
    let triangles: Vec<_> = result
        .into_iter()
        .map(|(n, t)| {
            let n = n.unwrap();
            let normal = stl_io::Vector([n[0] as f32, n[1] as f32, n[2] as f32]);
            let vertices = [
                stl_io::Vector([t.a[0] as f32, t.a[1] as f32, t.a[2] as f32]),
                stl_io::Vector([t.b[0] as f32, t.b[1] as f32, t.b[2] as f32]),
                stl_io::Vector([t.c[0] as f32, t.c[1] as f32, t.c[2] as f32]),
            ];
            stl_io::Triangle { normal, vertices }
        })
        .collect();
    let mut binary_stl = Vec::<u8>::new();
    stl_io::write_stl(&mut binary_stl, triangles.iter()).unwrap();
    let mut buffer = std::fs::File::create(&path).unwrap();
    buffer.write_all(&binary_stl).unwrap();
}

pub fn position(center: &Point3<f64>, heel: f64, trim: f64, draught: f64) -> Isometry3<f64> {
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
}

/*
/// Расчет нормали по крену и дифференту
pub fn normal(heel: f64, trim: f64) -> Vector3<f64> {
    let heel_rad = heel.to_radians();
    let trim_rad = -trim.to_radians();
    let trim_rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), trim_rad);
    let transformed_x_axis = trim_rotation.transform_vector(&Vector3::x_axis());
    let transformed_x_axis = UnitVector3::new_normalize(transformed_x_axis);
    let heel_rotation = UnitQuaternion::from_axis_angle(&transformed_x_axis, heel_rad);
    let rotation = heel_rotation * trim_rotation;
    rotation.transform_vector(&Vector3::new(0., 0., 1.))
}
*/

/// Расчет нормали по крену и дифференту
pub fn normal(heel: f64, trim: f64) -> Vector3<f64> {
    let heel_rad = heel.to_radians();
    let trim_rad = -trim.to_radians();
    let rotation = UnitQuaternion::from_euler_angles(heel_rad, trim_rad, 0.);
    rotation.transform_vector(&Vector3::new(0., 0., 1.))
}
