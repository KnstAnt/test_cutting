use nalgebra::*;
use obj::{Obj, ObjData};
use parry3d_f64::math::UnitVector;
use parry3d_f64::query::{IntersectResult, PointQuery};
use parry3d_f64::shape::{Cuboid, HalfSpace, Polyline, Shape, TriMesh, TriMeshFlags};
use parry3d_f64::transformation::intersect_meshes;
use std::collections::{HashMap, HashSet};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::RwLock;

mod sliced_mesh;
mod plane;
mod hydrostatics;
use sliced_mesh::*;
use plane::*;
use hydrostatics::*;

fn main() {
    let scale = 0.001;
    let mesh = load_stl(Path::new("assets/Sofiya.stl")).scaled(&Vector3::new(scale, scale, scale));
    
    let sliced_mesh = slice_parry_mesh(&mesh, plane: &Plane);
    let plane_x = Plane::from_point_and_normal(Point3::new(0., 0., 4.), Vector3::new(0., 0., 1.));
    let hydrostatics = calculate_hydrostatics(sliced_mesh, &plane_x);
}

#[inline(always)]
fn intersect_edge(a: &Point3<f64>, b: &Point3<f64>, d_a: f64, d_b: f64) -> Point3<f64> {
    // Доля пути от 'a' до 'b', где расстояние становится равным 0
    let t = d_a / (d_a - d_b);
    a + (b - a) * t
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
        .map(|v| Point3::new(v[0] as f64, v[1] as f64, v[2] as f64))
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
        .map(|v| Point3::new(v[0] as f64, v[1] as f64, v[2] as f64))
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
