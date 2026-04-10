use obj::{Obj, ObjData};
use parry3d_f64::glamx::DQuat;
use parry3d_f64::math::*;
use parry3d_f64::shape::{TriMesh, TriMeshFlags};
use std::io::Write;
use std::path::{Path, PathBuf};


pub fn load_obj(path: &Path) -> TriMesh {
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

pub fn load_stl(path: &Path) -> TriMesh {
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