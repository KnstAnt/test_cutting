use nalgebra::*;
use obj::{Obj, ObjData};
use parry3d_f64::glamx::dvec3;
use parry3d_f64::math::Vec3;
use parry3d_f64::query::{IntersectResult, PointQuery};
use parry3d_f64::shape::{Cuboid, HalfSpace, Polyline, Shape, TriMesh, TriMeshFlags};
use parry3d_f64::transformation::intersect_meshes;
use std::collections::{HashMap, HashSet};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::RwLock;
use std::time::Instant;
use crate::tools::Plane;
mod tools {
mod floating_position {
mod floating_position {
use nalgebra::{Matrix3, Point3, Rotation3, Vector3};
use parry3d_f64::shape::{Shape, TriMesh};
use crate::tools::Plane;
pub struct LoadingCondition {
    pub target_volume: f64,
    pub target_lcg: f64,
    pub target_tcg: f64,
}
#[derive(Debug, Clone, Copy)]
pub struct FloatingPosition {
    pub draft_z: f64,
    pub heel_rx: f64,
    pub trim_ry: f64,
}
impl FloatingPosition {
    pub fn to_plane(&self) -> Plane {
        let rotation = Rotation3::from_euler_angles(self.heel_rx, self.trim_ry, 0.0);
        let normal = rotation * Vector3::new(0.0, 0.0, 1.0);
        let point_on_plane = Point3::new(0.0, 0.0, self.draft_z);
        Plane::from_point_and_normal(point_on_plane, normal)
    }
}
pub struct SolverConfig {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub delta: f64,
}
pub fn find_equilibrium(
    mesh: &TriMesh,
    target: &LoadingCondition,
    initial_guess: FloatingPosition,
    config: &SolverConfig
) -> Result<FloatingPosition, &'static str> {
    let mut current_pos = initial_guess;
    let mut current_fx_norm = f64::MAX;
    for i in 0..config.max_iterations {
        let plane = current_pos.to_plane();
        let slice = plane.slice_mesh(mesh);
        let hydro = slice.hydrostatics(&plane);
        let area = slice.waterline_area().max(0.1);
        let f_x = Vector3::new(
            (hydro.volume - target.target_volume) / area,
            hydro.center_of_buoyancy.x - target.target_lcg,
            hydro.center_of_buoyancy.y - target.target_tcg,
        );
        let new_norm = f_x.norm();
        if new_norm < config.tolerance {
            println!("✅ Сходимость на итерации {}: fx={}", i, new_norm);
            return Ok(current_pos);
        }
        let jacobian = calculate_normalized_jacobian(mesh, current_pos, &f_x, area, config.delta, target);
        if let Some(step) = jacobian.lu().solve(&(-f_x)) {
            let mut alpha = 1.0;
            let mut best_pos = current_pos;
            let mut improved = false;
            for _ in 0..5 {
                let trial_pos = FloatingPosition {
                    draft_z: current_pos.draft_z + step[0] * alpha,
                    heel_rx: current_pos.heel_rx + step[1] * alpha,
                    trim_ry: current_pos.trim_ry + step[2] * alpha,
                };
                let t_plane = trial_pos.to_plane();
                let t_slice = t_plane.slice_mesh(mesh);
                let t_hydro = t_slice.hydrostatics(&t_plane);
                let t_area = t_slice.waterline_area().max(0.1);
                let t_fx = Vector3::new(
                    (t_hydro.volume - target.target_volume) / t_area,
                    t_hydro.center_of_buoyancy.x - target.target_lcg,
                    t_hydro.center_of_buoyancy.y - target.target_tcg,
                );
                if t_fx.norm() < current_fx_norm {
                    best_pos = trial_pos;
                    current_fx_norm = t_fx.norm();
                    improved = true;
                    break;
                }
                alpha *= 0.5;
            }
            if !improved {
                current_pos.draft_z += step[0] * 0.1;
            } else {
                current_pos = best_pos;
            }
        } else {
            return Err("Singular Jacobian");
        }
    }
    Err("Diverged after max iterations")
}
fn calculate_normalized_jacobian(
    mesh: &TriMesh,
    current_pos: FloatingPosition,
    f_x_normalized: &Vector3<f64>,
    base_area: f64,
    config_delta: f64,
    target: &LoadingCondition
) -> Matrix3<f64> {
    let mut jacobian = Matrix3::zeros();
    let delta = config_delta;
    let get_norm_f = |pos: FloatingPosition| {
        let p = pos.to_plane();
        let s = p.slice_mesh(mesh);
        let h = s.hydrostatics(&p);
        Vector3::new(
            (h.volume - target.target_volume) / base_area,
            h.center_of_buoyancy.x - target.target_lcg,
            h.center_of_buoyancy.y - target.target_tcg,
        )
    };
    let mut pos_z = current_pos; pos_z.draft_z += delta;
    jacobian.set_column(0, &((get_norm_f(pos_z) - f_x_normalized) / delta));
    let mut pos_rx = current_pos; pos_rx.heel_rx += delta;
    jacobian.set_column(1, &((get_norm_f(pos_rx) - f_x_normalized) / delta));
    let mut pos_ry = current_pos; pos_ry.trim_ry += delta;
    jacobian.set_column(2, &((get_norm_f(pos_ry) - f_x_normalized) / delta));
    jacobian
}
fn calculate_numerical_jacobian(mesh: &TriMesh, current_pos: FloatingPosition, f_x: &Vector3<f64>, config_delta: f64) -> Matrix3<f64> {
    let mut jacobian = Matrix3::zeros();
    let mut pos_z = current_pos;
    pos_z.draft_z += config_delta;
    let d_hydro_z = pos_z.to_plane().slice_mesh(mesh).hydrostatics(&pos_z.to_plane());
    jacobian.set_column(0, &((Vector3::new(d_hydro_z.volume, d_hydro_z.center_of_buoyancy.x, d_hydro_z.center_of_buoyancy.y) - f_x) / config_delta));
    let mut pos_rx = current_pos;
    pos_rx.heel_rx += config_delta;
    let d_hydro_rx = pos_rx.to_plane().slice_mesh(mesh).hydrostatics(&pos_rx.to_plane());
    jacobian.set_column(1, &((Vector3::new(d_hydro_rx.volume, d_hydro_rx.center_of_buoyancy.x, d_hydro_rx.center_of_buoyancy.y) - f_x) / config_delta));
    let mut pos_ry = current_pos;
    pos_ry.trim_ry += config_delta;
    let d_hydro_ry = pos_ry.to_plane().slice_mesh(mesh).hydrostatics(&pos_ry.to_plane());
    jacobian.set_column(2, &((Vector3::new(d_hydro_ry.volume, d_hydro_ry.center_of_buoyancy.x, d_hydro_ry.center_of_buoyancy.y) - f_x) / config_delta));
    jacobian
}
}
pub use floating_position::*;
}
mod hydrostatics {
use nalgebra::Point3;
pub struct Hydrostatics {
    pub volume: f64,
    pub center_of_buoyancy: Point3<f64>,
}
}
mod plane {
use nalgebra::{Point3, Vector3};
use parry3d_f64::math::{Vec3, Vector};
use parry3d_f64::shape::TriMesh;
use crate::tools::SlicedMesh;
pub struct Plane {
    pub normal: Vector3<f64>,
    pub d: f64,
}
impl Plane {
    pub fn from_point_and_normal(point: Point3<f64>, normal: Vector3<f64>) -> Self {
        let normal = normal.normalize();
        let d = normal.dot(&point.coords);
        Self { normal, d }
    }
    #[inline(always)]
    pub fn distance(&self, p: &Point3<f64>) -> f64 {
        self.normal.dot(&p.coords) - self.d
    }
    pub fn slice_mesh(&self, mesh: &TriMesh) -> SlicedMesh {
        let vertices: &[Vector] = mesh.vertices();
        let indices = mesh.indices();
        let distances: Vec<f64> = vertices.iter().map(|v| self.distance(&Point3::new(v.x, v.y, v.z))).collect();
        let mut submerged_triangles = Vec::with_capacity(indices.len() / 2);
        let mut waterline_edges = Vec::new();
        for face in indices {
            let v0 = &vertices[face[0] as usize];
            let v1 = &vertices[face[1] as usize];
            let v2 = &vertices[face[2] as usize];
            let d0 = distances[face[0] as usize];
            let d1 = distances[face[1] as usize];
            let d2 = distances[face[2] as usize];
            let mut pts = [
                (v0, d0),
                (v1, d1),
                (v2, d2),
            ];
            let above_count = pts.iter().filter(|&&(_, d)| d > 0.0).count();
            match above_count {
                0 => {
                    submerged_triangles.push([Point3::new(v0.x, v0.y, v0.z), Point3::new(v1.x, v1.y, v1.z), Point3::new(v2.x, v2.y, v2.z)]);
                }
                3 => {
                }
                1 => {
                    while pts[0].1 <= 0.0 {
                        pts.rotate_left(1);
                    }
                    let top = pts[0].0;
                    let bottom1 = pts[1].0;
                    let bottom2 = pts[2].0;
                    let i1 = intersect_edge(top, bottom1, pts[0].1, pts[1].1);
                    let i2 = intersect_edge(top, bottom2, pts[0].1, pts[2].1);
                    submerged_triangles.push([Point3::new(bottom1.x, bottom1.y, bottom1.z), Point3::new(bottom2.x, bottom2.y, bottom2.z), Point3::new(i1.x, i1.y, i1.z)]);
                    submerged_triangles.push([Point3::new(bottom2.x, bottom2.y, bottom2.z), Point3::new(i2.x, i2.y, i2.z), Point3::new(i1.x, i1.y, i1.z)]);
                    waterline_edges.push([i2, i1]);
                }
                2 => {
                    while pts[0].1 > 0.0 {
                        pts.rotate_left(1);
                    }
                    let bottom = pts[0].0;
                    let top1 = pts[1].0;
                    let top2 = pts[2].0;
                    let i1 = intersect_edge(bottom, top1, pts[0].1, pts[1].1);
                    let i2 = intersect_edge(bottom, top2, pts[0].1, pts[2].1);
                    submerged_triangles.push([Point3::new(bottom.x, bottom.y, bottom.z), Point3::new(i1.x, i1.y, i1.z), Point3::new(i2.x, i2.y, i2.z)]);
                    waterline_edges.push([i1, i2]);
                }
                _ => unreachable!(),
            }
        }
        SlicedMesh {
            submerged_triangles,
            waterline_edges: waterline_edges.into_iter().map(|vv| vv.map(|v| Point3::new(v.x, v.y, v.z))).collect(),
        }
    }
}
#[inline(always)]
fn intersect_edge(a: &Vector, b: &Vector, d_a: f64, d_b: f64) -> Vec3 {
    let denom = d_a - d_b;
    if denom.abs() < 1e-8 {
        return *a;
    }
    let t = d_a / denom;
    a + (b - a) * t
}
}
mod sliced_mesh {
use nalgebra::{Point3, Vector3};
use crate::tools::{Hydrostatics, Plane};
pub struct SlicedMesh {
    pub submerged_triangles: Vec<[Point3<f64>; 3]>,
    pub waterline_edges: Vec<[Point3<f64>; 2]>,
}
impl SlicedMesh {
    pub fn volume(&self, plane: &Plane) -> f64 {
        let mut total_volume = 0.0;
        let mut add_tetrahedron = |p1: &Point3<f64>, p2: &Point3<f64>, p3: &Point3<f64>| {
            total_volume += p1.coords.dot(&p2.coords.cross(&p3.coords)) / 6.0;
        };
        for [p1, p2, p3] in &self.submerged_triangles {
            add_tetrahedron(p1, p2, p3);
        }
        if let Some(first_edge) = self.waterline_edges.first() {
            let p_ref = first_edge[0];
            for [a, b] in &self.waterline_edges {
                let cross = (a - p_ref).cross(&(b - p_ref));
                let is_pointing_out = cross.dot(&plane.normal) > 0.0;
                if is_pointing_out {
                    add_tetrahedron(&p_ref, a, b);
                } else {
                    add_tetrahedron(&p_ref, b, a);
                }
            }
        }
        total_volume.abs()
    }
    pub fn hydrostatics(&self, plane: &Plane) -> Hydrostatics {
        let mut total_volume = 0.0;
        let mut sum_centroid = Vector3::zeros();
        let mut add_tetrahedron = |p1: &Point3<f64>, p2: &Point3<f64>, p3: &Point3<f64>| {
            let v_i = p1.coords.dot(&p2.coords.cross(&p3.coords)) / 6.0;
            total_volume += v_i;
            let centroid_i = (p1.coords + p2.coords + p3.coords) / 4.0;
            sum_centroid += centroid_i * v_i;
        };
        for tri in &self.submerged_triangles {
            add_tetrahedron(&tri[0], &tri[1], &tri[2]);
        }
        if let Some(first_edge) = self.waterline_edges.first() {
            let p_ref = first_edge[0];
            for edge in &self.waterline_edges {
                let a = edge[0];
                let b = edge[1];
                let cross = (a - p_ref).cross(&(b - p_ref));
                let is_pointing_out = cross.dot(&plane.normal) > 0.0;
                if is_pointing_out {
                    add_tetrahedron(&p_ref, &a, &b);
                } else {
                    add_tetrahedron(&p_ref, &b, &a);
                }
            }
        }
        let center_of_buoyancy = if total_volume.abs() > 1e-6 {
            Point3::from(sum_centroid / total_volume)
        } else {
            Point3::origin()
        };
        Hydrostatics {
            volume: total_volume.abs(),
            center_of_buoyancy,
        }
    }
    pub fn waterline_area(&self) -> f64 {
        self.waterline_edges.iter()
            .map(|[a, b]| (a.x * b.y - b.x * a.y))
            .sum::<f64>().abs() * 0.5
    }
}
}
pub use floating_position::*;
pub use sliced_mesh::*;
pub use plane::*;
pub use hydrostatics::*;
}
#[cfg(test)]
mod tests {
mod basic_test {
use parry3d_f64::glamx::dvec3;
use parry3d_f64::shape::Cuboid;
use parry3d_f64::shape::TriMesh;
use nalgebra::{Point3, Vector3};
use crate::tools::Plane;
fn create_test_cube() -> TriMesh {
    let cuboid = Cuboid::new(dvec3(1.0, 1.0, 1.0));
    let (vertices, indices) = cuboid.to_trimesh();
    TriMesh::new(vertices, indices).unwrap()
}
fn assert_approx_eq(a: f64, b: f64, epsilon: f64) {
    assert!((a - b).abs() < epsilon, "Ожидалось {}, получено {} (разница {})", a, b, (a - b).abs());
}
#[test]
fn test_fully_emerged_mesh() {
    let mesh = create_test_cube();
    let plane = Plane::from_point_and_normal(Point3::new(0.0, 0.0, -2.0), Vector3::new(0.0, 0.0, 1.0));
    let sliced = plane.slice_mesh(&mesh);
    let hydro = sliced.hydrostatics(&plane);
    assert_eq!(sliced.submerged_triangles.len(), 0, "Должно быть 0 подводных треугольников");
    assert_eq!(hydro.volume, 0.0, "Объем должен быть равен 0");
}
#[test]
fn test_fully_submerged_mesh() {
    let mesh = create_test_cube();
    let plane = Plane::from_point_and_normal(Point3::new(0.0, 0.0, 2.0), Vector3::new(0.0, 0.0, 1.0));
    let sliced = plane.slice_mesh(&mesh);
    let hydro = sliced.hydrostatics(&plane);
    assert!(sliced.submerged_triangles.len() > 0, "Весь меш должен быть погружен");
    assert_eq!(sliced.waterline_edges.len(), 0, "Сечения быть не должно");
    assert_approx_eq(hydro.volume, 8.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.x, 0.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.y, 0.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.z, 0.0, 1e-4);
}
#[test]
fn test_half_submerged_cube() {
    let mesh = create_test_cube();
    let plane = Plane::from_point_and_normal(Point3::origin(), Vector3::new(0.0, 0.0, 1.0));
    let sliced = plane.slice_mesh(&mesh);
    let hydro = sliced.hydrostatics(&plane);
    assert_approx_eq(hydro.volume, 4.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.x, 0.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.y, 0.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.z, -0.5, 1e-4);
}
#[test]
fn test_diagonal_slice() {
    let mesh = create_test_cube();
    let normal = Vector3::new(1.0, 0.0, 1.0).normalize();
    let plane = Plane::from_point_and_normal(Point3::origin(), normal);
    let sliced = plane.slice_mesh(&mesh);
    let hydro = sliced.hydrostatics(&plane);
    assert_approx_eq(hydro.volume, 4.0, 1e-4);
    assert!(hydro.center_of_buoyancy.x < 0.0);
    assert!(hydro.center_of_buoyancy.z < 0.0);
    assert_approx_eq(hydro.center_of_buoyancy.y, 0.0, 1e-4);
}
}
mod floating_position_test {
use std::{path::Path, time::Instant};
use parry3d_f64::{glamx::dvec3, shape::Shape};
use crate::{load_stl, tools::{FloatingPosition, LoadingCondition, SolverConfig, find_equilibrium}};
#[test]
fn basic() {
    let scale = 0.001f64;
    for path in ["assets/ark.stl", "assets/Sofiya.stl"] {
        let mesh = load_stl(Path::new(path)).scaled(dvec3(scale, scale, scale));
        let aabb = mesh.compute_aabb(&parry3d_f64::glamx::DPose3::identity());
        let total_height = aabb.maxs.z - aabb.mins.z;
        let conf = SolverConfig {
            max_iterations: 20,
            tolerance: 1e-4,
            delta: 1e-5,
        };
        let loading_condition = LoadingCondition {
            target_volume: 50.0,
            target_lcg: (aabb.maxs.x + aabb.mins.x) / 2.0,
            target_tcg: 0.0,
        };
        let initial_guess = FloatingPosition {
            draft_z: aabb.mins.z + (total_height * 0.4),
            heel_rx: 0.0,
            trim_ry: 0.0,
        };
        let t = Instant::now();
        let result = find_equilibrium(
            &mesh,
            &loading_condition,
            initial_guess,
            &conf,
        );
        let elapsed = t.elapsed();
        println!("\nTest model: {}", path);
        match result {
            Ok(pos) => println!("Converged in {:?}\nPosition: Draft={:.4}, Heel={:.4}°, Trim={:.4}°",
                                elapsed, pos.draft_z, pos.heel_rx.to_degrees(), pos.trim_ry.to_degrees()),
            Err(e) => println!("Failed: {} (Elapsed: {:?})", e, elapsed),
        }
    }
}
}
mod simple_solids {
use std::time::Instant;
use nalgebra::{ComplexField, Point3, Vector3};
use parry3d_f64::{glamx::{Vec3, dvec3}, shape::{Cuboid, TriMesh}};
use crate::tools::{Plane, SlicedMesh};
#[test]
fn test_cuboid_volume() {
    let cuboid = Cuboid::new(dvec3(1.0, 1.0, 1.0));
    let (v, i) = cuboid.to_trimesh();
    let mesh = TriMesh::new(v, i).unwrap();
    let plane = Plane::from_point_and_normal(Point3::origin(), Vector3::z());
    let mut result = plane.slice_mesh(&mesh);
    let p1 = Point3::new(-1.0, -1.0, 0.0);
    let p2 = Point3::new(1.0, -1.0, 0.0);
    let p3 = Point3::new(1.0, 1.0, 0.0);
    let p4 = Point3::new(-1.0, 1.0, 0.0);
    result.submerged_triangles.push([p1, p3, p2]);
    result.submerged_triangles.push([p1, p4, p3]);
    let volume = result.volume(&plane);
    let expected_volume = 4.0;
    println!("Куб 2x2x2 (объем = 8.0) | Calculated Volume: {}", volume);
    assert!((volume - expected_volume).abs() < 1e-5);
}
#[test]
fn test_volume() {
    for scene in test_scenes() {
        let max_z = scene.mesh.vertices().iter().map(|v| v.z).fold(f64::NEG_INFINITY, f64::max);
        let high_plane = Plane::from_point_and_normal(
            Point3::new(0.0, 0.0, max_z + 1.0),
            Vector3::z()
        );
        let sliced_full = high_plane.slice_mesh(&scene.mesh);
        let vol_full = sliced_full.volume(&high_plane);
        println!("Full Slice: {:<25} | Calc: {:.4} | Target: {:.4}",
                 scene.description, vol_full, scene.full_volume);
        assert!((vol_full - scene.full_volume).abs() < 1e-3,
                "Full volume mismatch for {}", scene.description);
        if scene.description == "Simple Cube" || scene.description == "3D Cross" {
            for (plane, factor) in test_planes(&scene.mesh) {
                let t = Instant::now();
                let sliced_half = plane.slice_mesh(&scene.mesh);
                let slice_elapsed = t.elapsed();
                let t = Instant::now();
                let result = sliced_half.volume(&plane);
                let volume_elapsed = t.elapsed();
                let target = scene.full_volume  * factor;
                println!("Half Slice: {:<25} | Calc: {:.4} | Target: {:.4} | Slice Elapsed: {:?} | Volume Elapsed: {:?}",
                         scene.description, result, target, slice_elapsed, volume_elapsed);
                assert!((result - target).abs() < 1e-3, "Half Slice of {} \n plane n {:?} d {:?},  \n result: {} \n target: {}", scene.description, plane.normal, plane.d, result, target);
            }
        }
    }
}
#[test]
fn test_hydrostatics() {
    for scene in test_scenes() {
        {
            let max_z = scene.mesh.vertices().iter().map(|v| v.z).fold(f64::NEG_INFINITY, f64::max);
            let plane = Plane::from_point_and_normal(
                Point3::new(0.0, 0.0, max_z + 1.0),
                Vector3::z()
            );
            let sliced_full = plane.slice_mesh(&scene.mesh);
            let result = sliced_full.hydrostatics(&plane).volume;
            let target = scene.full_volume;
            println!("Full Slice: {:<25} | Calc: {:.4} | Target: {:.4}",
                     scene.description, result, target);
            assert!((result - target).abs() < 1e-3, "Full volume mismatch for {} \n plane n {:?} d {:?},  \n result: {} \n target: {}", scene.description, plane.normal, plane.d, result, target);
        }
        if scene.description == "Simple Cube" || scene.description == "3D Cross" {
            for (plane, factor) in test_planes(&scene.mesh) {
                let t = Instant::now();
                let sliced_half = plane.slice_mesh(&scene.mesh);
                let slice_elapsed = t.elapsed();
                let t = Instant::now();
                let result = sliced_half.hydrostatics(&plane).volume;
                let volume_elapsed = t.elapsed();
                let target = scene.full_volume  * factor;
                println!("Half Slice: {:<25} | Calc: {:.4} | Target: {:.4} | Slice Elapsed: {:?} | Volume Elapsed: {:?}",
                         scene.description, result, target, slice_elapsed, volume_elapsed);
                assert!((result - target).abs() < 1e-3, "Half Slice of {} \n plane n {:?} d {:?},  \n result: {} \n target: {}", scene.description, plane.normal, plane.d, result, target);
            }
        }
    }
}
fn cap_and_get_volume(sliced: &SlicedMesh, plane: &Plane) -> f64 {
    if sliced.submerged_triangles.is_empty() { return 0.0; }
    let anchor = plane.normal * plane.d;
    let anchor_p = Point3::from_slice(&[anchor.x, anchor.y, anchor.z]);
    let mut total_volume = 0.0;
    for [p1, p2, p3] in &sliced.submerged_triangles {
        let v1 = p1 - anchor_p;
        let v2 = p2 - anchor_p;
        let v3 = p3 - anchor_p;
        total_volume += v1.dot(&v2.cross(&v3)) / 6.0;
    }
    total_volume.abs()
}
fn test_planes(mesh: &TriMesh) -> Vec<(Plane, f64)> {
    let aabb = mesh.aabb(&parry3d_f64::glamx::DPose3::IDENTITY);
    let center = aabb.mins.lerp(aabb.maxs, 0.5);
    return vec![
        (Plane::from_point_and_normal(Point3::new(center.x, center.y, aabb.mins.z + 1e-4), Vector3::z()), 0.0),
        (Plane::from_point_and_normal(Point3::new(center.x, center.y, aabb.maxs.z - 1e-4), Vector3::z()), 1.0),
        (Plane::from_point_and_normal(Point3::new(center.x, center.y, aabb.maxs.z + 1.0), Vector3::z()), 1.0),
        (Plane::from_point_and_normal(Point3::from_slice(&[center.x, center.y, center.z]), Vector3::new(1.0, 1.0, 1.0).normalize()), 0.5),
        (Plane::from_point_and_normal(Point3::from_slice(&[center.x, center.y, center.z]), Vector3::new(0.001, 0.001, 1.0).normalize()), 0.5),
        (Plane::from_point_and_normal(Point3::from_slice(&[center.x, center.y, center.z]), Vector3::new(0.3, 0.5, 0.8).normalize()), 0.5),
        (Plane::from_point_and_normal(Point3::new(center.x, center.y, center.z), Vector3::new(0.3, 0.5, 0.8).normalize()), 0.5),
    ]
}
struct TestScene {
    pub mesh: TriMesh,
    pub full_volume: f64,
    pub description: &'static str,
}
fn test_scenes() -> Vec<TestScene> {
    vec![
        create_box(2.0, 2.0, 2.0, dvec3(0.0, 0.0, 0.0), "Simple Cube"),
        create_box(1.0, 1.0, 1.0, dvec3(10.0, 10.0, 10.0), "Simple Offset Cube"),
        create_box(10.0, 0.01, 10.0, parry3d_f64::glamx::DVec3::ZERO, "Thin Plate"),
        create_l_shape("L-Shape Concave"),
        create_well("Deep Well"),
        create_two_islands("Two Disconnected Cubes"),
        create_pyramid(2.0, "Square Pyramid"),
        create_needle(100.0, 0.001, "Needle Shape"),
        create_cross("3D Cross"),
        create_stairs(5, "Stairs (Multiple Steps)"),
        create_tetrahedron(1.0, "Tetrahedron"),
    ]
}
fn create_box(hx: f64, hy: f64, hz: f64, offset: parry3d_f64::glamx::DVec3, desc: &'static str) -> TestScene {
    let (v, i) = Cuboid::new(dvec3(hx/2.0, hy/2.0, hz/2.0)).to_trimesh();
    let v_offset = v.into_iter().map(|p| p + offset).collect();
    TestScene {
        mesh: TriMesh::new(v_offset, i).unwrap(),
        full_volume: hx * hy * hz,
        description: desc,
    }
}
fn create_l_shape(desc: &'static str) -> TestScene {
    let (mut v1, mut i1) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();
    let (v2, i2) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();
    let vertex_offset = v1.len() as u32;
    let v2_off: Vec<parry3d_f64::glamx::DVec3> = v2.into_iter()
        .map(|p| p + dvec3(1.0, 0.0, 0.0))
        .collect();
    let i2_off: Vec<[u32; 3]> = i2.into_iter()
        .map(|f| [f[0] + vertex_offset, f[1] + vertex_offset, f[2] + vertex_offset])
        .collect();
    v1.extend(v2_off);
    i1.extend(i2_off);
    TestScene {
        mesh: TriMesh::new(v1, i1).expect("Failed to create TriMesh"),
        full_volume: 2.0,
        description: desc,
    }
}
fn create_well(desc: &'static str) -> TestScene {
    let mut all_v = Vec::new();
    let mut all_i = Vec::new();
    let offsets = vec![
        dvec3(0.0, 0.0, 0.0),
        dvec3(1.0, 0.0, 1.0),
        dvec3(-1.0, 0.0, 1.0),
        dvec3(0.0, 1.0, 1.0),
        dvec3(0.0, -1.0, 1.0),
    ];
    for offset in offsets {
        let (v, i) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();
        let v_len = all_v.len() as u32;
        let v_off: Vec<parry3d_f64::glamx::DVec3> = v.into_iter().map(|p| p + offset).collect();
        let i_off: Vec<[u32; 3]> = i.into_iter().map(|f| [f[0] + v_len, f[1] + v_len, f[2] + v_len]).collect();
        all_v.extend(v_off);
        all_i.extend(i_off);
    }
    TestScene {
        mesh: TriMesh::new(all_v, all_i).expect("Well mesh failed"),
        full_volume: 5.0,
        description: desc,
    }
}
fn create_two_islands(desc: &'static str) -> TestScene {
    let (mut v1, mut i1) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();
    let (v2, i2) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();
    let v2_off: Vec<parry3d_f64::glamx::DVec3> = v2.into_iter().map(|p| p + dvec3(10.0, 0.0, 0.0)).collect();
    let i2_off: Vec<[u32; 3]> = i2.into_iter().map(|f| [f[0]+8, f[1]+8, f[2]+8]).collect();
    v1.extend(v2_off);
    i1.extend(i2_off);
    TestScene {
        mesh: TriMesh::new(v1, i1).unwrap(),
        full_volume: 2.0,
        description: desc,
    }
}
fn create_pyramid(size: f64, desc: &'static str) -> TestScene {
    let h = size;
    let s = size / 2.0;
    let vertices = vec![
        dvec3(-s, -s, 0.0), dvec3(s, -s, 0.0),
        dvec3(s, s, 0.0), dvec3(-s, s, 0.0),
        dvec3(0.0, 0.0, h)
    ];
    let indices = vec![
        [0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4],
        [0, 2, 1], [0, 3, 2]
    ];
    TestScene {
        mesh: TriMesh::new(vertices, indices).expect("Pyramid failed"),
        full_volume: (size * size * h) / 3.0,
        description: desc,
    }
}
fn create_needle(length: f64, thickness: f64, desc: &'static str) -> TestScene {
    let h_len = length / 2.0;
    let h_thick = thickness / 2.0;
    let (v, i) = Cuboid::new(dvec3(h_len, h_thick, h_thick)).to_trimesh();
    let angle = 0.1;
    let rotated_v: Vec<parry3d_f64::glamx::DVec3> = v.into_iter().map(|p| {
        let x = p.x * angle.cos() - p.y * angle.sin();
        let y = p.x * angle.sin() + p.y * angle.cos();
        dvec3(x, y, p.z)
    }).collect();
    TestScene {
        mesh: TriMesh::new(rotated_v, i).expect("Needle mesh failed"),
        full_volume: length * thickness * thickness,
        description: desc,
    }
}
fn create_cross(desc: &'static str) -> TestScene {
    let mut all_v = Vec::new();
    let mut all_i = Vec::new();
    let dims = [
        dvec3(1.5, 0.5, 0.5),
        dvec3(0.5, 1.5, 0.5),
        dvec3(0.5, 0.5, 1.5),
    ];
    for half_extents in dims {
        let (v, i) = Cuboid::new(half_extents).to_trimesh();
        let v_len = all_v.len() as u32;
        all_v.extend(v);
        all_i.extend(i.into_iter().map(|f| [f[0] + v_len, f[1] + v_len, f[2] + v_len]));
    }
    TestScene {
        mesh: TriMesh::new(all_v, all_i).expect("Cross mesh failed"),
        full_volume: 9.0,
        description: desc,
    }
}
fn create_stairs(steps: usize, desc: &'static str) -> TestScene {
    let mut all_v = Vec::new();
    let mut all_i = Vec::new();
    let step_width = 1.0;
    let step_height = 0.2;
    let step_depth = 0.2;
    for i in 0..steps {
        let half_extents = dvec3(step_width / 2.0, step_depth / 2.0, step_height / 2.0);
        let (v, idx) = Cuboid::new(half_extents).to_trimesh();
        let offset = dvec3(
            0.0,
            i as f64 * step_depth,
            i as f64 * step_height
        );
        let v_len = all_v.len() as u32;
        let v_off: Vec<parry3d_f64::glamx::DVec3> = v.into_iter().map(|p| p + offset).collect();
        let i_off: Vec<[u32; 3]> = idx.into_iter().map(|f| [f[0] + v_len, f[1] + v_len, f[2] + v_len]).collect();
        all_v.extend(v_off);
        all_i.extend(i_off);
    }
    let single_step_vol = step_width * step_depth * step_height;
    TestScene {
        mesh: TriMesh::new(all_v, all_i).expect("Stairs mesh failed"),
        full_volume: single_step_vol * steps as f64,
        description: desc,
    }
}
fn create_tetrahedron(a: f64, desc: &'static str) -> TestScene {
    let h = a * (2.0 / 3.0).sqrt();
    let r = a / (3.0).sqrt();
    let vertices = vec![
        dvec3(r, 0.0, 0.0),
        dvec3(-r / 2.0, a / 2.0, 0.0),
        dvec3(-r / 2.0, -a / 2.0, 0.0),
        dvec3(0.0, 0.0, h),
    ];
    let indices = vec![
        [0, 1, 2],
        [0, 2, 3],
        [2, 1, 3],
        [1, 0, 3],
    ];
    let volume = a.powi(3) / (6.0 * 2.0f64.sqrt());
    TestScene {
        mesh: TriMesh::new(vertices, indices).expect("Tetrahedron failed"),
        full_volume: volume,
        description: desc,
    }
}
}
}
fn main() {
    let scale = 0.001f64;
    for path in ["assets/ark.stl", "assets/Sofiya.stl", "assets/ark.stl", "assets/Sofiya.stl"] {
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
        println!("Elapsed slice {:?}, hydrostatics {:?}", elapsed[0], elapsed[1]);
    }
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