use parry3d_f64::glamx::dvec3;
use parry3d_f64::shape::Cuboid;
use parry3d_f64::shape::TriMesh;
// use nalgebra::{Point3, Vector3};

use crate::tools::Plane;

// Вспомогательная функция для генерации тестового меша (Куб 2x2x2)
// Z меняется от -1 до +1
fn create_test_cube() -> TriMesh {
    let cuboid = Cuboid::new(dvec3(1.0, 1.0, 1.0));
    let (vertices, indices) = cuboid.to_trimesh();
    TriMesh::new(vertices, indices).unwrap()
}

// Вспомогательная функция для сравнения f32 с погрешностью
fn assert_approx_eq(a: f64, b: f64, epsilon: f64) {
    assert!((a - b).abs() < epsilon, "Ожидалось {}, получено {} (разница {})", a, b, (a - b).abs());
}

#[test]
fn test_fully_emerged_mesh() {
    let mesh = create_test_cube();
    // Плоскость ниже куба (Z = -2). Куб полностью в воздухе (находится "над" плоскостью)
    // Нормаль смотрит вверх, d = -2. Расстояние до точек куба (Z от -1 до 1) будет положительным.
    let plane = Plane::from_point_and_normal(dvec3(0.0, 0.0, -2.0), dvec3(0.0, 0.0, 1.0));
    
    let sliced = plane.slice_mesh(&mesh);
    let hydro = sliced.hydrostatics(&plane);

    assert_eq!(sliced.submerged_triangles.len(), 0, "Должно быть 0 подводных треугольников");
    assert_eq!(hydro.volume, 0.0, "Объем должен быть равен 0");
}

#[test]
fn test_fully_submerged_mesh() {
    let mesh = create_test_cube();
    // Плоскость выше куба (Z = 2). Куб полностью под водой.
    let plane = Plane::from_point_and_normal(dvec3(0.0, 0.0, 2.0), dvec3(0.0, 0.0, 1.0));
    
    let sliced = plane.slice_mesh(&mesh);
    let hydro = sliced.hydrostatics(&plane);

    assert!(sliced.submerged_triangles.len() > 0, "Весь меш должен быть погружен");
    assert_eq!(sliced.waterline_edges.len(), 0, "Сечения быть не должно");
    assert_approx_eq(hydro.volume, 8.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.x, 0.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.y, 0.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.z, 0.0, 1e-4); // Центр куба
}

#[test]
fn test_half_submerged_cube() {
    let mesh = create_test_cube();
    // Плоскость ровно по центру куба (Z = 0)
    let plane = Plane::from_point_and_normal(parry3d_f64::glamx::DVec3::ZERO, dvec3(0.0, 0.0, 1.0));
    
    let sliced = plane.slice_mesh(&mesh);
    let hydro = sliced.hydrostatics(&plane);

    // Ожидаемый объем: половина куба = 4.0
    assert_approx_eq(hydro.volume, 4.0, 1e-4);
    
    // Ожидаемый центр величины (LCB, TCB, VCB): 
    // По X и Y = 0. По Z = середина подводной части (от -1 до 0), то есть -0.5
    assert_approx_eq(hydro.center_of_buoyancy.x, 0.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.y, 0.0, 1e-4);
    assert_approx_eq(hydro.center_of_buoyancy.z, -0.5, 1e-4);
}

#[test]
fn test_diagonal_slice() {
    let mesh = create_test_cube();
    // Режем куб по диагонали (Нормаль под 45 градусов в плоскости XZ), проходящей через 0,0,0
    // Это отрежет ровно половину объема по сложной геометрии
    let normal = dvec3(1.0, 0.0, 1.0).normalize();
    let plane = Plane::from_point_and_normal(parry3d_f64::glamx::DVec3::ZERO, normal);
    
    let sliced = plane.slice_mesh(&mesh);
    let hydro = sliced.hydrostatics(&plane);

    // Объем всё равно должен быть ровно половиной = 4.0
    assert_approx_eq(hydro.volume, 4.0, 1e-4);
    
    // Центр масс сместится в отрицательные координаты по X и Z (под воду)
    assert!(hydro.center_of_buoyancy.x < 0.0);
    assert!(hydro.center_of_buoyancy.z < 0.0);
    assert_approx_eq(hydro.center_of_buoyancy.y, 0.0, 1e-4); // Симметрия по Y сохраняется
}