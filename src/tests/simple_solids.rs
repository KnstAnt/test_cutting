use std::time::Instant;

use nalgebra::{ComplexField, Point3, Vector3};
use parry3d_f64::{glamx::{Vec3, dvec3}, shape::{Cuboid, TriMesh}};

use crate::tools::{Plane, SlicedMesh};


#[test]
fn test_cuboid_volume() {
    // 1. Куб 2x2x2 (объем = 8.0)
    let cuboid = Cuboid::new(dvec3(1.0, 1.0, 1.0));
    let (v, i) = cuboid.to_trimesh();
    let mesh = TriMesh::new(v, i).unwrap();

    // 2. Сечем ровно пополам (Z = 0)
    let plane = Plane::from_point_and_normal(Point3::origin(), Vector3::z());
    let mut result = plane.slice_mesh(&mesh);

    // 3. Чтобы объем был верным, нужно закрыть "дырку" на срезе (Waterline)
    // Для простого куба и плоскости Z=0 срез — это квадрат с вершинами в (+-1, +-1, 0)
    // В общем случае тут нужна триангуляция многоугольника, но для теста добавим крышку вручную:
    let p1 = Point3::new(-1.0, -1.0, 0.0);
    let p2 = Point3::new(1.0, -1.0, 0.0);
    let p3 = Point3::new(1.0, 1.0, 0.0);
    let p4 = Point3::new(-1.0, 1.0, 0.0);
    
    // Добавляем два треугольника крышки (нормалью вверх)
    result.submerged_triangles.push([p1, p3, p2]);
    result.submerged_triangles.push([p1, p4, p3]);

    // 4. Проверка объема
    let volume = result.volume(&plane);
    let expected_volume = 4.0; // Половина куба 2x2x2
    
    println!("Куб 2x2x2 (объем = 8.0) | Calculated Volume: {}", volume);
    assert!((volume - expected_volume).abs() < 1e-5);
}
///
/// 
#[test]
fn test_volume() {
    for scene in test_scenes() {
        // --- ТЕСТ 1: Плоскость ВЫШЕ фигуры (Объем должен быть 100%) ---
        // Находим самую высокую точку меша
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

        // --- ТЕСТ 2: Плоскость через ЦЕНТР (для симметричных фигур ожидаем ~50%) ---
        // Это сработает для Cube, Needle, Cross, если они центрированы по Z
        if scene.description == "Simple Cube" || scene.description == "3D Cross" {
            for (plane, factor) in test_planes(&scene.mesh) {
                let t = Instant::now();
                let sliced_half = plane.slice_mesh(&scene.mesh);
                let slice_elapsed = t.elapsed();
                // let result = cap_and_get_volume(&sliced_half, &plane);
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
///
/// 
#[test]
fn test_hydrostatics() {
    for scene in test_scenes() {
        // --- ТЕСТ 1: Плоскость ВЫШЕ фигуры (Объем должен быть 100%) ---
        // Находим самую высокую точку меша
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

        // --- ТЕСТ 2: Плоскость через ЦЕНТР (для симметричных фигур ожидаем ~50%) ---
        // Это сработает для Cube, Needle, Cross, если они центрированы по Z
        if scene.description == "Simple Cube" || scene.description == "3D Cross" {
            for (plane, factor) in test_planes(&scene.mesh) {
                let t = Instant::now();
                let sliced_half = plane.slice_mesh(&scene.mesh);
                let slice_elapsed = t.elapsed();
                // let result = cap_and_get_volume(&sliced_half, &plane);
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
// Закрываем крышку (Capping)
fn cap_and_get_volume(sliced: &SlicedMesh, plane: &Plane) -> f64 {
    if sliced.submerged_triangles.is_empty() { return 0.0; }
    
    // ВАЖНО: Все тетраэдры должны сходиться в точку НА плоскости.
    // Это гарантирует, что "крышка" будет иметь нулевой вклад в объем 
    // относительно плоскости, и мы получим чистый объем под ней.
    let anchor = plane.normal * plane.d; // Точка проекции начала координат на плоскость
    let anchor_p = Point3::from_slice(&[anchor.x, anchor.y, anchor.z]);

    let mut total_volume = 0.0;

    // 1. Объем исходных (подводных) граней
    for [p1, p2, p3] in &sliced.submerged_triangles {
        // Считаем объем тетраэдра относительно нашей точки на плоскости
        let v1 = p1 - anchor_p;
        let v2 = p2 - anchor_p;
        let v3 = p3 - anchor_p;
        total_volume += v1.dot(&v2.cross(&v3)) / 6.0;
    }

    // 2. Объем крышки (если использовать anchor_p как центр веера, 
    // то вклад этих тетраэдров будет равен 0, так как они плоские.
    // Но для надежности формулы мы считаем только submerged_triangles 
    // относительно точки на срезе)

    total_volume.abs()
}

///
/// Секущие плоскости
fn test_planes(mesh: &TriMesh) -> Vec<(Plane, f64)> {
    // Считаем границы фигуры (AABB)
    let aabb = mesh.aabb(&parry3d_f64::glamx::DPose3::IDENTITY); // У TriMesh в parry3d есть этот метод
    let center = aabb.mins.lerp(aabb.maxs, 0.5);   // Точка ровно посередине фигуры
    // Создаем плоскости
    return vec![
        // --- ГРАНИЧНЫЕ СЛУЧАИ ---
        // Чуть-чуть ВЫШЕ нижней грани (почти 0 объема)
        (Plane::from_point_and_normal(Point3::new(center.x, center.y, aabb.mins.z + 1e-4), Vector3::z()), 0.0),
        // Чуть-чуть НИЖЕ верхней грани (почти 100% объема)
        (Plane::from_point_and_normal(Point3::new(center.x, center.y, aabb.maxs.z - 1e-4), Vector3::z()), 1.0),
        // Гарантированно ВЫШЕ всей фигуры
        (Plane::from_point_and_normal(Point3::new(center.x, center.y, aabb.maxs.z + 1.0), Vector3::z()), 1.0),
        // --- СЛОЖНЫЕ УГЛЫ (через центр) ---
        (Plane::from_point_and_normal(Point3::from_slice(&[center.x, center.y, center.z]), Vector3::new(1.0, 1.0, 1.0).normalize()), 0.5),
        (Plane::from_point_and_normal(Point3::from_slice(&[center.x, center.y, center.z]), Vector3::new(0.001, 0.001, 1.0).normalize()), 0.5),
        (Plane::from_point_and_normal(Point3::from_slice(&[center.x, center.y, center.z]), Vector3::new(0.3, 0.5, 0.8).normalize()), 0.5),
        // Плоскость, наклоненная
        (Plane::from_point_and_normal(Point3::new(center.x, center.y, center.z), Vector3::new(0.3, 0.5, 0.8).normalize()), 0.5),
    ]
}


///
struct TestScene {
    pub mesh: TriMesh,
    pub full_volume: f64,
    pub description: &'static str,
}

/// Если ваш код пройдет все эти фигуры при разных углах наклона плоскости (например, normal: [1.0, 0.5, -0.2].normalize()), значит, алгоритм сечения крайне надежен.
fn test_scenes() -> Vec<TestScene> {
    vec![
        // 1. Единичный куб
        create_box(2.0, 2.0, 2.0, dvec3(0.0, 0.0, 0.0), "Simple Cube"),
        // 1. Единичный куб, смещенный (проверка инвариантности к позиции)
        create_box(1.0, 1.0, 1.0, dvec3(10.0, 10.0, 10.0), "Simple Offset Cube"),
        // 2. Длинная тонкая пластина (проверка численной точности на узких гранях)
        create_box(10.0, 0.01, 10.0, parry3d_f64::glamx::DVec3::ZERO, "Thin Plate"),
        // 3. Г-образная фигура (первая вогнутость)
        // Два куба 1x1x1: один в (0,0,0), второй в (1,0,0). Итоговый объем 2.0
        create_l_shape("L-Shape Concave"),
        // 4. "Колодец" (глубокая вогнутость)
        // 4 куба по бокам + 1 снизу. Каждый 1x1x1. Объем 5.0
        create_well("Deep Well"),
        // 5. Две несвязанные фигуры (проверка обработки разрозненных островов)
        // Два куба по 1.0. Объем 2.0
        create_two_islands("Two Disconnected Cubes"),
        // 6. Пирамида (наклонные грани, не параллельные осям)
        create_pyramid(2.0, "Square Pyramid"),
        // 7. Сверх-тонкий "игольчатый" меш (проверка на вырожденные треугольники)
        create_needle(100.0, 0.001, "Needle Shape"),
        // 8. Крест (множественные вогнутости по всем осям)
        create_cross("3D Cross"),
        // 9. "Лестница" (множество мелких ступенек)
        create_stairs(5, "Stairs (Multiple Steps)"),
        // 10. Тетраэдр (самый простой, но жесткий тест для алгоритмов сечения)
        create_tetrahedron(1.0, "Tetrahedron"),
    ]
}

// --- Вспомогательные функции для генерации (примеры реализации) ---

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
    // Создаем два куба 1x1x1 (половины граней = 0.5)
    let (mut v1, mut i1) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();
    let (v2, i2) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();

    let vertex_offset = v1.len() as u32; // Для стандартного куба это 8

    // 1. Сдвигаем второй куб по оси X на 1.0, чтобы он прилегал к первому
    let v2_off: Vec<parry3d_f64::glamx::DVec3> = v2.into_iter()
        .map(|p| p + dvec3(1.0, 0.0, 0.0)) 
        .collect();

    // 2. Смещаем индексы: каждый индекс второго меша должен указывать на новые вершины в общем списке
    let i2_off: Vec<[u32; 3]> = i2.into_iter()
        .map(|f| [f[0] + vertex_offset, f[1] + vertex_offset, f[2] + vertex_offset])
        .collect();

    v1.extend(v2_off);
    i1.extend(i2_off);

    TestScene {
        // Используем TriMesh::new(...), в новых версиях возвращает Result, поэтому .expect
        mesh: TriMesh::new(v1, i1).expect("Failed to create TriMesh"),
        full_volume: 2.0, // Два куба 1x1x1
        description: desc,
    }
}
/// 4. "Колодец" (глубокая вогнутость)
/// 4 куба по бокам + 1 снизу. Каждый 1x1x1. Объем 5.0
fn create_well(desc: &'static str) -> TestScene {
    let mut all_v = Vec::new();
    let mut all_i = Vec::new();
    // Смещения для 5 кубов: 1 основание + 4 стенки
    let offsets = vec![
        dvec3(0.0, 0.0, 0.0),  // Дно
        dvec3(1.0, 0.0, 1.0),  // Стенка +X
        dvec3(-1.0, 0.0, 1.0), // Стенка -X
        dvec3(0.0, 1.0, 1.0),  // Стенка +Y
        dvec3(0.0, -1.0, 1.0), // Стенка -Y
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
/// 5. Две несвязанные фигуры (проверка обработки разрозненных островов)
/// Два куба по 1.0. Объем 2.0
fn create_two_islands(desc: &'static str) -> TestScene {
    let (mut v1, mut i1) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();
    let (v2, i2) = Cuboid::new(dvec3(0.5, 0.5, 0.5)).to_trimesh();
    // Разносим их далеко друг от друга
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
///
/// 6. Пирамида (наклонные грани, не параллельные осям)
fn create_pyramid(size: f64, desc: &'static str) -> TestScene {
    let h = size; // Высота
    let s = size / 2.0; // Половина основания
    let vertices = vec![
        dvec3(-s, -s, 0.0), dvec3(s, -s, 0.0), 
        dvec3(s, s, 0.0), dvec3(-s, s, 0.0),
        dvec3(0.0, 0.0, h) // Вершина
    ];
    let indices = vec![
        [0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4], // Бока
        [0, 2, 1], [0, 3, 2] // Основание
    ];
    TestScene {
        mesh: TriMesh::new(vertices, indices).expect("Pyramid failed"),
        full_volume: (size * size * h) / 3.0,
        description: desc,
    }
}
///
/// 7. Сверх-тонкий "игольчатый" меш (проверка на вырожденные треугольники)
fn create_needle(length: f64, thickness: f64, desc: &'static str) -> TestScene {
    // We create a very thin box (needle)
    // Half-extents: length/2, thickness/2, thickness/2
    let h_len = length / 2.0;
    let h_thick = thickness / 2.0;
    let (v, i) = Cuboid::new(dvec3(h_len, h_thick, h_thick)).to_trimesh();
    // We rotate the needle slightly so it's not perfectly aligned with axes
    // This forces the intersection algorithm to work with non-zero coordinates
    let angle = 0.1; // small rotation in radians
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
///
/// 8. Крест (множественные вогнутости по всем осям)
fn create_cross(desc: &'static str) -> TestScene {
    let mut all_v = Vec::new();
    let mut all_i = Vec::new();
    // Три балки 3.0 x 1.0 x 1.0, ориентированные по осям X, Y и Z
    let dims = [
        dvec3(1.5, 0.5, 0.5), // Балка вдоль X
        dvec3(0.5, 1.5, 0.5), // Балка вдоль Y
        dvec3(0.5, 0.5, 1.5), // Балка вдоль Z
    ];
    for half_extents in dims {
        let (v, i) = Cuboid::new(half_extents).to_trimesh();
        let v_len = all_v.len() as u32;
        
        all_v.extend(v);
        all_i.extend(i.into_iter().map(|f| [f[0] + v_len, f[1] + v_len, f[2] + v_len]));
    }
    // ТЕОРЕТИЧЕСКИЙ ОБЪЕМ:
    // Каждая балка = 3 * 1 * 1 = 3.0. Всего 3 балки.
    // Они пересекаются в центре (куб 1x1x1).
    // Общий объем = (3 + 3 + 3) - (2 * объем центрального куба) = 9.0 - 2.0 = 7.0.
    // ВНИМАНИЕ: Если ваш меш содержит внутренние грани, сумма тетраэдров может выдать 9.0!
    TestScene {
        mesh: TriMesh::new(all_v, all_i).expect("Cross mesh failed"),
        full_volume: 9.0, //    7.0, 
        description: desc,
    }
}
///
/// 9. "Лестница" (множество мелких ступенек)
fn create_stairs(steps: usize, desc: &'static str) -> TestScene {
    let mut all_v = Vec::new();
    let mut all_i = Vec::new();
    let step_width = 1.0;
    let step_height = 0.2;
    let step_depth = 0.2;
    for i in 0..steps {
        // Каждая ступень — это блок step_width x step_depth x step_height
        let half_extents = dvec3(step_width / 2.0, step_depth / 2.0, step_height / 2.0);
        let (v, idx) = Cuboid::new(half_extents).to_trimesh();
        // Смещаем каждую ступень: по Y (вглубь) и по Z (вверх)
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
    // Объем одной ступени = w * d * h
    let single_step_vol = step_width * step_depth * step_height;
    TestScene {
        mesh: TriMesh::new(all_v, all_i).expect("Stairs mesh failed"),
        full_volume: single_step_vol * steps as f64,
        description: desc,
    }
}
///
/// 10. Тетраэдр (самый простой, но жесткий тест для алгоритмов сечения)
fn create_tetrahedron(a: f64, desc: &'static str) -> TestScene {
    // Координаты вершин равностороннего тетраэдра с ребром a
    let h = a * (2.0 / 3.0).sqrt(); // Высота
    let r = a / (3.0).sqrt();       // Радиус описанной окружности основания
    let vertices = vec![
        dvec3(r, 0.0, 0.0),                               // V0
        dvec3(-r / 2.0, a / 2.0, 0.0),                    // V1
        dvec3(-r / 2.0, -a / 2.0, 0.0),                   // V2
        dvec3(0.0, 0.0, h),                               // V3 (Вершина)
    ];
    let indices = vec![
        [0, 1, 2], // Основание (по часовой или против — проверьте нормали)
        [0, 2, 3], // Бок 1
        [2, 1, 3], // Бок 2
        [1, 0, 3], // Бок 3
    ];
    // Формула объема правильного тетраэдра: V = (a^3) / (6 * sqrt(2))
    let volume = a.powi(3) / (6.0 * 2.0f64.sqrt());
    TestScene {
        mesh: TriMesh::new(vertices, indices).expect("Tetrahedron failed"),
        full_volume: volume,
        description: desc,
    }
}
