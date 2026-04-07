use std::{path::Path, time::Instant};

use parry3d_f64::{glamx::{Vec3, dvec3}, shape::{Shape, TriMesh}};

use crate::{load_stl, tools::{FloatingPosition, LoadingCondition, SolverConfig, find_equilibrium}};

#[test]
fn basic() {
    // Если STL в мм, scale 0.001 переводит всё в метры.
    // Тогда все допуски (tolerance) теперь тоже в метрах и м³.
    let scale = 0.001f64;

    for path in ["assets/ark.stl", "assets/Sofiya.stl"] {
        let mesh = load_stl(Path::new(path)).scaled(dvec3(scale, scale, scale));
        // Получаем AABB, чтобы понять габариты и задать адекватный target_volume
        let aabb = mesh.compute_aabb(&parry3d_f64::glamx::DPose3::identity());
        let total_height = aabb.maxs.z - aabb.mins.z;
        let conf = SolverConfig {
            max_iterations: 20,    // Newton-Raphson обычно сходится за 5-10 шагов. 20 — с запасом.
            tolerance: 1e-4,       // Точность 0.1 мм для координат и 0.0001 м³ для объема.
            delta_z: 1e-4,         // Шаг для численной производной. f64 позволяет брать малые значения.
            delta_angle: 1e-4,     // Шаг для численной производной. f64 позволяет брать малые значения.
        };
        // Настраиваем "погрузку" на ~50% от высоты меша
        let loading_condition = LoadingCondition {
            target_volume: 50.0,   // Значение "от фонаря", в идеале вычисляется как Mass / Density
            target_lcg: (aabb.maxs.x + aabb.mins.x) / 2.0, // Центр тяжести по центру модели
            target_tcg: 0.0,                               // Без крена
        };
        let initial_guess = FloatingPosition {
            draft_z: aabb.mins.z + (total_height * 0.4), // Начинаем чуть ниже середины
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
#[test]
fn solver_cube_equilibrium() {
    let mesh = create_test_cube(2.0);
    let target = LoadingCondition {
        target_volume: 4.0,
        target_lcg: 0.0,
        target_tcg: 0.0,
    };
    let result = find_equilibrium(
        &mesh,
        &target,
        FloatingPosition {
            draft_z: 0.2,
            heel_rx: 0.1,
            trim_ry: -0.1,
        },
        &SolverConfig {
            max_iterations: 50,
            tolerance: 1e-6,
            delta_z: 1e-5,         // Шаг для численной производной. f64 позволяет брать малые значения.
            delta_angle: 1e-5,     // Шаг для численной производной. f64 позволяет брать малые значения.
        }
    ).unwrap();

    // assert!((result.draft_z - 0.0).abs() < 1e-3);
    // assert!(result.draft_z.abs() < 1e-2);
    let plane = result.to_plane();
    let hydro = plane.slice_mesh(&mesh).hydrostatics(&plane);
    assert!((hydro.volume - target.target_volume).abs() < 1e-3);
    assert!(result.heel_rx.abs() < 1e-3);
    assert!(result.trim_ry.abs() < 1e-3);
}
fn create_test_cube(size: f64) -> TriMesh {
    let s = size / 2.0;

    let vertices = vec![
        dvec3(-s, -s, -s),
        dvec3( s, -s, -s),
        dvec3( s,  s, -s),
        dvec3(-s,  s, -s),
        dvec3(-s, -s,  s),
        dvec3( s, -s,  s),
        dvec3( s,  s,  s),
        dvec3(-s,  s,  s),
    ];

    let indices = vec![
        [0,1,2],[0,2,3], // bottom
        [4,5,6],[4,6,7], // top
        [0,1,5],[0,5,4],
        [1,2,6],[1,6,5],
        [2,3,7],[2,7,6],
        [3,0,4],[3,4,7],
    ];

    TriMesh::new(vertices, indices).unwrap()
}
#[test]
fn test_equilibrium_cube_draft() {
    let mesh = create_test_cube(2.0); // volume = 8

    let target = LoadingCondition {
        target_volume: 4.0, // половина
        target_lcg: 0.0,
        target_tcg: 0.0,
    };

    let result = find_equilibrium(
        &mesh,
        &target,
        FloatingPosition {
            draft_z: -0.5, // плохой старт
            heel_rx: 0.2,
            trim_ry: -0.2,
        },
        &SolverConfig {
            max_iterations: 50,
            tolerance: 1e-6,
            delta_z: 1e-5,         // Шаг для численной производной. f64 позволяет брать малые значения.
            delta_angle: 1e-5,     // Шаг для численной производной. f64 позволяет брать малые значения.
        },
    ).expect("Solver failed");

    // половина куба → ватерлиния на z = 0
    // assert!((result.draft_z - 0.0).abs() < 1e-3);
    // assert!(result.draft_z.abs() < 1e-2);
    let plane = result.to_plane();
    let hydro = plane.slice_mesh(&mesh).hydrostatics(&plane);
    assert!((hydro.volume - target.target_volume).abs() < 1e-3);
    assert!(result.heel_rx.abs() < 1e-3);
    assert!(result.trim_ry.abs() < 1e-3);
}
#[test]
fn test_translation_invariance() {
    let offset = dvec3(10.0, -5.0, 3.0);
    let mesh = translate_mesh(&create_test_cube(2.0), dvec3(10.0, -5.0, 3.0));
    // mesh = mesh.translated(offset);

    let target = LoadingCondition {
        target_volume: 4.0,
        target_lcg: offset.x,
        target_tcg: offset.y,
    };

    let result = find_equilibrium(
        &mesh,
        &target,
        FloatingPosition {
            draft_z: offset.z,
            heel_rx: 0.1,
            trim_ry: 0.1,
        },
        &SolverConfig {
            max_iterations: 50,
            tolerance: 1e-6,
            delta_z: 1e-5,         // Шаг для численной производной. f64 позволяет брать малые значения.
            delta_angle: 1e-5,     // Шаг для численной производной. f64 позволяет брать малые значения.
        },
    ).unwrap();

    assert!((result.draft_z - offset.z).abs() < 1e-3);
    assert!(result.heel_rx.abs() < 1e-3);
    assert!(result.trim_ry.abs() < 1e-3);
}
#[test]
fn test_random_initial_guess() {
    let mesh = create_test_cube(2.0);

    let target = LoadingCondition {
        target_volume: 4.0,
        target_lcg: 0.0,
        target_tcg: 0.0,
    };

    for i in 0..10 {
        let result = find_equilibrium(
            &mesh,
            &target,
            FloatingPosition {
                draft_z: -1.0 + i as f64 * 0.2,
                heel_rx: (i as f64 * 0.1),
                trim_ry: -(i as f64 * 0.1),
            },
            &SolverConfig {
                max_iterations: 50,
                tolerance: 1e-5,
                delta_z: 1e-5,         // Шаг для численной производной. f64 позволяет брать малые значения.
                delta_angle: 1e-5,     // Шаг для численной производной. f64 позволяет брать малые значения.
            },
        );

        assert!(result.is_ok(), "Failed on iteration {}", i);

        let pos = result.unwrap();
        assert!(pos.heel_rx.abs() < 1e-2);
        assert!(pos.trim_ry.abs() < 1e-2);
    }
}
#[test]
fn test_rotated_cube_equilibrium() {
    let mesh = create_test_cube(2.0);

    let target = LoadingCondition {
        target_volume: 4.0,
        target_lcg: 0.0,
        target_tcg: 0.0,
    };

    let result = find_equilibrium(
        &mesh,
        &target,
        FloatingPosition {
            draft_z: 0.0,
            heel_rx: 0.5, // сильный наклон
            trim_ry: 0.5,
        },
        &SolverConfig {
            max_iterations: 50,
            tolerance: 1e-5,
            delta_z: 1e-5,         // Шаг для численной производной. f64 позволяет брать малые значения.
            delta_angle: 1e-5,     // Шаг для численной производной. f64 позволяет брать малые значения.
        },
    ).unwrap();

    assert!(result.heel_rx.abs() < 1e-2);
    assert!(result.trim_ry.abs() < 1e-2);
}
#[test]
fn test_small_waterline_area() {
    let mesh = create_test_cube(2.0);

    let target = LoadingCondition {
        target_volume: 0.01, // почти пустой
        target_lcg: 0.0,
        target_tcg: 0.0,
    };

    let result = find_equilibrium(
        &mesh,
        &target,
        FloatingPosition {
            draft_z: -0.99,
            heel_rx: 0.0,
            trim_ry: 0.0,
        },
        &SolverConfig {
            max_iterations: 50,
            tolerance: 1e-5,
            delta_z: 1e-5,         // Шаг для численной производной. f64 позволяет брать малые значения.
            delta_angle: 1e-5,     // Шаг для численной производной. f64 позволяет брать малые значения.
        },
    );

    assert!(result.is_ok(), "Solver unstable near zero waterline");
}
/// несколько тел (islands)
#[test]
fn test_two_bodies() {
    let cube1 = create_test_cube(2.0);
    // let cube2 = create_test_cube(2.0).translated(dvec3(5.0, 0.0, 0.0));
    let cube2 = translate_mesh(&cube1, dvec3(5.0, 0.0, 0.0));
    let mesh = merge_meshes(&cube1, &cube2);

    let target = LoadingCondition {
        target_volume: 4.0, // половина одного куба
        target_lcg: 0.0,
        target_tcg: 0.0,
    };

    let result = find_equilibrium(
        &mesh,
        &target,
        FloatingPosition {
            draft_z: 0.0,
            heel_rx: 0.0,
            trim_ry: 0.0,
        },
        &SolverConfig {
            max_iterations: 50,
            tolerance: 1e-5,
            delta_z: 1e-5,         // Шаг для численной производной. f64 позволяет брать малые значения.
            delta_angle: 1e-5,     // Шаг для численной производной. f64 позволяет брать малые значения.
        },
    );

    assert!(result.is_ok());
}
// #[test]
// fn test_regression_stability() {
//     let mesh = create_test_cube(2.0);

//     let target = LoadingCondition {
//         target_volume: 3.3,
//         target_lcg: 0.2,
//         target_tcg: -0.1,
//     };

//     let result = find_equilibrium(
//         &mesh,
//         &target,
//         FloatingPosition {
//             draft_z: 0.1,
//             heel_rx: 0.1,
//             trim_ry: 0.1,
//         },
//         &SolverConfig {
//             max_iterations: 50,
//             tolerance: 1e-6,
//             delta: 1e-5,
//         },
//     ).unwrap();

//     // фиксируешь текущее поведение
//     assert!((result.draft_z - EXPECTED_DRAFT).abs() < 1e-4);
// }
fn translate_mesh(mesh: &TriMesh, offset: parry3d_f64::glamx::DVec3) -> TriMesh {
    let vertices: Vec<parry3d_f64::glamx::DVec3> = mesh
        .vertices()
        .iter()
        .map(|v| *v + offset)
        .collect();
    let indices = mesh.indices().to_vec();
    TriMesh::new(vertices, indices).unwrap()
}
fn merge_meshes(a: &TriMesh, b: &TriMesh) -> TriMesh {
    let mut vertices = a.vertices().to_vec();
    let mut indices = a.indices().to_vec();
    let offset = vertices.len() as u32;
    vertices.extend_from_slice(b.vertices());
    indices.extend(
        b.indices()
            .iter()
            .map(|tri| [tri[0] + offset, tri[1] + offset, tri[2] + offset])
    );
    TriMesh::new(vertices, indices).unwrap()
}
