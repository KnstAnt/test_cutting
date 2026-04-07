use std::{path::Path, time::Instant};

use parry3d_f64::{glamx::dvec3, shape::Shape};

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
            delta: 1e-5,           // Шаг для численной производной. f64 позволяет брать малые значения.
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