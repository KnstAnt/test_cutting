use nalgebra::{Matrix3, Point3, Rotation3, Vector3};
use parry3d_f64::shape::{Shape, TriMesh};

use crate::tools::Plane;

///
/// Целевые параметры (Дано)
pub struct LoadingCondition {
    pub target_volume: f64, // Масса / Плотность воды
    pub target_lcg: f64,    // Продольный центр тяжести
    pub target_tcg: f64,    // Поперечный центр тяжести
}
///
/// Текущее положение судна
#[derive(Debug, Clone, Copy)]
pub struct FloatingPosition {
    pub draft_z: f64,
    pub heel_rx: f64,
    pub trim_ry: f64,
}
//
impl FloatingPosition {
    /// Переводит углы и осадку в секущую плоскость
    pub fn to_plane(&self) -> Plane {
        // Создаем матрицу вращения из углов Эйлера (крен и дифферент)
        let rotation = Rotation3::from_euler_angles(self.heel_rx, self.trim_ry, 0.0);
        // Вектор мировой вертикали (Z) поворачивается вместе с судном
        // Важно: в зависимости от вашей системы координат, возможно нужно вращать (0,1,0)
        let normal = rotation * Vector3::new(0.0, 0.0, 1.0); 
        let point_on_plane = Point3::new(0.0, 0.0, self.draft_z);
        
        Plane::from_point_and_normal(point_on_plane, normal)
    }
}
///
/// 
pub struct SolverConfig {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub delta: f64, // Шаг для вычисления производных (например, 1e-4)
}
///
/// 
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
        let area = slice.waterline_area().max(0.1); // Защита от нуля

        // Нормализуем вектор ошибок: теперь всё в МЕТРАХ
        let f_x = Vector3::new(
            (hydro.volume - target.target_volume) / area, // Ошибка в метрах осадки
            hydro.center_of_buoyancy.x - target.target_lcg,
            hydro.center_of_buoyancy.y - target.target_tcg,
        );

        let new_norm = f_x.norm();
        if new_norm < config.tolerance {
            println!("✅ Сходимость на итерации {}: fx={}", i, new_norm);
            return Ok(current_pos);
        }

        // Численный Якобиан (используй ту же логику нормализации внутри!)
        let jacobian = calculate_normalized_jacobian(mesh, current_pos, &f_x, area, config.delta, target);

        if let Some(step) = jacobian.lu().solve(&(-f_x)) {
            // --- BACKTRACKING LINE SEARCH ---
            let mut alpha = 1.0;
            let mut best_pos = current_pos;
            let mut improved = false;

            // Пробуем уменьшать шаг, если он делает хуже
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
                alpha *= 0.5; // Слишком большой прыжок, уменьшаем вдвое
            }

            if !improved {
                // Если даже маленький шаг не помогает, пробуем хоть куда-то сдвинуться
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
    f_x_normalized: &Vector3<f64>, // Это уже нормализованный (V-Vt)/Area
    base_area: f64, 
    config_delta: f64,
    target: &LoadingCondition
) -> Matrix3<f64> {
    let mut jacobian = Matrix3::zeros();
    let delta = config_delta;

    // Вспомогательная функция для получения нормализованного вектора в смещенной точке
    let get_norm_f = |pos: FloatingPosition| {
        let p = pos.to_plane();
        let s = p.slice_mesh(mesh);
        let h = s.hydrostatics(&p);
        // Мы делим на base_area, чтобы производная была согласована с текущим шагом
        Vector3::new(
            (h.volume - target.target_volume) / base_area, 
            h.center_of_buoyancy.x - target.target_lcg,
            h.center_of_buoyancy.y - target.target_tcg,
        )
    };

    // Колонка 0: Draft
    let mut pos_z = current_pos; pos_z.draft_z += delta;
    jacobian.set_column(0, &((get_norm_f(pos_z) - f_x_normalized) / delta));

    // Колонка 1: Heel
    let mut pos_rx = current_pos; pos_rx.heel_rx += delta;
    jacobian.set_column(1, &((get_norm_f(pos_rx) - f_x_normalized) / delta));

    // Колонка 2: Trim
    let mut pos_ry = current_pos; pos_ry.trim_ry += delta;
    jacobian.set_column(2, &((get_norm_f(pos_ry) - f_x_normalized) / delta));

    jacobian
}
///
/// 
// pub fn find_equilibrium(
//     mesh: &TriMesh, 
//     target: &LoadingCondition, 
//     initial_guess: FloatingPosition,
//     config: &SolverConfig
// ) -> Result<FloatingPosition, &'static str> {
//     let aabb = mesh.compute_aabb(&parry3d_f64::glamx::DPose3::identity());
//     let mut current_pos = initial_guess;
//     let relaxation = 0.5; // Начинаем с осторожного шага
//     for i in 0..config.max_iterations {
//         let base_plane = current_pos.to_plane();
//         let base_slice = base_plane.slice_mesh(mesh);
//         let base_hydro = base_slice.hydrostatics(&base_plane);

//         // Вектор ошибок
//         let f_x = Vector3::new(
//             base_hydro.volume - target.target_volume,
//             base_hydro.center_of_buoyancy.x - target.target_lcg,
//             base_hydro.center_of_buoyancy.y - target.target_tcg,
//         );
//         println!("find_equilibrium | fx: {:?}", f_x.norm());
//         if f_x.norm() < config.tolerance {
//             println!("find_equilibrium | Сходимость достигнута на итерации {}", i);
//             return Ok(current_pos);
//         }
//         // Вычисляем Якобиан (как раньше)...
//         let jacobian = calculate_numerical_jacobian(mesh, current_pos, &f_x, config.delta);
//         // Решаем систему J * step = -F
//         if let Some(step) = jacobian.lu().solve(&(-f_x)) {
//             // ПРИМЕНЯЕМ ШАГ С ЗАТУХАНИЕМ
//             current_pos.draft_z += step[0] * relaxation;
//             current_pos.heel_rx += step[1] * relaxation;
//             current_pos.trim_ry += step[2] * relaxation;
//             // ЖЕСТКИЙ КЛАМПИНГ: не даем судну улететь в космос
//             current_pos.draft_z = current_pos.draft_z.clamp(aabb.mins.z, aabb.maxs.z);
//             // Ограничиваем углы (например, не более 80 градусов), чтобы не перевернуться
//             current_pos.heel_rx = current_pos.heel_rx.clamp(-1.4, 1.4);
//             current_pos.trim_ry = current_pos.trim_ry.clamp(-1.4, 1.4);
//         } else {
//             // Если Якобиан "сдох", пробуем сделать маленький шаг чисто по осадке (вертикально)
//             current_pos.draft_z += (target.target_volume - base_hydro.volume).signum() * 0.1;
//         }
//     }
//     Err("Не удалось найти равновесие за отведенное количество итераций")
// }
///
/// Вычисляем Якобиан (Матрицу производных) численно
fn calculate_numerical_jacobian(mesh: &TriMesh, current_pos: FloatingPosition, f_x: &Vector3<f64>, config_delta: f64) -> Matrix3<f64> {
    let mut jacobian = Matrix3::zeros();
    // Сдвиг по Z (Draft)
    let mut pos_z = current_pos;
    pos_z.draft_z += config_delta;
    let d_hydro_z = pos_z.to_plane().slice_mesh(mesh).hydrostatics(&pos_z.to_plane());
    jacobian.set_column(0, &((Vector3::new(d_hydro_z.volume, d_hydro_z.center_of_buoyancy.x, d_hydro_z.center_of_buoyancy.y) - f_x) / config_delta));
    // Сдвиг по Rx (Heel)
    let mut pos_rx = current_pos;
    pos_rx.heel_rx += config_delta;
    let d_hydro_rx = pos_rx.to_plane().slice_mesh(mesh).hydrostatics(&pos_rx.to_plane());
    jacobian.set_column(1, &((Vector3::new(d_hydro_rx.volume, d_hydro_rx.center_of_buoyancy.x, d_hydro_rx.center_of_buoyancy.y) - f_x) / config_delta));
    // Сдвиг по Ry (Trim)
    let mut pos_ry = current_pos;
    pos_ry.trim_ry += config_delta;
    let d_hydro_ry = pos_ry.to_plane().slice_mesh(mesh).hydrostatics(&pos_ry.to_plane());
    jacobian.set_column(2, &((Vector3::new(d_hydro_ry.volume, d_hydro_ry.center_of_buoyancy.x, d_hydro_ry.center_of_buoyancy.y) - f_x) / config_delta));
    jacobian
}



// pub fn find_equilibrium(
//     mesh: &TriMesh, 
//     target: &LoadingCondition, 
//     initial_guess: FloatingPosition,
//     config: &SolverConfig
// ) -> Result<FloatingPosition, &'static str> {
    
//     let mut current_pos = initial_guess;

//     for _ in 0..config.max_iterations {
//         // 1. Оцениваем текущее состояние
//         let base_plane = current_pos.to_plane();
//         let base_slice = base_plane.slice_mesh(mesh);
//         let base_hydro = base_slice.hydrostatics(&base_plane);

//         // Вектор ошибок (residuals)
//         let f_x = Vector3::new(
//             base_hydro.volume - target.target_volume,
//             base_hydro.center_of_buoyancy.x - target.target_lcg, // Если X - продольная ось
//             base_hydro.center_of_buoyancy.y - target.target_tcg, // Если Y - поперечная ось
//         );

//         // Условие выхода (сходимость)
//         if f_x.norm() < config.tolerance {
//             return Ok(current_pos);
//         }

//         // 2. Вычисляем Якобиан (Матрицу производных) численно
//         let mut jacobian = Matrix3::zeros();
        
//         // Сдвиг по Z (Draft)
//         let mut pos_z = current_pos; pos_z.draft_z += config.delta;
//         let d_hydro_z = pos_z.to_plane().slice_mesh(mesh).hydrostatics(&pos_z.to_plane());
//         jacobian.set_column(0, &((Vector3::new(d_hydro_z.volume, d_hydro_z.center_of_buoyancy.x, d_hydro_z.center_of_buoyancy.y) - f_x) / config.delta));

//         // Сдвиг по Rx (Heel)
//         let mut pos_rx = current_pos; pos_rx.heel_rx += config.delta;
//         let d_hydro_rx = pos_rx.to_plane().slice_mesh(mesh).hydrostatics(&pos_rx.to_plane());
//         jacobian.set_column(1, &((Vector3::new(d_hydro_rx.volume, d_hydro_rx.center_of_buoyancy.x, d_hydro_rx.center_of_buoyancy.y) - f_x) / config.delta));

//         // Сдвиг по Ry (Trim)
//         let mut pos_ry = current_pos; pos_ry.trim_ry += config.delta;
//         let d_hydro_ry = pos_ry.to_plane().slice_mesh(mesh).hydrostatics(&pos_ry.to_plane());
//         jacobian.set_column(2, &((Vector3::new(d_hydro_ry.volume, d_hydro_ry.center_of_buoyancy.x, d_hydro_ry.center_of_buoyancy.y) - f_x) / config.delta));

//         // 3. Решаем систему уравнений J * delta = -F
//         // Используем LU-разложение из nalgebra для инвертирования матрицы
//         let step = match jacobian.lu().solve(&(-f_x)) {
//             Some(s) => s,
//             None => return Err("Матрица Якобиана сингулярна (судно полностью вышло из воды или полностью утонуло)"),
//         };

//         // 4. Обновляем положение
//         // Можно добавить коэффициент релаксации (damping) для нестабильных корпусов (например 0.5 * step)
//         current_pos.draft_z += step[0];
//         current_pos.heel_rx += step[1];
//         current_pos.trim_ry += step[2];
//     }

//     Err("Не удалось найти равновесие за отведенное количество итераций")
// }
