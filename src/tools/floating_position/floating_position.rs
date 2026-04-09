use nalgebra::{Matrix3, Point3, Rotation3, Vector3};
use parry3d_f64::shape::TriMesh;

use crate::tools::{Hydrostatics, Plane};

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
    // ///
    // /// Переводит углы и осадку в секущую плоскость.
    // /// Вращение происходит вокруг начала координат мира.
    // pub fn to_plane(&self) -> Plane {
    //     // Создаем матрицу вращения из углов Эйлера (крен и дифферент)
    //     let rotation = Rotation3::from_euler_angles(self.heel_rx, self.trim_ry, 0.0);
    //     // Вектор мировой вертикали (Z) поворачивается вместе с судном
    //     // Важно: в зависимости от вашей системы координат, возможно нужно вращать (0,1,0)
    //     let normal = rotation * Vector3::new(0.0, 0.0, 1.0); 
    //     let point_on_plane = Point3::new(0.0, 0.0, self.draft_z);        
    //     Plane::from_point_and_normal(point_on_plane, normal)
    // }
    ///
    /// Переводит углы и осадку в секущую плоскость
    /// ВАЖНО: Тут мы принимаем точку привязки (обычно LCG/TCG цели),
    /// чтобы вращение происходило вокруг центра тяжести (LCG, TCG) судна, а не вокруг начала координат мира.
    pub fn to_plane_relative(&self, ref_x: f64, ref_y: f64) -> Plane {
        let rotation = Rotation3::from_euler_angles(self.heel_rx, self.trim_ry, 0.0);
        let normal = rotation * Vector3::new(0.0, 0.0, 1.0); 
        // Плоскость проходит через draft_z в точке (ref_x, ref_y)
        let point_on_plane = Point3::new(ref_x, ref_y, self.draft_z);        
        Plane::from_point_and_normal(point_on_plane, normal)
    }
}
///
/// 
pub struct SolverConfig {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub delta_z: f64, // Шаг осадки для вычисления производных (например, 1e-4)
    pub delta_angle: f64,   // Шаг углов для вычисления производных (например, 1e-4)
}
///
/// 
pub fn find_equilibrium(
    mesh: &TriMesh, 
    target: &LoadingCondition, 
    initial_guess: FloatingPosition,
    config: &SolverConfig
) -> Result<FloatingPosition, String> {
    let mut current_pos = initial_guess;
    let mut lambda = 1e-2;
    // Динамический масштаб для ограничений шага на основе объема судна
    let l_scale = target.target_volume.cbrt().max(0.1);

    for i in 0..config.max_iterations {
        // Используем относительную плоскость для стабильности вращения
        let plane = current_pos.to_plane_relative(target.target_lcg, target.target_tcg);
        let slice = plane.slice_mesh(mesh);
        let hydro = slice.hydrostatics(&plane);

        // ФАЗА 1: Если мы почти в воздухе, углы не имеют смысла
        let volume_ratio = hydro.volume / target.target_volume;
        println!("✅ На итерации {}: volume_ratio: {}", i, volume_ratio);
        let is_too_shallow = volume_ratio < 0.05; // Менее 5% погружения

        // Нормализуем вектор ошибок: теперь всё в МЕТРАХ
        let f_x = make_f(&hydro, target);
        if !f_x.norm().is_finite() {
            return Err(format!("Уравнение разошлось (NaN). Проверьте целостность геометрии."));
        }
        let current_fx_norm = f_x.norm(); // Считаем 1 раз за итерацию
        if current_fx_norm < config.tolerance {
            println!("✅ Сходимость на итерации {}: fx={}", i, current_fx_norm);
            return Ok(current_pos);
        }

        // Численный Якобиан
        let mut jacobian = calculate_normalized_jacobian(mesh, current_pos, &config, target);
        // Если судно слишком высоко, принудительно зануляем влияние углов в Якобиане,
        // чтобы решатель просто "топил" куб вниз, не пытаясь его вращать в воздухе.
        if is_too_shallow {
            println!("✅ На итерации {}: volume_ratio: {}", i, volume_ratio);
            jacobian.set_column(1, &Vector3::zeros());
            jacobian.set_column(2, &Vector3::zeros());
        }
        let mut jtj = jacobian.transpose() * jacobian;
        // --- ЗАЩИТА ОТ "СУДНА ВНЕ ВОДЫ" (Нулевой Якобиан) ---
        let diag = Vector3::new(jtj[(0,0)], jtj[(1,1)], jtj[(2,2)]);
        if diag[0] < 1e-9 {    // diag.max()
            // Если мы ниже воды — выплываем, если выше — тонем
            let nudge = if hydro.volume < target.target_volume { 0.1 * l_scale } else { -0.1 * l_scale };
            current_pos.draft_z += nudge;
            continue;
        }
        // --- MARQUARDT SCALING ---
        // Автоматически балансирует масштабы радиан и метров Marquardt (без жестких констант)
        jtj[(0,0)] += lambda * diag[0].max(1e-4);
        jtj[(1,1)] += lambda * diag[1].max(1e-4);
        jtj[(2,2)] += lambda * diag[2].max(1e-4);
        let gradient = jacobian.transpose() * -f_x;
        if let Some(mut step) = jtj.lu().solve(&gradient) {
            // Динамическое ограничение шага Ньютона
            step[0] = step[0].clamp(-l_scale * 0.5, l_scale * 0.5);
            step[1] = step[1].clamp(-0.05, 0.05); // ~11 градусов
            step[2] = step[2].clamp(-0.05, 0.05);
            println!("✅ На итерации {}: Шага Ньютона: {:?}", i, step);

            let mut alpha = 1.0;
            let mut improved = false;
            
            // --- BACKTRACKING LINE SEARCH ---
            // Пробуем уменьшать шаг, если он делает хуже
            for _ in 0..8 {
                // if alpha < 1e-4 { break; }
                let max_angle_step = 0.05;
                let step_heel = step[1].clamp(-max_angle_step, max_angle_step);
                let step_trim = step[2].clamp(-max_angle_step, max_angle_step);
                let trial_pos = FloatingPosition {
                    draft_z: current_pos.draft_z + step[0] * alpha,
                    heel_rx: current_pos.heel_rx + step_heel * alpha,
                    trim_ry: current_pos.trim_ry + step_trim * alpha,
                };
                
                let t_plane = trial_pos.to_plane_relative(target.target_lcg, target.target_tcg);
                let t_hydro = t_plane.slice_mesh(mesh).hydrostatics(&t_plane);
                let t_fx = make_f(&t_hydro, target);
                if t_fx.norm() < current_fx_norm {
                    current_pos = trial_pos;
                    improved = true;
                    break;
                }
                alpha *= 0.5; // Слишком большой прыжок, уменьшаем вдвое
            }

            if improved {
                // Шаг успешен, уменьшаем влияние градиентного спуска lambda
                // current_pos = best_pos;
                lambda = (lambda * 0.1).max(1e-7);
            } else {
                // Шаг неудачен, увеличиваем lambda (переход к градиентному спуску)
                // НИКАКИХ РУЧНЫХ СДВИГОВ ЗДЕСЬ! На следующей итерации алгоритм сам сделает правильный шаг
                lambda = (lambda * 10.0).min(1e4);
                // Если застряли, просто немного меняем осадку (Line search failed)
                current_pos.draft_z += (target.target_volume - hydro.volume).signum() * 0.01 * l_scale;
            }
        } else {
            // Матрица выродилась (почти невозможно с LM, но на всякий случай)
            // Фолбэк на случай экстремальной числовой нестабильности
            lambda = lambda * 10.0;
        }
    }
    // Проверка финального результата перед выходом
    let final_plane = current_pos.to_plane_relative(target.target_lcg, target.target_tcg);
    let final_hydro = final_plane.slice_mesh(mesh).hydrostatics(&final_plane);
    if make_f(&final_hydro, target).norm() < config.tolerance * 50.0 {
        Ok(current_pos)
    } else {
        Err(format!("Diverged after max iterations {}", config.max_iterations))
    }
}
///
/// Численный Якобиан
fn calculate_normalized_jacobian(
    mesh: &TriMesh, 
    current_pos: FloatingPosition, 
    conf: &SolverConfig,
    target: &LoadingCondition
) -> Matrix3<f64> {
    let mut jacobian = Matrix3::zeros();
    let get_f = |pos: FloatingPosition| {
        let p = pos.to_plane_relative(target.target_lcg, target.target_tcg);
        make_f(&p.slice_mesh(mesh).hydrostatics(&p), &target)
    };
    // Z Осадка
    let f_z2 = get_f(FloatingPosition { draft_z: current_pos.draft_z + conf.delta_z, ..current_pos });
    let f_z1 = get_f(FloatingPosition { draft_z: current_pos.draft_z - conf.delta_z, ..current_pos });
    jacobian.set_column(0, &((f_z2 - f_z1) / (2.0 * conf.delta_z)));
    // Rx Крен
    let f_rx2 = get_f(FloatingPosition { heel_rx: current_pos.heel_rx + conf.delta_angle, ..current_pos });
    let f_rx1 = get_f(FloatingPosition { heel_rx: current_pos.heel_rx - conf.delta_angle, ..current_pos });
    jacobian.set_column(1, &((f_rx2 - f_rx1) / (2.0 * conf.delta_angle)));
    // Ry Дифферент
    let f_ry2 = get_f(FloatingPosition { trim_ry: current_pos.trim_ry + conf.delta_angle, ..current_pos });
    let f_ry1 = get_f(FloatingPosition { trim_ry: current_pos.trim_ry - conf.delta_angle, ..current_pos });
    jacobian.set_column(2, &((f_ry2 - f_ry1) / (2.0 * conf.delta_angle)));
    jacobian
}
///
/// Вычисление вектора остатков (Residuals)
fn make_f(h: &Hydrostatics, target: &LoadingCondition) -> Vector3<f64> {
    let v_scale = target.target_volume.max(1e-7);
    let l_scale = target.target_volume.cbrt().max(1e-3); 
    let vol_err = (h.volume - target.target_volume) / v_scale;
    // Если объема нет, CoB будет NaN. Заменяем на 0, чтобы не "взрывать" Якобиан.
    if h.volume < 1e-8 || !h.center_of_buoyancy.x.is_finite() {
        return Vector3::new(vol_err * 2.0, 0.0, 0.0);
    }
    Vector3::new(
        vol_err,
        (h.center_of_buoyancy.x - target.target_lcg) / l_scale,
        (h.center_of_buoyancy.y - target.target_tcg) / l_scale,
    )
}
