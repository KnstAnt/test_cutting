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
) -> Result<FloatingPosition, &'static str> {
    let mut current_pos = initial_guess;
    let mut current_fx_norm = f64::NAN;
    let mut lambda = 1e-2;

    for i in 0..config.max_iterations {
        let plane = current_pos.to_plane();
        let slice = plane.slice_mesh(mesh);
        let hydro = slice.hydrostatics(&plane);

        // Нормализуем вектор ошибок: теперь всё в МЕТРАХ
        let f_x = make_f(&hydro, target);
        let new_norm = f_x.norm();
        if current_fx_norm.is_nan() {
            current_fx_norm = new_norm;
        }
        if new_norm < config.tolerance {
            println!("✅ Сходимость на итерации {}: fx={}", i, new_norm);
            return Ok(current_pos);
        }

        // Численный Якобиан
        let jacobian = calculate_normalized_jacobian(mesh, current_pos, config.delta_z, config.delta_angle, target);
        // Матрица Якобиана с регуляризацией Левенберга-Марквардта
        let jtj = jacobian.transpose() * jacobian + Matrix3::identity() * lambda;
        let gradient = jacobian.transpose() * -f_x;
        if let Some(mut step) = jtj.lu().solve(&gradient) {
            // Ограничитим шаг Ньютона
            let step_norm = step.norm();
            if step_norm > 1.0 {
                step /= step_norm; // Защита от гигантских прыжков
            }
            let mut alpha = 1.0;
            let mut best_pos = current_pos;
            let mut improved = false;
            
            // --- BACKTRACKING LINE SEARCH ---
            // Пробуем уменьшать шаг, если он делает хуже
            for _ in 0..10 {
                if alpha < 1e-4 { break; }
                let trial_pos = FloatingPosition {
                    draft_z: current_pos.draft_z + step[0] * alpha,
                    heel_rx: current_pos.heel_rx + step[1] * alpha,
                    trim_ry: current_pos.trim_ry + step[2] * alpha,
                };
                
                let t_plane = trial_pos.to_plane();
                let t_slice = t_plane.slice_mesh(mesh);
                let t_hydro = t_slice.hydrostatics(&t_plane);

                // Замените блок расчета t_fx на:
                let t_fx = make_f(&t_hydro, target);
                if t_fx.norm() < current_fx_norm {
                    best_pos = trial_pos;
                    current_fx_norm = t_fx.norm();
                    improved = true;
                    break;
                }
                alpha *= 0.5; // Слишком большой прыжок, уменьшаем вдвое
            }

            if improved {
                // Шаг успешен, уменьшаем влияние градиентного спуска
                current_pos = best_pos;
                lambda = (lambda * 0.3).max(1e-7);
            } else {
                // Шаг неудачен, увеличиваем lambda (переход к градиентному спуску)
                // 🔥 ВАЖНО: двигаемся даже если не улучшилось
                let grad = jacobian.transpose() * f_x;
                current_pos.draft_z -= grad[0] * 0.01;
                current_pos.heel_rx -= grad[1] * 0.01;
                current_pos.trim_ry -= grad[2] * 0.01;
                lambda = (lambda * 10.0).min(1e5);
            }
            current_pos.draft_z = current_pos.draft_z.clamp(-100.0, 100.0);
            current_pos.heel_rx = current_pos.heel_rx.clamp(-0.5, 0.5);
            current_pos.trim_ry = current_pos.trim_ry.clamp(-0.5, 0.5);
        } else {
            // Фолбэк на случай экстремальной числовой нестабильности
            lambda = lambda * 10.0;
            // let grad = jacobian.transpose() * f_x;
            // current_pos.draft_z -= grad[0].clamp(-1.0, 1.0) * 0.01;
            // current_pos.heel_rx -= grad[1] * 0.01;
            // current_pos.trim_ry -= grad[2] * 0.01;
            // current_pos.draft_z = current_pos.draft_z.clamp(-10.0, 10.0);
            // current_pos.heel_rx = current_pos.heel_rx.clamp(-0.5, 0.5);
            // current_pos.trim_ry = current_pos.trim_ry.clamp(-0.5, 0.5);
            // continue;
            // return Err("Singular Jacobian");
        }
    }
    Err("Diverged after max iterations")
}
fn calculate_normalized_jacobian(
    mesh: &TriMesh, 
    current_pos: FloatingPosition, 
    delta_z: f64,
    delta_angle: f64,
    target: &LoadingCondition
) -> Matrix3<f64> {
    let mut jacobian = Matrix3::zeros();

    // Вспомогательная функция для получения нормализованного вектора в смещенной точке
    let get_f = |pos: FloatingPosition| {
        let p = pos.to_plane();
        let s = p.slice_mesh(mesh);
        let h = s.hydrostatics(&p);
        make_f(&h, &target)
    };    // Колонка 0: Draft
    let mut pos_z_plus = current_pos;
    pos_z_plus.draft_z += delta_z;
    let mut pos_z_minus = current_pos;
    pos_z_minus.draft_z -= delta_z;
    let df_dz = (get_f(pos_z_plus) - get_f(pos_z_minus)) / (2.0 * delta_z);
    jacobian.set_column(0, &df_dz);
    // heel
    let mut pos_rx_plus = current_pos;
    pos_rx_plus.heel_rx += delta_angle;
    let mut pos_rx_minus = current_pos;
    pos_rx_minus.heel_rx -= delta_angle;
    let df_drx = (get_f(pos_rx_plus) - get_f(pos_rx_minus)) / (2.0 * delta_angle);
    jacobian.set_column(1, &df_drx);
    // trim
    let mut pos_ry_plus = current_pos;
    pos_ry_plus.trim_ry += delta_angle;
    let mut pos_ry_minus = current_pos;
    pos_ry_minus.trim_ry -= delta_angle;
    let df_dry = (get_f(pos_ry_plus) - get_f(pos_ry_minus)) / (2.0 * delta_angle);
    jacobian.set_column(2, &df_dry);
    jacobian
}
///
/// 
fn make_f(h: &Hydrostatics, target: &LoadingCondition) -> Vector3<f64> {
    let v_scale = target.target_volume.max(1.0);
    let l_scale = target.target_volume.cbrt(); // характерный размер
    // let l_scale = 1.0; // характерный размер
    Vector3::new(
        (h.volume - target.target_volume) / v_scale,
        (h.center_of_buoyancy.x - target.target_lcg) / l_scale,
        (h.center_of_buoyancy.y - target.target_tcg) / l_scale,
    )
}
