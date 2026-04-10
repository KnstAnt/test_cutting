use parry3d_f64::math::Vec3;

use crate::tools::{Hydrostatics, Plane};

type DVec3 = parry3d_f64::glamx::DVec3;

///
/// Результат сечения солида плоскостью
pub struct SlicedMesh {
    /// Треугольники, оказавшиеся под плоскостью (полезный объем)
    pub submerged_triangles: Vec<[DVec3; 3]>,
    /// Отрезки, формирующие контур сечения (ватерлинию)
    pub waterline_edges: Vec<[Vec3; 2]>,
}
impl SlicedMesh {
    ///
    /// Для замкнутого меша объем считается как сумма ориентированных объемов тетраэдров.
    /// Требует передачи плоскости `Plane` для закрытия "крышки" разреза.
    pub fn volume(&self, plane: &Plane) -> f64 {
        let mut total_volume = 0.0;
        let mut add_tetrahedron = |p1: &DVec3, p2: &DVec3, p3: &DVec3| {
            total_volume += p1.dot(p2.cross(*p3)) / 6.0;
        };
        // 1. Интегрируем погруженные треугольники корпуса
        for [p1, p2, p3] in &self.submerged_triangles {
            add_tetrahedron(p1, p2, p3);
        }
        // 2. ЗАКРЫВАЕМ "КРЫШКУ" (Без этого объем будет неверным!)
        if let Some(first_edge) = self.waterline_edges.first() {
            let p_ref = first_edge[0]; // Центральная точка для веера
            for [a, b] in &self.waterline_edges {
                let cross = (a - p_ref).cross(b - p_ref);
                let is_pointing_out = cross.dot(plane.normal) > 0.0;
                // Сохраняем правильное направление нормали (из воды)
                if is_pointing_out {
                    add_tetrahedron(&p_ref, a, b);
                } else {
                    add_tetrahedron(&p_ref, b, a);
                }
            }
        }
        total_volume.abs()
    }
    ///
    /// Вычисление физических характеристик (объема, центра величины/масс, моментов инерции) традиционно делают через интегрирование по объему,
    /// но для полигональных сеток это решается в разы элегантнее — через интегрирование по поверхности.
    /// В основе лежит Теорема о дивергенции (Формула Остроградского-Гаусса)
    pub fn hydrostatics(&self, plane: &Plane) -> Hydrostatics {
        let mut total_volume = 0.0;
        let mut sum_centroid = DVec3::ZERO;
        // Используем origin плоскости как локальный ноль.
        // Это гарантирует, что даже если судно в 1000м от начала координат,
        // расчеты внутри будут оперировать малыми числами.
        let p_ref_global = plane.normal * plane.d;
        // Вспомогательная замыкание (closure) для добавления тетраэдра
        let mut add_tetrahedron = |p1: &DVec3, p2: &DVec3, p3: &DVec3| {
            let v1 = p1 - p_ref_global;
            let v2 = p2 - p_ref_global;
            let v3 = p3 - p_ref_global;
            // Знаковый объем тетраэдра
            let v_i = v1.dot(v2.cross(v3)) / 6.0;
            total_volume += v_i;
            // Центроид тетраэдра (в локальных координатах)
            let centroid_i = (v1 + v2 + v3) / 4.0;
            sum_centroid += centroid_i * v_i;
        };
        // Шаг 1: Интегрируем родные треугольники корпуса (погруженную часть)
        for tri in &self.submerged_triangles {
            // Порядок обхода сохранен в slice_parry_mesh, нормали смотрят наружу
            add_tetrahedron(&tri[0], &tri[1], &tri[2]);
        }
        // Шаг 2: Закрываем "крышку" (плоскость ватерлинии)
        // Если есть хотя бы одно ребро сечения
        if let Some(first_edge) = self.waterline_edges.first() {
            // Берем любую произвольную точку на плоскости сечения как центр веера
            let p_fan = first_edge[0]; 
            for edge in &self.waterline_edges {
                let a = edge[0];
                let b = edge[1];
                // Проверка нормали: крышка должна смотреть ПРОТИВ погружения (вверх)
                // plane.normal обычно смотрит вверх. Проверяем ориентацию треугольника [p_fan, a, b]
                let cross = (a - p_fan).cross(b - p_fan);
                // В зависимости от направления векторов, передаем вершины 
                // так, чтобы соблюдался правильный порядок обхода
                if cross.dot(plane.normal) > 0.0 {
                    add_tetrahedron(&p_fan, &a, &b);
                } else {
                    add_tetrahedron(&p_fan, &b, &a);
                }
            }
        }
        // Итоговый объем (берем abs только в конце, чтобы видеть ошибки намотки)
        let final_volume = total_volume.abs();
        // Итоговый центр величины - это сумма взвешенных центроидов, деленная на общий объем
        let center_of_buoyancy = if final_volume > 1e-9 {
            // Возвращаем центр из локальных координат в глобальные
            p_ref_global + (sum_centroid / total_volume)
        } else {
            DVec3::ZERO
        };
        Hydrostatics {
            volume: final_volume,
            center_of_buoyancy,
        }
    }
    ///
    /// Расчет площади ватерлинии
    pub fn waterline_area(&self) -> f64 {
        // Сумма площадей треугольников "крышки" через векторное произведение
        // Или просто площадь 2D проекции контура
        self.waterline_edges.iter()
            .map(|[a, b]| a.x * b.y - b.x * a.y)
            .sum::<f64>().abs() * 0.5
    }
}
