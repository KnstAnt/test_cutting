use parry3d_f64::math::Vec3;

use crate::tools::{Hydrostatics, Plane};

pub struct SlicedMesh {
    /// Треугольники, оказавшиеся под плоскостью (полезный объем)
    pub submerged_triangles: Vec<[Vec3; 3]>,
    /// Отрезки, формирующие контур сечения (ватерлинию)
    pub waterline_edges: Vec<[Vec3; 2]>,
}
impl SlicedMesh {
    ///
    /// Для замкнутого меша объем считается как сумма ориентированных объемов тетраэдров.
    /// Требует передачи плоскости `Plane` для закрытия "крышки" разреза.
    pub fn volume(&self, plane: &Plane) -> f64 {
        let mut total_volume = 0.0;
        let mut add_tetrahedron = |p1: &Vec3, p2: &Vec3, p3: &Vec3| {
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
        let mut sum_centroid = Vec3::ZERO;
        // Вспомогательная замыкание (closure) для добавления тетраэдра
        // Использует начало координат (0,0,0) как четвертую вершину
        let mut add_tetrahedron = |p1: &Vec3, p2: &Vec3, p3: &Vec3| {
            // Вычисляем знаковый объем тетраэдра (смешанное произведение векторов)
            let v_i = p1.dot(p2.cross(*p3)) / 6.0;
            total_volume += v_i;
            // Центр масс самого тетраэдра
            let centroid_i = (p1 + p2 + p3) / 4.0;
            // Накапливаем взвешенный центр масс
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
            let p_ref = first_edge[0]; 
            for edge in &self.waterline_edges {
                let a = edge[0];
                let b = edge[1];
                // Нам нужно, чтобы нормаль крышки смотрела "вверх" (из воды),
                // то есть совпадала по направлению с нормалью секущей плоскости.
                // Проверяем направление нормали образованного треугольника:
                let cross = (a - p_ref).cross(b - p_ref);
                let is_pointing_out = cross.dot(plane.normal) > 0.0;
                // В зависимости от направления векторов, передаем вершины 
                // так, чтобы соблюдался правильный порядок обхода
                if is_pointing_out {
                    add_tetrahedron(&p_ref, &a, &b);
                } else {
                    add_tetrahedron(&p_ref, &b, &a);
                }
            }
        }
        // Итоговый центр величины - это сумма взвешенных центроидов, деленная на общий объем
        let center_of_buoyancy = if total_volume.abs() > 1e-6 {
            Vec3::from(sum_centroid / total_volume)
        } else {
            // Защита от деления на ноль (судно полностью в воздухе)
            Vec3::ZERO
        };
        Hydrostatics {
            volume: total_volume.abs(),
            center_of_buoyancy,
        }
    }
}
