use std::ops::Div;

use crate::tools::{Hydrostatics, Plane};
use parry3d_f64::math::Vec3;

pub struct SlicedMesh {
    /// Треугольники, оказавшиеся под плоскостью (полезный объем)
    pub submerged_triangles: Vec<[Vec3; 3]>,
    /// Отрезки, формирующие контур сечения (ватерлинию)
    pub waterline_edges: Vec<[Vec3; 2]>,
}

impl SlicedMesh {
    ///
    /// Для замкнутого меша объем считается как сумма ориентированных объемов тетраэдров.
    pub fn volume(&self) -> f64 {
        if self.submerged_triangles.is_empty() {
            return 0.0;
        }

        // 1. Выбираем точку опоры (p_ref).
        // Если есть ватерлиния, берем точку на ней.
        // Если нет (меш полностью погружен и замкнут), берем любую вершину меша.
        let p_ref = self
            .waterline_edges
            .first()
            .map(|edge| edge[0])
            .unwrap_or_else(|| self.submerged_triangles[0][0]);

        let mut total_volume = 0.0;

        // 2. Считаем сумму знаковых объемов тетраэдров относительно p_ref
        for [p0, p1, p2] in &self.submerged_triangles {
            let a = *p0 - p_ref;
            let b = *p1 - p_ref;
            let c = *p2 - p_ref;

            // Смешанное произведение (triple product)
            total_volume += a.dot(b.cross(c)) / 6.0;
        }

        total_volume.abs()
    }
    //
    pub fn hydrostatics_old(&self, plane: &Plane) -> Hydrostatics {
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
    //
    pub fn hydrostatics(&self) -> Hydrostatics {
        if self.submerged_triangles.is_empty() {
            return Hydrostatics {
                volume: 0.,
                center_of_buoyancy: Vec3::ZERO,
            };
        }

        let mut total_volume = 0.0;
        let mut sum_centroid = Vec3::ZERO;

        // 1. Находим точку на плоскости, которая будет вершиной всех тетраэдров.
        // Это гарантирует, что "крышка" (ватерлиния) имеет нулевой объем
        // и не влияет на итоговую сумму.
        // plane.d в вашей реализации — это normal.dot(point).
        let p_ref = self
            .waterline_edges
            .first()
            .map(|edge| edge[0])
            .unwrap_or_else(|| self.submerged_triangles.first().unwrap()[0]);

        // 2. Интегрируем только погруженные треугольники корпуса
        for tri in &self.submerged_triangles {
            let p0 = tri[0];
            let p1 = tri[1];
            let p2 = tri[2];

            // Векторы сторон тетраэдра относительно точки на плоскости воды
            let a = p0 - p_ref;
            let b = p1 - p_ref;
            let c = p2 - p_ref;

            // Знаковый объем тетраэдра (смешанное произведение)
            // 1/6 * |(a × b) · c|
            let v_i = a.dot(b.cross(c)) / 6.0;

            total_volume += v_i;

            // Центроид тетраэдра: (p0 + p1 + p2 + p_ref) / 4
            // Взвешиваем центроид объемом тетраэдра
            let centroid_i = (p0 + p1 + p2 + p_ref) * 0.25;
            sum_centroid += centroid_i * v_i;
        }

        // 3. Финальные расчеты.
        // Если нормаль плоскости смотрит "вверх", объем погруженной части
        // корпуса (нормали которого "наружу") будет отрицательным.
        // Это нормально, берем модуль.
        let abs_volume = total_volume.abs();

        let center_of_buoyancy = if abs_volume > f64::EPSILON {
            // Делим на знаковый объем, чтобы сохранить правильную ориентацию центра
            sum_centroid / total_volume
        } else {
            Vec3::ZERO
        };
        Hydrostatics {
            volume: abs_volume,
            center_of_buoyancy,
        }
    }
    //
    //

    pub fn calculate_waterline_properties(&self) -> (f64, Vec3) {
        let edges = &self.waterline_edges;

        if edges.is_empty() {
            return (0.0, Vec3::ZERO);
        }

        // 1. Сдвиг в локальный ноль для защиты от потери точности f64
        let mut sum_pts = Vec3::ZERO;
        for edge in edges {
            sum_pts += edge[0] + edge[1];
        }
        let origin = sum_pts / (edges.len() * 2) as f64;

        let mut signed_area_2x = 0.0;
        let mut moment_x = 0.0;
        let mut moment_y = 0.0;
        let mut z_sum = 0.0;

        // 2. Интегрируем только по компонентам X и Y
        for edge in edges {
            // Точки относительно origin
            let p1 = edge[0] - origin;
            let p2 = edge[1] - origin;

            // 2D детерминант в плоскости XY (векторное произведение проекций)
            let det = p1.x * p2.y - p2.x * p1.y;

            signed_area_2x += det;

            // Статические моменты проекции
            moment_x += (p1.x + p2.x) * det;
            moment_y += (p1.y + p2.y) * det;

            // Z просто усредняем для информации о положении плоскости
            z_sum += p1.z + p2.z;
        }

        // Площадь проекции (берем модуль в конце, чтобы учесть направление обхода)
        let area = signed_area_2x.abs() / 2.0;

        if area < 1e-10 {
            return (0.0, origin);
        }

        // 3. Вычисление центра тяжести проекции (центроида)
        // Формула: Cx = sum( (x1+x2) * det ) / (3 * sum(det))
        let cx = moment_x / (3.0 * signed_area_2x);
        let cy = moment_y / (3.0 * signed_area_2x);
        let cz = z_sum / (edges.len() * 2) as f64;

        let centroid = Vec3::new(origin.x + cx, origin.y + cy, origin.z + cz);

        (area, centroid)
    }
}
