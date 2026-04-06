use nalgebra::{Point3, Vector3};

use crate::{plane::Plane, sliced_mesh::SlicedMesh};

pub struct Hydrostatics {
    /// Объем погруженной части (Водоизмещение)
    pub volume: f64,
    /// Центр величины (Center of Buoyancy - LCB, TCB, VCB)
    pub center_of_buoyancy: Point3<f64>,
}

pub fn calculate_hydrostatics(sliced: &SlicedMesh, plane: &Plane) -> Hydrostatics {
    let mut total_volume = 0.0;
    let mut sum_centroid = Vector3::zeros();

    // Вспомогательная замыкание (closure) для добавления тетраэдра
    // Использует начало координат (0,0,0) как четвертую вершину
    let mut add_tetrahedron = |p1: &Point3<f64>, p2: &Point3<f64>, p3: &Point3<f64>| {
        // Вычисляем знаковый объем тетраэдра (смешанное произведение векторов)
        let v_i = p1.coords.dot(&p2.coords.cross(&p3.coords)) / 6.0;
        total_volume += v_i;
        
        // Центр масс самого тетраэдра
        let centroid_i = (p1.coords + p2.coords + p3.coords) / 4.0;
        
        // Накапливаем взвешенный центр масс
        sum_centroid += centroid_i * v_i;
    };

    // Шаг 1: Интегрируем родные треугольники корпуса (погруженную часть)
    for tri in &sliced.submerged_triangles {
        // Порядок обхода сохранен в slice_parry_mesh, нормали смотрят наружу
        add_tetrahedron(&tri[0], &tri[1], &tri[2]);
    }

    // Шаг 2: Закрываем "крышку" (плоскость ватерлинии)
    // Если есть хотя бы одно ребро сечения
    if let Some(first_edge) = sliced.waterline_edges.first() {
        // Берем любую произвольную точку на плоскости сечения как центр веера
        let p_ref = first_edge[0]; 
        
        for edge in &sliced.waterline_edges {
            let a = edge[0];
            let b = edge[1];
            
            // Нам нужно, чтобы нормаль крышки смотрела "вверх" (из воды),
            // то есть совпадала по направлению с нормалью секущей плоскости.
            // Проверяем направление нормали образованного треугольника:
            let cross = (a - p_ref).cross(&(b - p_ref));
            let is_pointing_out = cross.dot(&plane.normal) > 0.0;
            
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
        Point3::from(sum_centroid / total_volume)
    } else {
        // Защита от деления на ноль (судно полностью в воздухе)
        Point3::origin()
    };

    Hydrostatics {
        volume: total_volume,
        center_of_buoyancy,
    }
}