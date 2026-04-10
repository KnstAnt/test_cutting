use parry3d_f64::{math::*, shape::TriMesh};
use crate::tools::SlicedMesh;

type DVec3 = parry3d_f64::glamx::DVec3;
const EPS: f64 = 1e-6;

/// Секущая плоскость: (n, p) = d
pub struct Plane {
    pub normal: DVec3,
    pub d: f64, // Расстояние от начала координат вдоль нормали
}
impl Plane {
    /// Создает плоскость из нормали и точки на плоскости
    pub fn from_point_and_normal(point: Vec3, normal: Vec3) -> Self {
        let normal = normal.normalize();
        let d = normal.dot(point);
        Self { normal, d }
    }
    /// Знаковое расстояние от точки до плоскости
    #[inline(always)]
    pub fn distance(&self, point: &Vec3) -> f64 {
        self.normal.dot(*point) - self.d
    }
    ///
    /// Функция, которая принимает TriMesh из parry и сечет ее плоскостью. 
    /// Мы будем сохранять только те части треугольников, которые находятся «под» плоскостью (d≤0),
    /// что типично для задачи расчета погруженного объема судна.
    /// Также мы соберем отрезки, лежащие на самой секущей плоскости (периметр сечения).
    pub fn slice_mesh(&self, mesh: &TriMesh) -> SlicedMesh {
        let indices = mesh.indices();
        let vertices  = mesh.vertices();
        // Шаг 1: Предварительно вычисляем расстояния для всех вершин.
        // Это отлично векторизуется и предотвращает повторные расчеты для смежных треугольников.
        // Весь float-шум около нуля жестко прибиваем к 0.0
        let distances: Vec<f64> = vertices.iter().map(|point| self.distance(point)).collect();
        let mut submerged_triangles = Vec::with_capacity(indices.len() / 2);
        let mut waterline_edges = Vec::new();
        // Шаг 2: Проходим по всем треугольникам
        for face in indices {
            let v0 = &vertices[face[0] as usize];
            let v1 = &vertices[face[1] as usize];
            let v2 = &vertices[face[2] as usize];
            let d0 = distances[face[0] as usize];
            let d1 = distances[face[1] as usize];
            let d2 = distances[face[2] as usize];
            // Собираем вершины и их расстояния в массив для удобной сортировки
            // СТРОГИЙ SNAPPING (убираем float-шум около нуля)
            let mut pts = [
                (v0, if d0.abs() < EPS { 0.0 } else { d0 }),
                (v1, if d1.abs() < EPS { 0.0 } else { d1 }),
                (v2, if d2.abs() < EPS { 0.0 } else { d2 }),
            ];
            // Считаем, сколько вершин находится "над" водой (положительное расстояние)
            // СТРОГАЯ БИНАРНАЯ КЛАССИФИКАЦИЯ (Без нулей!)
            // Если строго больше 0 — над водой. Всё остальное (включая 0) — под водой.
            let above_count = pts.iter().filter(|&&(_, d)| d > 0.0).count();
            match above_count {
                0 => {
                    // Весь треугольник под водой. Просто копируем.
                    submerged_triangles.push([*v0, *v1, *v2]);
                }
                3 => {
                    // Весь треугольник над водой. Игнорируем.
                }
                1 => {
                    // 1 точка над водой. Вращаем, пока pts[0] не станет этой точкой (> 0.0)
                    // Треугольник обрезается, превращаясь в четырехугольник (2 новых треугольника под водой).
                    while pts[0].1 <= 0.0 {
                        pts.rotate_left(1);
                    }
                    let top = pts[0].0;
                    let bottom1 = pts[1].0;
                    let bottom2 = pts[2].0;
                    // Находим две точки пересечения на ребрах (top -> bottom1) и (top -> bottom2)
                    let i1 = intersect_edge(top, bottom1, pts[0].1, pts[1].1);
                    let i2 = intersect_edge(top, bottom2, pts[0].1, pts[2].1);
                    // Добавляем два новых треугольника, сохраняя порядок обхода (winding order)
                    submerged_triangles.push([*bottom1, *bottom2, i1]);
                    submerged_triangles.push([*bottom2, i2, i1]);
                    // Добавляем ребро сечения в контур
                    // FIX 4 & 5: Фильтр длины и правильная ориентация контура
                    if (i1 - i2).length() > EPS {
                        waterline_edges.push([i2, i1]); 
                    }
                }
                2 => {
                    // 2 точки над водой. Вращаем, пока pts[0] не станет точкой ПОД водой (<= 0.0)
                    // Треугольник обрезается, остаётся 1 маленький треугольник под водой.
                    while pts[0].1 > 0.0 {
                        pts.rotate_left(1);
                    }
                    let bottom = pts[0].0;
                    let top1 = pts[1].0;
                    let top2 = pts[2].0;
                    // Находим две точки пересечения
                    let i1 = intersect_edge(bottom, top1, pts[0].1, pts[1].1);
                    let i2 = intersect_edge(bottom, top2, pts[0].1, pts[2].1);
                    // Добавляем новый маленький треугольник
                    submerged_triangles.push([*bottom, i1, i2]);
                    // Добавляем ребро сечения в контур
                    // FIX 4 & 5: Фильтр длины и правильная ориентация контура
                    if (i1 - i2).length() > EPS {
                        waterline_edges.push([i1, i2]);
                    }
                }
                _ => unreachable!(),
            }
        }
        SlicedMesh {
            submerged_triangles,
            waterline_edges,
        }
    }
//    ///
//    /// Подписанное расстояние до плоскости
//    #[inline]
//    pub fn signed_distance(&self, p: &Point3<f64>) -> f64 {
//        self.normal.dot(&p.coords) - self.d
//    }
}
///
/// Безопасный intersect
/// Математика пересечения ребра $V_a V_b$ с плоскостью использует линейную интерполяцию.
/// Если $d_a$ и $d_b$ — расстояния от вершин до плоскости, то точка пересечения $I$ вычисляется как:
/// ```ignore
/// $$I = V_a + \frac{d_a}{d_a - d_b} (V_b - V_a)$$
/// ```
#[inline(always)]
fn intersect_edge(a: &DVec3, b: &DVec3, d_a: f64, d_b: f64) -> DVec3 {
    let denom = d_a - d_b;
    if denom.abs() < 1e-12 {
        return (*a + *b) * 0.5; // лучше midpoint
    }
    let t = d_a / denom;
    a + (b - a) * t
}
/// 
/// epsilon классификация
fn classify(d: f64) -> i32 {
    if d > EPS { 1 }
    else if d < -EPS { -1 }
    else { 0 }
}
///
/// 
fn sort_edge(a: Vec3, b: Vec3) -> [Vec3; 2] {
    if a.x < b.x { [a, b] } else { [b, a] }
}