use parry3d_f64::{math::*, shape::TriMesh};
use crate::tools::SlicedMesh;

/// Секущая плоскость: (n, p) = d
pub struct Plane {
    pub normal: Vec3,
    pub d: f64, // Расстояние от начала координат вдоль нормали
}
//
impl Plane {
    /// Создает плоскость из нормали и точки на плоскости
    pub fn from_point_and_normal(point: Vec3, normal: Vec3) -> Self {
        let normal = normal.normalize();
        let d = normal.dot(point);
        // Считаем скалярное произведение вручную
        // let d = normal.x * point.x + normal.y * point.y + normal.z * point.z;
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
        let vertices  = mesh.vertices(); 
        let indices = mesh.indices();
        // Шаг 1: Предварительно вычисляем расстояния для всех вершин.
        // Это отлично векторизуется и предотвращает повторные расчеты для смежных треугольников.
        let distances: Vec<f64> = vertices.iter().map(|v| self.distance(&Vec3::new(v.x, v.y, v.z))).collect();
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
            let mut pts = [
                (v0, d0),
                (v1, d1),
                (v2, d2),
            ];
            // Считаем, сколько вершин находится "над" водой (положительное расстояние)
            let above_count = pts.iter().filter(|&&(_, d)| d > 0.0).count();
            match above_count {
                0 => {
                    // Весь треугольник под водой. Просто копируем.
                    submerged_triangles.push([Vec3::new(v0.x, v0.y, v0.z), Vec3::new(v1.x, v1.y, v1.z), Vec3::new(v2.x, v2.y, v2.z)]);
                }
                3 => {
                    // Весь треугольник над водой. Игнорируем.
                }
                1 => {
                    // 1 вершина над водой, 2 под водой.
                    // Треугольник обрезается, превращаясь в четырехугольник (2 новых треугольника под водой).
                    // Сдвигаем массив так, чтобы вершина "над водой" (d > 0) оказалась на нулевой позиции (pts[0])
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
                    submerged_triangles.push([Vec3::new(bottom1.x, bottom1.y, bottom1.z), Vec3::new(bottom2.x, bottom2.y, bottom2.z), Vec3::new(i1.x, i1.y, i1.z)]);
                    submerged_triangles.push([Vec3::new(bottom2.x, bottom2.y, bottom2.z), Vec3::new(i2.x, i2.y, i2.z), Vec3::new(i1.x, i1.y, i1.z)]);
                    // submerged_triangles.push([*bottom2, i2, i1]);
                    // Добавляем ребро сечения в контур
                    waterline_edges.push([i2, i1]);
                }
                2 => {
                    // 2 вершины над водой, 1 под водой.
                    // Треугольник обрезается, остаётся 1 маленький треугольник под водой.
                    // Сдвигаем массив так, чтобы вершина "под водой" (d <= 0) оказалась на нулевой позиции (pts[0])
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
                    submerged_triangles.push([Vec3::new(bottom.x, bottom.y, bottom.z), Vec3::new(i1.x, i1.y, i1.z), Vec3::new(i2.x, i2.y, i2.z)]);
                    // Добавляем ребро сечения в контур
                    waterline_edges.push([i1, i2]);
                }
                _ => unreachable!(),
            }
        }
        SlicedMesh {
            submerged_triangles, //: submerged_triangles.into_iter().map(|vv| vv.map(|v| Point3::new(v.x, v.y, v.z))).collect(),
            waterline_edges: waterline_edges.into_iter().map(|vv| vv.map(|v| Vec3::new(v.x, v.y, v.z))).collect(),
        }
    }
}
///
/// Математика пересечения ребра $V_a V_b$ с плоскостью использует линейную интерполяцию.
/// Если $d_a$ и $d_b$ — расстояния от вершин до плоскости, то точка пересечения $I$ вычисляется как:
/// ```ignore
/// $$I = V_a + \frac{d_a}{d_a - d_b} (V_b - V_a)$$
/// ```
#[inline(always)]
fn intersect_edge(a: &Vec3, b: &Vec3, d_a: f64, d_b: f64) -> Vec3 {
    // Доля пути от 'a' до 'b', где расстояние становится равным 0
    let t = d_a / (d_a - d_b);
    a + (b - a) * t
}
