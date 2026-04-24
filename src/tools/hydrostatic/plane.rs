use crate::tools::SlicedMesh;
use parry3d_f64::{math::*, shape::TriMesh};

#[derive(Clone)]
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
        let vertices = mesh.vertices();
        let indices = mesh.indices();

        // Предварительный расчет расстояний
        let distances: Vec<f64> = vertices
            .iter()
            .map(|v| self.distance(&Vec3::new(v.x, v.y, v.z)))
            .collect();

        let mut submerged_triangles = Vec::with_capacity(indices.len());
        let mut waterline_edges = Vec::new();

        for face in indices {
            let idx = [face[0] as usize, face[1] as usize, face[2] as usize];
            let d = [distances[idx[0]], distances[idx[1]], distances[idx[2]]];
            let v = [
                Vec3::new(vertices[idx[0]].x, vertices[idx[0]].y, vertices[idx[0]].z),
                Vec3::new(vertices[idx[1]].x, vertices[idx[1]].y, vertices[idx[1]].z),
                Vec3::new(vertices[idx[2]].x, vertices[idx[2]].y, vertices[idx[2]].z),
            ];

            let above_mask = [d[0] > 0.0, d[1] > 0.0, d[2] > 0.0];
            let above_count = above_mask.iter().filter(|&&a| a).count();

            match above_count {
                0 => {
                    // Полностью под водой
                    submerged_triangles.push([v[0], v[1], v[2]]);
                }
                3 => {
                    // Полностью над водой — игнорируем
                }
                1 => {
                    // Одна вершина над водой. Ищем её индекс.
                    let i0 = above_mask.iter().position(|&a| a).unwrap();
                    let i1 = (i0 + 1) % 3;
                    let i2 = (i0 + 2) % 3;

                    // Точки пересечения на ребрах, выходящих из i0
                    let p1 = intersect_edge(&v[i0], &v[i1], d[i0], d[i1]);
                    let p2 = intersect_edge(&v[i0], &v[i2], d[i0], d[i2]);

                    // Оставшаяся под водой часть — четырехугольник (v[i1], v[i2], p2, p1)
                    // Разбиваем на два треугольника, сохраняя порядок обхода (i1 -> i2 -> p2 -> p1)
                    submerged_triangles.push([v[i1], v[i2], p1]);
                    submerged_triangles.push([v[i2], p2, p1]);

                    // Ребро ватерлинии: от p1 к p2 (согласовано с обходом)
                    waterline_edges.push([p1, p2]);
                }
                2 => {
                    // Две вершины над водой, одна под. Ищем индекс той, что ПОД.
                    let i0 = above_mask.iter().position(|&a| !a).unwrap();
                    let i1 = (i0 + 1) % 3;
                    let i2 = (i0 + 2) % 3;

                    // Точки пересечения на ребрах, выходящих из i0
                    let p1 = intersect_edge(&v[i0], &v[i1], d[i0], d[i1]);
                    let p2 = intersect_edge(&v[i0], &v[i2], d[i0], d[i2]);

                    // Один треугольник под водой: (v[i0], p1, p2)
                    submerged_triangles.push([v[i0], p1, p2]);

                    // Ребро ватерлинии: от p2 к p1 (согласовано с обходом)
                    waterline_edges.push([p2, p1]);
                }
                _ => unreachable!(),
            }
        }

        SlicedMesh {
            submerged_triangles,
            waterline_edges,
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
    let denom = d_a - d_b;
    if denom.abs() < 1e-8 {
        return *a; // Разница слишком мала, точки практически в одном месте
    }
    let t = d_a / denom;
    // let t = d_a / (d_a - d_b);
    a + (b - a) * t
}
