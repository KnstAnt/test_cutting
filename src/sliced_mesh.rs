use nalgebra::Point3;
use parry3d_f64::shape::TriMesh;

use crate::{intersect_edge, plane::Plane};

pub struct SlicedMesh {
    /// Треугольники, оказавшиеся под плоскостью (полезный объем)
    pub submerged_triangles: Vec<[Point3<f64>; 3]>,
    /// Отрезки, формирующие контур сечения (ватерлинию)
    pub waterline_edges: Vec<[Point3<f64>; 2]>,
}

pub fn slice_parry_mesh(mesh: &TriMesh, plane: &Plane) -> SlicedMesh {
    let vertices = mesh.vertices();
    let indices = mesh.indices();

    // Шаг 1: Предварительно вычисляем расстояния для всех вершин.
    // Это отлично векторизуется и предотвращает повторные расчеты для смежных треугольников.
    let distances: Vec<f64> = vertices.iter().map(|v| plane.distance(v)).collect();

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
                submerged_triangles.push([*v0, *v1, *v2]);
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
                submerged_triangles.push([*bottom1, *bottom2, i1]);
                submerged_triangles.push([*bottom2, i2, i1]);

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
                submerged_triangles.push([*bottom, i1, i2]);

                // Добавляем ребро сечения в контур
                waterline_edges.push([i1, i2]);
            }
            _ => unreachable!(),
        }
    }

    SlicedMesh {
        submerged_triangles,
        waterline_edges,
    }
}