use parry3d_f64::{math::Vec3, shape::TriMesh};

use crate::tools::{
    Slice,
    strength::{clip_axis, split_by_x},
};

/// Главный строитель шпаций
pub struct HullSlicer {
    mesh: TriMesh,
}
//
impl HullSlicer {
    //
    pub fn new(mesh: TriMesh) -> Self {
        Self { mesh }
    }
    /// Рассекает меш на шпации по заданным координатам X
    pub fn slice(&self, x_coords: &[f64]) -> Vec<Slice> {
        if x_coords.len() < 2 {
            return Vec::new();
        }

        let vertices = self.mesh.vertices();
        let indices = self.mesh.indices();

        // Исходные треугольники
        let mut current_polygons: Vec<Vec<Vec3>> = indices
            .iter()
            .map(|tri| {
                vec![
                    vertices[tri[0] as usize],
                    vertices[tri[1] as usize],
                    vertices[tri[2] as usize],
                ]
            })
            .collect();

        let mut stations = Vec::with_capacity(x_coords.len() - 1);

        for window in x_coords.windows(2) {
            let x_start = window[0];
            let x_end = window[1];

            let mut next_polygons = Vec::with_capacity(current_polygons.len());
            let mut station_polygons = Vec::with_capacity(current_polygons.len());

            for poly in current_polygons {
                let mut min_x = f64::MAX;
                let mut max_x = f64::MIN;

                for p in &poly {
                    min_x = min_x.min(p.x);
                    max_x = max_x.max(p.x);
                }

                // Если полигон целиком в текущей шпации
                if min_x >= x_start && max_x <= x_end {
                    station_polygons.push(poly);
                    continue;
                }

                // Если полигон целиком впереди
                if min_x >= x_end {
                    next_polygons.push(poly);
                    continue;
                }

                // Если полигон пересекает x_end
                // Режем: то что слева (в шпацию), то что справа (в следующий шаг)
                let (left_part, right_part) = split_by_x(poly, x_end);

                if !left_part.is_empty() {
                    station_polygons.push(left_part);
                }
                if !right_part.is_empty() {
                    next_polygons.push(right_part);
                }
            }

            stations.push(Slice {
                x_start,
                x_end,
                polygons: station_polygons,
            });

            current_polygons = next_polygons;
        }
        stations
    }
}
