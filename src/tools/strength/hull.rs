use parry3d_f64::{math::Vec3, shape::TriMesh};

use crate::tools::{
    Slice, strength::*,
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
        if x_coords.len() < 2 { return Vec::new(); }

        let n_stations = x_coords.len() - 1;
        
        let vertices = self.mesh.vertices();
        let indices = self.mesh.indices();
        
         // 1. Binning: Распределяем индексы треугольников по "корзинам" шпаций
        let mut grid: Vec<Vec<u32>> = vec![Vec::with_capacity(indices.len() / n_stations); n_stations];
        
        for (tri_idx, tri) in indices.iter().enumerate() {
            let p0 = vertices[tri[0] as usize];
            let p1 = vertices[tri[1] as usize];
            let p2 = vertices[tri[2] as usize];

            let t_min = p0.x.min(p1.x).min(p2.x);
            let t_max = p0.x.max(p1.x).max(p2.x);

            // Бинарный поиск для определения диапазона шпаций
            let start_idx = match x_coords.binary_search_by(|x| x.partial_cmp(&t_min).unwrap()) {
                Ok(idx) => idx,
                Err(idx) => idx.saturating_sub(1),
            }.min(n_stations - 1);

            let end_idx = match x_coords.binary_search_by(|x| x.partial_cmp(&t_max).unwrap()) {
                Ok(idx) => idx,
                Err(idx) => idx,
            }.min(n_stations - 1);

            for i in start_idx..=end_idx {
                grid[i].push(tri_idx as u32);
            }
        }

        // 2. Sequential Processing: Нарезка каждой шпации
        let mut stations = Vec::with_capacity(n_stations);
        let mut buf_a = [Vec3::ZERO; 16];
        let mut buf_b = [Vec3::ZERO; 16];

        for i in 0..n_stations {
            let x_s = x_coords[i];
            let x_e = x_coords[i + 1];
            let tri_indices = &grid[i];
            
            let mut st_points = Vec::with_capacity(tri_indices.len() * 3);
            let mut st_ranges = Vec::with_capacity(tri_indices.len());

            for &idx in tri_indices {
                let tri = indices[idx as usize];
                let tri_verts = [vertices[tri[0] as usize], vertices[tri[1] as usize], vertices[tri[2] as usize]];

                // Нарезаем треугольник границами X
                let n1 = clip_axis_to_buffer(&tri_verts, &mut buf_b, 0, x_s, false);
                if n1 < 3 { continue; }
                let n2 = clip_axis_to_buffer(&buf_b[..n1], &mut buf_a, 0, x_e, true);
                if n2 < 3 { continue; }

                let start = st_points.len();
                st_points.extend_from_slice(&buf_a[..n2]);
                st_ranges.push(start..st_points.len());
            }

            stations.push(Slice {
                x_start: x_s,
                x_end: x_e,
                points: st_points,
                poly_ranges: st_ranges,
            });
        }
        stations
    }
}
