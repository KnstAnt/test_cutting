use parry3d_f64::math::Vec3;

use crate::tools::strength::clip_axis;


/// Представляет собой геометрию корпуса в границах одной шпации
pub struct Slice {
    pub x_start: f64,
    pub x_end: f64,
    /// Набор отсеченных многоугольников (фрагментов треугольников), 
    /// которые попали в эту шпацию.
    pub polygons: Vec<Vec<Vec3>>, 
}
//
impl Slice {
    /// Внутренний метод: объем конкретного многоугольника ниже z_wl
    pub fn calculate_displacements(&self, z_levels: &[f64]) -> Vec<f64> {
        z_levels.iter().map(|&z_wl| {
            let mut total_vol = 0.0;
            let mut poly_out = [Vec3::ZERO; 12]; // Буфер для отсечения по Z

            for poly in &self.polygons {
                let n = clip_axis(poly, poly.len(), &mut poly_out, 2, z_wl, true);
                if n < 3 { continue; }

                let mut vol = 0.0;
                let base_p = poly_out[0];
                for i in 1..n - 1 {
                    let p1 = poly_out[i];
                    let p2 = poly_out[i + 1];
                    let area_xy = 0.5 * ((p1.x - base_p.x) * (p2.y - base_p.y) - (p2.x - base_p.x) * (p1.y - base_p.y));
                    let avg_depth = ((z_wl - base_p.z) + (z_wl - p1.z) + (z_wl - p2.z)) / 3.0;
                    vol += area_xy * avg_depth;
                }
                total_vol += vol;
            }
            total_vol.abs()
        }).collect()
    }
}
