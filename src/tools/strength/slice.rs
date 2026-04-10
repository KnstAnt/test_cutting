use parry3d_f64::math::Vec3;
use crate::tools::strength::*;
use std::ops::Range;
use rayon::prelude::*;
use rayon::slice::ParallelSlice;

/// Представляет собой геометрию корпуса в границах одной шпации
pub struct Slice {
    pub x_start: f64,
    pub x_end: f64,
    pub points: Vec<Vec3>,
    pub poly_ranges: Vec<Range<usize>>,
}
//
impl Slice {
    /// Внутренний метод: объем конкретного многоугольника ниже z_wl
    pub fn calculate_displacements(&self, z_levels: &[f64]) -> Vec<(f64, f64)> {
         if z_levels.is_empty() { return Vec::new(); }

        // 1. Считаем начальный объем для самой нижней точки (база)
        let mut current_v = self.calculate_single_volume(z_levels[0]);
        
        // 2. Параллельно вычисляем приращения (dV) для каждого слоя
        let layer_volumes: Vec<f64> = z_levels.par_windows(2).map(|w| {
            let z_low = w[0];
            let z_high = w[1];
            let dz = z_high - z_low;
            let mut d_vol = 0.0;
            
            let mut buf_in = [Vec3::ZERO; 16];
            let mut buf_above = [Vec3::ZERO; 16];

            for range in &self.poly_ranges {
                let poly = &self.points[range.start..range.end];

                // --- ЧАСТЬ 1: Фрагменты внутри слоя ---
                // Отсекаем всё, что выше z_high, а затем из результата берем то, что выше z_low
                let n_tmp = clip_axis_to_buffer(poly, &mut buf_above, 2, z_high, true);
                if n_tmp >= 3 {
                    let n_in = clip_axis_to_buffer(&buf_above[..n_tmp], &mut buf_in, 2, z_low, false);
                    if n_in >= 3 {
                        let p0 = buf_in[0];
                        for i in 1..n_in - 1 {
                            let p1 = buf_in[i];
                            let p2 = buf_in[i+1];
                            let area_xy = 0.5 * ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y));
                            // Высота считается от Z_low (честный "скос")
                            let depth = ((p0.z - z_low) + (p1.z - z_low) + (p2.z - z_low)) / 3.0;
                            d_vol += area_xy * depth;
                        }
                    }
                }

                // --- ЧАСТЬ 2: Фрагменты выше слоя (Транзит) ---
                // Берем части полигона, которые строго выше z_high
                let n_above = clip_axis_to_buffer(poly, &mut buf_above, 2, z_high, false);
                if n_above >= 3 {
                    let p0 = buf_above[0];
                    let mut area_above = 0.0;
                    for i in 1..n_above - 1 {
                        area_above += 0.5 * ((buf_above[i].x - p0.x) * (buf_above[i+1].y - p0.y) - (buf_above[i+1].x - p0.x) * (buf_above[i].y - p0.y));
                    }
                    // Весь этот "пояс" проходит через слой вертикально
                    d_vol += area_above * dz;
                }
            }
            d_vol
        }).collect();

        // 3. Сборка результатов
        let mut results = Vec::with_capacity(z_levels.len());
        results.push((z_levels[0], current_v));

        for (i, d_v) in layer_volumes.into_iter().enumerate() {
            current_v += d_v;
            results.push((z_levels[i + 1], current_v.max(0.0)));
        }

        results
    }
    /// Честный расчет объема для начальной точки
    fn calculate_single_volume(&self, z_wl: f64) -> f64 {
        let mut poly_buffer = [Vec3::ZERO; 16];
        let mut total_vol = 0.0;
        for range in &self.poly_ranges {
            let poly = &self.points[range.start..range.end];
            let n = clip_axis_to_buffer(poly, &mut poly_buffer, 2, z_wl, true);
            if n < 3 { continue; }
            let p0 = poly_buffer[0];
            for i in 1..n - 1 {
                let p1 = poly_buffer[i];
                let p2 = poly_buffer[i+1];
                let area_xy = 0.5 * ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y));
                let depth = ((z_wl - p0.z) + (z_wl - p1.z) + (z_wl - p2.z)) / 3.0;
                total_vol += area_xy * depth;
            }
        }
        total_vol.abs()
    }    
}
