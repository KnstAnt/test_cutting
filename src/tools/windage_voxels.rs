use std::collections::HashMap;

use parry3d_f64::{math::Vec3, shape::TriMesh};
use sal_core::{dbg::Dbg, error::Error};

use crate::tools::{Bound, Bounds};

/// площади и смещения расчета остойчивости
#[derive(Debug, Clone)]
pub struct AreaResult {
    /// Площадь парусности сплошных поверхностей для осадки d_min без палубного груза, м^2
    pub av_cs_dmin: f64,
    /// Cтатический момент площади парусности по длине относительно начала координат, м^2
    pub mv_x_cs_dmin: f64,
    /// Cтатический момент площади парусности по высоте относительно ОП, м^2
    pub mv_z_cs_dmin: f64,
    /// Разница в площадях парусности для текущей осадки и осадки d_min, м^2
    pub delta_av: f64,
    /// Разница в статических моментах для текущей осадки и осадки dmin относительно начала координат, м^3
    pub delta_mv_x: f64,
    /// Разница в статических моментах для текущей осадки и осадки dmin относительно ОП, м^3
    pub delta_mv_z: f64,
    /// Отстояние по вертикали центра площади проекции подводной части корпуса на диаметральную плоскость
    /// в прямом положении судна (при нулевом крене) на спокойной воде для текущей осадки [м]
    pub area_volume_z: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct WindageStrip {
    /// Координата X центра этой полоски (может быть смещена относительно центра судна)
    pub x: f64,
    /// Нижняя точка силуэта в этой полосе (обычно киль или линия палубы)
    pub z_min: f64,
    /// Верхняя точка силуэта в этой полосе (край надстроек или фальшборта)
    pub z_max: f64,
    /// Площадь полоски: (z_max - z_min) * dx
    pub area: f64,
}

pub struct WindageProfile {
    pub midel_dx: f64,
    pub draught_min: f64,
    pub lbp: f64,
    pub x_min: f64,
    pub z_min: f64,
    pub resolution: u32,
    pub step: f64,
    pub occupancy: Vec<Vec<bool>>,
}

impl WindageProfile {
    pub fn new(
        mesh: &parry3d_f64::shape::TriMesh,
        midel_from_stern: f64,
        draught_min: f64,
        lbp: f64,
        resolution: u32,
    ) -> Self {
        let aabb = mesh.local_aabb();
        let (x_min, x_max) = (aabb.mins.x, aabb.maxs.x);
        let (z_min, z_max) = (aabb.mins.z, aabb.maxs.z);

        // Вычисляем единый шаг для обеспечения квадратности ячеек
        let max_dim = (x_max - x_min).max(z_max - z_min);
        let step = max_dim / resolution as f64;
        
        let res_x = ((x_max - x_min) / step).ceil() as usize + 1;
        let res_z = ((z_max - z_min) / step).ceil() as usize + 1;
        
        let mut occupancy = vec![vec![false; res_z]; res_x];

        for tri in mesh.triangles() {
            let pts = [tri.a, tri.b, tri.c];
            
            // 1. Диапазон по X для треугольника
            let t_xmin = pts.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
            let t_xmax = pts.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
            
            let ix_start = (((t_xmin - x_min) / step).floor() as usize).min(res_x - 1);
            let ix_end = (((t_xmax - x_min) / step).floor() as usize).min(res_x - 1);

            for ix in ix_start..=ix_end {
                let x_curr = x_min + ix as f64 * step;
                let x_next = x_curr + step;

                // 2. Аналитическое нахождение Z-интервала треугольника в этой X-полосе
                if let Some((z_low, z_high)) = get_triangle_z_range_in_x_slice(&pts, x_curr, x_next) {
                    let iz_start = (((z_low - z_min) / step).floor() as usize).min(res_z - 1);
                    let iz_end = (((z_high - z_min) / step).floor() as usize).min(res_z - 1);

                    for iz in iz_start..=iz_end {
                        occupancy[ix][iz] = true;
                    }
                }
            }
        }


        Self {
            draught_min,
            midel_dx: x_min + midel_from_stern,
            lbp,
            x_min,
            z_min,
            step,
            resolution,
            occupancy,
        }
    }

    pub fn calculate_area(
        &self,
        draught: f64,
        trim_deg: f64,
    ) -> Result<(f64, f64, f64, f64), Error> {
                let trim_tan = trim_deg.to_radians().tan();
        let mut av = 0.0;
        let mut mx = 0.0;
        let mut mz = 0.0;
        let mut sub_area = 0.0;
        let mut sub_mz = 0.0;

        let cell_area = self.step * self.step;

        for (ix, col) in self.occupancy.iter().enumerate() {
            let x_c = self.x_min + (ix as f64 + 0.5) * self.step;
            let arm = x_c - self.midel_dx;
            let local_wl = draught + arm * trim_tan;

            for (iz, &is_occupied) in col.iter().enumerate() {
                if !is_occupied { continue; }

                let z_c = self.z_min + (iz as f64 + 0.5) * self.step;

                if z_c >= local_wl {
                    av += cell_area;
                    mx += x_c * cell_area;
                    mz += z_c * cell_area;
                } else {
                    sub_area += cell_area;
                    sub_mz += z_c * cell_area;
                }
            }
        }

        let cz_sub = if sub_area > 1e-7 { sub_mz / sub_area } else { 0.0 };
        if av < 1e-7 { return Ok((0.0, 0.0, 0.0, cz_sub)); }

        Ok((av, mx, mz, cz_sub))
    }
}

/// Точный расчет Z-границ треугольника внутри вертикальной полоски X
fn get_triangle_z_range_in_x_slice(pts: &[Vec3; 3], x1: f64, x2: f64) -> Option<(f64, f64)> {
    let mut z_min = f64::MAX;
    let mut z_max = f64::MIN;
    let mut found = false;

    for p in pts {
        if p.x >= x1 && p.x <= x2 {
            z_min = z_min.min(p.z);
            z_max = z_max.max(p.z);
            found = true;
        }
    }

    for i in 0..3 {
        let p_s = pts[i];
        let p_e = pts[(i + 1) % 3];
        for &x_b in &[x1, x2] {
            if (p_s.x <= x_b && p_e.x >= x_b) || (p_s.x >= x_b && p_e.x <= x_b) {
                let dx = p_e.x - p_s.x;
                if dx.abs() > 1e-11 {
                    let t = (x_b - p_s.x) / dx;
                    let z_int = p_s.z + t * (p_e.z - p_s.z);
                    z_min = z_min.min(z_int);
                    z_max = z_max.max(z_int);
                    found = true;
                }
            }
        }
    }
    if found { Some((z_min, z_max)) } else { None }
}