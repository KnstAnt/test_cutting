use parry3d_f64::math::Vec3;
use sal_core::error::Error;

#[derive(Debug, Clone)]
pub struct WindageColumn {
    /// Список неперекрывающихся интервалов (z_min, z_max) в этой X-полосе
    pub intervals: Vec<(f64, f64)>,
}

pub struct WindageProfile {
    pub x_min: f64,
    pub step: f64,
    pub midel_dx: f64,
    pub draught_min: f64,
    pub lbp: f64,
    pub columns: Vec<WindageColumn>,
}

impl WindageProfile {
    pub fn new(
        mesh: &parry3d_f64::shape::TriMesh,
        midel_dx: f64,
        draught_min: f64,
        lbp: f64,
        resolution: u32,
    ) -> Self {
        let aabb = mesh.local_aabb();
        let (x_min, x_max) = (aabb.mins.x, aabb.maxs.x);

        let step = (x_max - x_min) / resolution as f64;
        let res_x = resolution as usize;
        let inv_step = 1.0 / step;

        // Временное хранилище для всех интервалов каждой колонки
        let mut raw_columns: Vec<Vec<(f64, f64)>> = vec![Vec::new(); res_x];

        for tri in mesh.triangles() {
            let pts = [tri.a, tri.b, tri.c];

            // Фильтр нормали (только один борт)
            let v0 = pts[1] - pts[0];
            let v1 = pts[2] - pts[0];
            if v0.cross(v1).y <= 1e-9 {
                continue;
            }

            let t_xmin = pts.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
            let t_xmax = pts.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);

            let ix_start = (((t_xmin - x_min) * inv_step).max(0.0) as usize).min(res_x - 1);
            let ix_end = (((t_xmax - x_min) * inv_step).max(0.0) as usize).min(res_x - 1);

            for ix in ix_start..=ix_end {
                let x_curr = x_min + ix as f64 * step;
                let x_next = x_curr + step;

                if let Some((z_low, z_high)) = get_triangle_z_range_in_x_slice(&pts, x_curr, x_next)
                {
                    if z_high > z_low {
                        raw_columns[ix].push((z_low, z_high));
                    }
                }
            }
        }

        let columns = raw_columns
            .into_iter()
            .map(|mut list| {
                if list.is_empty() {
                    return WindageColumn {
                        intervals: Vec::new(),
                    };
                }

                list.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

                let mut merged = Vec::new();
                if let Some(first) = list.first() {
                    let mut current = *first;
                    for i in 1..list.len() {
                        let next = list[i];
                        if next.0 <= current.1 {
                            // Отрезки перекрываются, расширяем текущий
                            current.1 = current.1.max(next.1);
                        } else {
                            // Разрыв, сохраняем текущий и начинаем новый
                            merged.push(current);
                            current = next;
                        }
                    }
                    merged.push(current);
                }

                WindageColumn { intervals: merged }
            })
            .collect();

        Self {
            x_min,
            step,
            midel_dx,
            draught_min,
            lbp,
            columns,
        }
    }

    pub fn calculate_area(&self, draught: f64, trim_deg: f64) -> (f64, f64, f64, f64) {
        let trim_tan = trim_deg.to_radians().tan();
        let mut av = 0.0;
        let mut mx_sum = 0.0;
        let mut mz_sum = 0.0;
        let mut sub_area = 0.0;
        let mut sub_mz_sum = 0.0;

        for (ix, col) in self.columns.iter().enumerate() {
            if col.intervals.is_empty() {
                continue;
            }

            let x_c = self.x_min + (ix as f64 + 0.5) * self.step;
            let arm = x_c - self.midel_dx;
            let local_wl = draught + arm * trim_tan;

            for &(z_min, z_max) in &col.intervals {
                // 1. Надводная часть интервала
                let z_up_start = z_min.max(local_wl);
                let z_up_end = z_max;
                if z_up_end > z_up_start {
                    let h = z_up_end - z_up_start;
                    let area = h * self.step;
                    av += area;
                    mx_sum += x_c * area;
                    mz_sum += (z_up_start + z_up_end) * 0.5 * area;
                }

                // 2. Подводная часть интервала
                let z_sub_start = z_min;
                let z_sub_end = z_max.min(local_wl);
                if z_sub_end > z_sub_start {
                    let h = z_sub_end - z_sub_start;
                    let area = h * self.step;
                    sub_area += area;
                    sub_mz_sum += (z_sub_start + z_sub_end) * 0.5 * area;
                }
            }
        }

        let cz_sub = if sub_area > 1e-7 {
            sub_mz_sum / sub_area
        } else {
            0.0
        };

        (av, mx_sum, mz_sum, cz_sub)
    }

    /// Расчет площади проекции по правилу дополнительного запаса плавучести в носу
        pub fn bow_area(&self, draught: f64, trim_deg: f64) -> Result<f64, Error> {
        let trim_tan = trim_deg.to_radians().tan();

        // 1. Определяем физические границы судна по LBP.
        // Если мидель — это центр LBP, то:
        let stern_x = self.midel_dx - self.lbp / 2.0; // Корма
        let bow_x = self.midel_dx + self.lbp / 2.0;   // Нос
        
        // Граница 15% зоны от носового перпендикуляра в сторону кормы
        let bow_zone_start = bow_x - (self.lbp * 0.15);
        let bow_zone_end = bow_x;

        let mut total_bow_area = 0.0;

        for (ix, col) in self.columns.iter().enumerate() {
            if col.intervals.is_empty() {
                continue;
            }

            let x_low = self.x_min + ix as f64 * self.step;
            let x_high = x_low + self.step;

            // Пересечение текущей колонки с зоной 15% LBP в носу
            let overlap_min = x_low.max(bow_zone_start);
            let overlap_max = x_high.min(bow_zone_end);

            if overlap_max <= overlap_min {
                continue; 
            }

            let effective_dx = overlap_max - overlap_min;

            // Плечо для дифферента: x_center относительно миделя
            let x_center = x_low + 0.5 * self.step;
            let arm = x_center - self.midel_dx; 
            let local_wl = draught + arm * trim_tan;

            let mut stripe_air_area = 0.0;
            for &(z_min, z_max) in &col.intervals {
                let air_top = z_max;
                let air_bot = z_min.max(local_wl);

                if air_top > air_bot {
                    stripe_air_area += (air_top - air_bot) * effective_dx;
                }
            }

            total_bow_area += stripe_air_area;
        }

        Ok(total_bow_area)
    }
}

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
