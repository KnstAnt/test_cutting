use parry3d_f64::math::Vec3;
use sal_core::error::Error;

pub struct WindageProfile {
    pub triangles: Vec<[Vec3; 3]>,
    pub midel_dx: f64,
}

impl WindageProfile {
    pub fn new(mesh: &parry3d_f64::shape::TriMesh, midel_dx: f64) -> Self {
        let triangles: Vec<[Vec3; 3]> = mesh
            .triangles()
            .filter_map(|tri| {
                let pts = [tri.a, tri.b, tri.c];
                
                // Вычисляем нормаль: (B - A) x (C - A)
                let v0 = pts[1] - pts[0];
                let v1 = pts[2] - pts[0];
                let normal = v0.cross(v1);

                // Оставляем только те, что смотрят в сторону положительного Y
                // (один борт судна + внешние поверхности надстроек)
                if normal.y > 1e-9 {
                    Some(pts)
                } else {
                    None
                }
            })
            .collect();
        
        Self {
            triangles,
            midel_dx,
        }
    }
    pub fn calculate_area(&self, draught: f64, trim_deg: f64) -> (f64, f64, f64, f64) {
        let trim_tan = trim_deg.to_radians().tan();
        
        let mut av = 0.0;     // Площадь над водой
        let mut mx = 0.0;     // Момент X над водой
        let mut mz = 0.0;     // Момент Z над водой
        
        let mut sub_area = 0.0;
        let mut sub_mz = 0.0;

        for tri in &self.triangles {
            // Относительные координаты: Z=0 теперь ватерлиния с учетом дифферента
            let rel_pts: Vec<Vec3> = tri.iter().map(|p| {
                let local_wl = draught + (p.x - self.midel_dx) * trim_tan;
                Vec3::new(p.x, 0.0, p.z - local_wl)
            }).collect();

            // --- 1. РАСЧЕТ НАДВОДНОЙ ЧАСТИ (z_rel > 0) ---
            let poly_above = clip_triangle_by_z(&rel_pts, true);
            let (a_up, mx_up, mz_up) = calculate_poly_moments(&poly_above, draught, self.midel_dx, trim_tan);
            
            av += a_up;
            mx += mx_up;
            mz += mz_up;

            // --- 2. РАСЧЕТ ПОДВОДНОЙ ЧАСТИ (z_rel < 0) ---
            let poly_below = clip_triangle_by_z(&rel_pts, false);
            let (a_sub, _, mz_sub) = calculate_poly_moments(&poly_below, draught, self.midel_dx, trim_tan);
            sub_area += a_sub;
            sub_mz += mz_sub;
        }

        let cz_sub = if sub_area > 1e-7 { sub_mz / sub_area } else { 0.0 };

        (av, mx, mz, cz_sub)
    }
}

/// Универсальная функция отсечения: above=true (z > 0), false (z < 0)
fn clip_triangle_by_z(pts: &[Vec3], above: bool) -> Vec<Vec3> {
    let mut result = Vec::with_capacity(4);
    let sign = if above { 1.0 } else { -1.0 };

    for i in 0..3 {
        let p1 = pts[i];
        let p2 = pts[(i + 1) % 3];

        if (p1.z * sign) >= 0.0 {
            result.push(p1);
        }

        if (p1.z > 0.0 && p2.z < 0.0) || (p1.z < 0.0 && p2.z > 0.0) {
            let t = p1.z / (p1.z - p2.z);
            let intersect = p1 + (p2 - p1) * t;
            result.push(intersect);
        }
    }
    result
}

/// Расчет площади и моментов для произвольного выпуклого полигона (результата клиппинга)
fn calculate_poly_moments(poly: &[Vec3], draught: f64, midel: f64, tan: f64) -> (f64, f64, f64) {
    let mut area_sum = 0.0;
    let mut mz_sum = 0.0;
    let mut mx_sum = 0.0;

    if poly.len() < 3 { return (0.0, 0.0, 0.0); }

    let p0 = poly[0];
    for i in 1..(poly.len() - 1) {
        let p1 = poly[i];
        let p2 = poly[i + 1];

        // 2D Cross product для площади на плоскости XZ
        let area = 0.5 * ((p1.x - p0.x) * (p2.z - p0.z) - (p2.x - p0.x) * (p1.z - p0.z)).abs();
        
        if area > 1e-9 {
            let cx = (p0.x + p1.x + p2.x) / 3.0;
            let cz_rel = (p0.z + p1.z + p2.z) / 3.0;
            
            let local_wl = draught + (cx - midel) * tan;
            let cz_abs = cz_rel + local_wl;

            area_sum += area;
            mx_sum += cx * area;
            mz_sum += cz_abs * area;
        }
    }
    (area_sum, mx_sum, mz_sum)
}
