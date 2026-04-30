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
    pub midel_from_stern: f64,
    pub draught_min: f64,
    pub lbp: f64,
    pub silhouette: Vec<(f64, f64)>,
}

impl WindageProfile {
    pub fn new(
        mesh: &parry3d_f64::shape::TriMesh,
        midel_from_stern: f64,
        draught_min: f64,
        lbp: f64,
    ) -> Self {
        let vertices = mesh.vertices();
        let indices = mesh.indices();

        // 1. Строим карту связей: Индекс точки -> Соседние индексы
        // Мы берем только уникальные ребра из треугольников
        let mut adj: HashMap<u32, Vec<u32>> = HashMap::new();
        for tri in indices {
            for i in 0..3 {
                let v1 = tri[i];
                let v2 = tri[(i + 1) % 3];
                if v1 != v2 {
                    adj.entry(v1).or_default().push(v2);
                    adj.entry(v2).or_default().push(v1);
                }
            }
        }

        // 2. Ищем стартовую точку (самая левая по X)
        let start_v_idx = (0..vertices.len())
            .min_by(|&a, &b| vertices[a].x.partial_cmp(&vertices[b].x).unwrap())
            .expect("Mesh is empty") as u32;

        let mut silhouette = Vec::new();
        let mut curr_idx = start_v_idx;
        // Направление "взгляда" при старте (из-за левого края смотрим вправо)
        let mut last_angle = std::f64::consts::PI; 
        
        let mut visited = std::collections::HashSet::new();

        // 3. Обход только по существующим точкам (вершинам)
        for _ in 0..vertices.len() {
            let curr_v = vertices[curr_idx as usize];
            silhouette.push((curr_v.x, curr_v.z));
            visited.insert(curr_idx);

            let mut best_next = None;
            let mut min_diff = f64::MAX;
            let mut best_angle = 0.0;

            // Смотрим только соседей текущей точки по мешу
            if let Some(neighbors) = adj.get(&curr_idx) {
                for &next_idx in neighbors {
                    let next_v = vertices[next_idx as usize];
                    
                    // Считаем угол поворота
                    let target_angle = (next_v.z - curr_v.z).atan2(next_v.x - curr_v.x);
                    let mut diff = target_angle - last_angle;
                    while diff <= 0.0 { diff += 2.0 * std::f64::consts::PI; }

                    // Выбираем самое "внешнее" ребро (наименьший разворот влево)
                    if diff < min_diff {
                        min_diff = diff;
                        best_next = Some(next_idx);
                        best_angle = target_angle;
                    }
                }
            }

            if let Some(next) = best_next {
                if next == start_v_idx && silhouette.len() > 2 { break; }
                
                // Если зациклились или попали в тупик
                if visited.contains(&next) && next != start_v_idx { break; }
                
                curr_idx = next;
                last_angle = best_angle - std::f64::consts::PI;
            } else {
                break;
            }
        }

        Self {
            midel_from_stern,
            draught_min,
            lbp,
            silhouette ,
        }        
    }

    /// Расчет площади и центра площади парусности для минимальной осадки
    /// результат: (area_v, moment_av_x, moment_av_z, area_sub_shift_z)
    pub fn calculate_area(
        &self,
        draught: f64,
        trim_deg: f64,
    ) -> Result<(f64, f64, f64, f64), Error> {
        if self.silhouette.len() < 3 {
            return Ok((0.0, 0.0, 0.0, 0.0));
        }

        let trim_tan = trim_deg.to_radians().tan();

        // 1. Надводная часть (Парусность)
        // Отсекаем всё, что НИЖЕ ватерлинии (оставляем точки, где z >= wl)
        let air_poly = self.clip_silhouette_by_water(draught, trim_tan, true);
        let (av, cx, cz) = calculate_poly_props(&air_poly);

        // 2. Подводная часть боковой проекции
        // Отсекаем всё, что ВЫШЕ ватерлинии (оставляем точки, где z <= wl)
        let sub_poly = self.clip_silhouette_by_water(draught, trim_tan, false);
        let (area_sub, _, area_sub_shift_z) = calculate_poly_props(&sub_poly);

        if av < 1e-7 {
            return Ok((0.0, 0.0, 0.0, area_sub_shift_z));
        }

        Ok((av, cx, cz, area_sub_shift_z))
    }

        /// Аналитическое отсечение многоугольника наклонной линией (Sutherland-Hodgman)
    fn clip_silhouette_by_water(&self, draught: f64, trim_tan: f64, above: bool) -> Vec<(f64, f64)> {
        let mut result = Vec::new();
        for i in 0..self.silhouette.len() {
            let p1 = self.silhouette[i];
            let p2 = self.silhouette[(i + 1) % self.silhouette.len()];

            let wl1 = draught + (p1.0 - self.midel_from_stern) * trim_tan;
            let wl2 = draught + (p2.0 - self.midel_from_stern) * trim_tan;

            // Условие нахождения точки "внутри" области
            let in1 = if above { p1.1 >= wl1 } else { p1.1 <= wl1 };
            let in2 = if above { p2.1 >= wl2 } else { p2.1 <= wl2 };

            if in1 && in2 {
                result.push(p2);
            } else if in1 && !in2 {
                result.push(intersect_with_wl(p1, p2, draught, self.midel_from_stern, trim_tan));
            } else if !in1 && in2 {
                result.push(intersect_with_wl(p1, p2, draught, self.midel_from_stern, trim_tan));
                result.push(p2);
            }
        }
        result
    }

    /*  /// Расчет распределения площади парусности
    pub fn bounded_windage_area(
        &self,
        target_bounds: &Bounds,
        draught: f64,
    ) -> Result<Vec<f64>, Error> {
        let error = Error::new("WindageProfile", "bounded_windage_area");

        // Подготавливаем значения площади для каждой полоски профиля при текущей осадке
        let src_values: Vec<f64> = self
            .z_intervals
            .iter()
            .zip(self.x_bounds.iter())
            .map(|((z_min, z_max), bound)| {
                (z_max - z_min.max(draught)).max(0.) * bound.length().unwrap_or(0.0)
            })
            .collect();
        // Используем встроенный метод Bounds для пересчета в целевую сетку
        target_bounds
            .intersect(&self.x_bounds, &src_values)
            .map_err(|e| error.pass_with("bounds.intersect", e))
    }*/

    /// Расчет площади и центра площади парусности
    pub fn windage_area(&self, draught: f64, trim: f64) -> Result<AreaResult, Error> {
        //      let error = Error::new("Bounds", "intersect");
        let (av_cs_dmin, mv_x_cs_dmin, mv_z_cs_dmin, _) =
            self.calculate_area(self.draught_min, trim)?;
        let (av_current, mv_x_current, mv_z_current, area_volume_z) =
            self.calculate_area(draught, trim)?;

        Ok(AreaResult {
            av_cs_dmin,
            mv_x_cs_dmin,
            mv_z_cs_dmin,
            delta_av: av_current - av_cs_dmin,
            delta_mv_x: mv_x_current - mv_x_cs_dmin,
            delta_mv_z: mv_z_current - mv_z_cs_dmin,
            area_volume_z,
        })
    }
  /*   /// Расчет площади проекции по правилу дополнительного запаса плавучести в носу
   pub fn bow_area(&self, draught: f64, trim_deg: f64) -> Result<f64, Error> {
        let trim_tan = trim_deg.to_radians().tan();

        // Определяем границы носовой части (последние 15% от LBP)
        // Предполагаем, что 0 — это кормовой перпендикуляр в системе координат меша.
        // Если начало координат другое, добавьте смещение к 0.85 * lbp и lbp.
        let bow_start = self.lbp * 0.85;
        let bow_end = self.lbp;

        let mut total_bow_area = 0.0;

        for (i, x_bound) in self.x_bounds.iter().enumerate() {
            let x_center = x_bound
                .center()
                .ok_or(Error::new("WindageProfile", "bow_area: x_center error"))?;
            let dx = x_bound
                .length()
                .ok_or(Error::new("WindageProfile", "bow_area: dx error"))?;

            // 1. Находим, какая часть текущей полоски попадает в "носовую зону"
            // (Используем метод пересечения отрезков для ratio)
            let x_low = x_center - dx * 0.5;
            let x_high = x_center + dx * 0.5;

            let overlap_min = x_low.max(bow_start);
            let overlap_max = x_high.min(bow_end);

            if overlap_max <= overlap_min {
                continue; // Полоска целиком вне носовой зоны
            }

            let ratio = (overlap_max - overlap_min) / dx;

            // 2. Расчет локальной ватерлинии
            let arm = x_center - self.midel_from_stern;
            let local_wl = draught + arm * trim_tan;

            // 3. Суммируем площади всех надводных интервалов в этой полоске
            let mut stripe_air_area = 0.0;
            for &(z_min, z_max) in &self.z_intervals[i] {
                let air_top = z_max;
                let air_bot = z_min.max(local_wl);

                if air_top > air_bot {
                    stripe_air_area += (air_top - air_bot) * dx;
                }
            }

            total_bow_area += stripe_air_area * ratio;
        }

        Ok(total_bow_area)
    }*/
}

fn intersect_segments(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64), p3: (f64, f64)) -> Option<(f64, f64)> {
    let dx1 = p1.0 - p0.0; let dy1 = p1.1 - p0.1;
    let dx2 = p3.0 - p2.0; let dy2 = p3.1 - p2.1;
    let det = dx1 * dy2 - dy1 * dx2;
    if det.abs() < 1e-12 { return None; }
    let t = ((p2.0 - p0.0) * dy2 - (p2.1 - p0.1) * dx2) / det;
    let u = ((p2.0 - p0.0) * dy1 - (p2.1 - p0.1) * dx1) / det;
    if (1e-6..=0.9999).contains(&t) && (0.0..=1.0).contains(&u) {
        Some((p0.0 + t * dx1, p0.1 + t * dy1))
    } else { None }
}

fn find_closest_index(xz: (f64, f64), vertices: &[Vec3]) -> u32 {
    (0..vertices.len())
        .min_by(|&a, &b| {
            let d1 = (vertices[a].x - xz.0).powi(2) + (vertices[a].z - xz.1).powi(2);
            let d2 = (vertices[b].x - xz.0).powi(2) + (vertices[b].z - xz.1).powi(2);
            d1.partial_cmp(&d2).unwrap()
        }).unwrap() as u32
}

// --- Чистая геометрия ---
/// Аналитическое нахождение точки пересечения ребра с наклонной ватерлинией
fn intersect_with_wl(p1: (f64, f64), p2: (f64, f64), d: f64, m: f64, tan: f64) -> (f64, f64) {
    let (x1, z1) = p1;
    let (x2, z2) = p2;
    
    // Обработка вертикального ребра
    if (x2 - x1).abs() < 1e-11 {
        return (x1, d + (x1 - m) * tan);
    }

    let slope_e = (z2 - z1) / (x2 - x1);
    
    // Если наклон ребра совпадает с наклоном воды (параллельны)
    if (tan - slope_e).abs() < 1e-11 {
        return p1; 
    }

    // Решаем уравнение: z1 + (x - x1) * slope_e = d + (x - m) * tan
    let x_int = (z1 - d + m * tan - x1 * slope_e) / (tan - slope_e);
    let z_int = d + (x_int - m) * tan;
    
    (x_int, z_int)
}

/// Расчет геометрических характеристик многоугольника
fn calculate_poly_props(poly: &[(f64, f64)]) -> (f64, f64, f64) {
    if poly.len() < 3 {
        return (0.0, 0.0, 0.0);
    }

    let mut area = 0.0;
    let mut mx = 0.0;
    let mut mz = 0.0;

    for i in 0..poly.len() {
        let p1 = poly[i];
        let p2 = poly[(i + 1) % poly.len()];
        
        let weight = p1.0 * p2.1 - p2.0 * p1.1; 
        area += weight;
        mx += (p1.0 + p2.0) * weight;
        mz += (p1.1 + p2.1) * weight;
    }

    let final_area = (area * 0.5).abs();
    if final_area < 1e-8 {
        return (0.0, 0.0, 0.0);
    }

    // mx и mz накоплены как 2 * момент, поэтому делим на 3 * area_accumulated (что равно 6 * Area)
    (final_area, mx / (3.0 * area), mz / (3.0 * area))
}

fn angle_diff(dir: (f64, f64), next: (f64, f64)) -> f64 {
    let a1 = dir.1.atan2(dir.0);
    let a2 = next.1.atan2(next.0);
    let mut d = a1 - a2; // Разница углов для выбора "самого левого"
    while d <= 0.0 { d += 2.0 * std::f64::consts::PI; }
    d
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

fn dot_product(a: (f64, f64), b: (f64, f64)) -> f64 { 
    a.0 * b.0 + a.1 * b.1 
}

fn cross_product(a: (f64, f64), b: (f64, f64)) -> f64 { 
    a.0 * b.1 - a.1 * b.0 
}
