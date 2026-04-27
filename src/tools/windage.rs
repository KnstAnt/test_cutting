use parry3d_f64::shape::TriMesh;
use sal_core::error::Error;

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
    /// Смещение миделя по Х относительно начала координат
    pub dx: f64,
    /// Описание сетки по X (от кормы до носа)
    pub x_bounds: Bounds,
    /// Высоты (min_z, max_z) для каждой части x_bounds
    pub z_bounds: Vec<(f64, f64)>,
}

impl WindageProfile {
    pub fn new(mesh: &parry3d_f64::shape::TriMesh, dx: f64, resolution: u32) -> Self {
        let aabb = mesh.local_aabb();
        let (x_min, x_max) = (aabb.mins.x, aabb.maxs.x);
        
        // Создаем Bounds по X. Он берет на себя хранение x_min, x_max и расчет dx
        let x_bounds = Bounds::from_min_max(x_min, x_max, resolution as usize)
            .expect("WindageProfile: Failed to create x_bounds");
        
        let res = resolution as usize;
        let scale_dx = (x_max - x_min) / res as f64;
        let mut z_data = vec![(f64::MAX, f64::MIN); res];
        
        let vertices = mesh.vertices();
        for tri in mesh.indices() {
            let pts = [vertices[tri[0] as usize], vertices[tri[1] as usize], vertices[tri[2] as usize]];
            let t_x = (pts[0].x.min(pts[1].x).min(pts[2].x), pts[0].x.max(pts[1].x).max(pts[2].x));
            let t_z = (pts[0].z.min(pts[1].z).min(pts[2].z), pts[0].z.max(pts[1].z).max(pts[2].z));

            let s_idx = (((t_x.0 - x_min) / scale_dx).floor() as usize).min(res - 1);
            let e_idx = (((t_x.1 - x_min) / scale_dx).ceil() as usize).min(res - 1);

            for i in s_idx..=e_idx {
                z_data[i].0 = z_data[i].0.min(t_z.0);
                z_data[i].1 = z_data[i].1.max(t_z.1);
            }
        }

        Self { dx, x_bounds, z_bounds: z_data }
    }
    /// Расчет площади и центра площади парусности для минимальной осадки
    pub fn windage_area_min(&self) -> Result<(f64, f64, f64), Error> {
        // Минимальная осадка — это самая низкая точка z_min во всем профиле
        let min_z = self.z_bounds.iter().map(|(z1, _)| *z1).fold(f64::MAX, f64::min);
        let res = self.windage_area(min_z, 0.)?;
        Ok((res.area, res.z_center, res.moment_x))
    }

    /// Расчет распределения площади парусности
    pub fn bounded_windage_area(&self, target_bounds: &Bounds, draught: f64) -> Result<Vec<f64>, Error> {
        let error = Error::new("WindageProfile", "bounded_windage_area");

        // Подготавливаем значения площади для каждой полоски профиля при текущей осадке
        let src_values: Vec<f64> = self.z_bounds.iter().zip(self.x_bounds.iter())
            .map(|((z_min, z_max), bound)| 
                (z_max - z_min.max(draught)).max(0.) * bound.length().unwrap_or(0.0)
            )
            .collect();
        // Используем встроенный метод Bounds для пересчета в целевую сетку
        target_bounds.intersect(&self.x_bounds, &src_values)
            .map_err(|e| error.pass_with("bounds.intersect", e))
    }

    /// Расчет площади и центра площади парусности
    /// Возвращает (площадь_парусности, Z_центра_парусности, момент_X, Z_центра_подводной_части_проекции)
    pub fn windage_area(&self, draught_mid: f64, trim: f64) -> Result<AreaResult, Error> {
        let mut w_area = 0.0;
        let mut w_mz = 0.0;
        let mut w_mx = 0.0;
        let mut sub_area = 0.0;
        let mut sub_mz = 0.0;

        let tan_trim = trim.to_radians().tan();

        for (i, x_bound) in self.x_bounds.iter().enumerate() {
            let x = x_bound.center().unwrap_or(0.0);
            let dx = x_bound.length().unwrap_or(0.0);
            let (z_min, z_max) = self.z_bounds[i];

            // Локальная осадка в данной точке X
            let local_draught = draught_mid.to_radians() + (x - self.dx) * tan_trim;

            // Надводная часть
            let w_bottom = z_min.max(local_draught);
            if z_max > w_bottom {
                let a = (z_max - w_bottom) * dx;
                w_area += a;
                w_mx += x * a;
                w_mz += ((z_max + w_bottom) * 0.5) * a;
            }

            // Подводная часть (проекция)
            let s_top = z_max.min(local_draught);
            if s_top > z_min {
                let a = (s_top - z_min) * dx;
                sub_area += a;
                sub_mz += ((s_top + z_min) * 0.5) * a;
            }
        }

        if w_area < 1e-9 { return Ok(AreaResult::zero()); }

        Ok(AreaResult {
            area: w_area,
            z_center: w_mz / w_area,
            moment_x: w_mx,
            sub_z_center: if sub_area > 1e-9 { sub_mz / sub_area } else { 0.0 },
        })
    }

    /// Расчет площади проекции по правилу дополнительного запаса плавучести в носу
    pub fn bow_area(&self, trim: f64, draught: f64, lbp: f64, x_ref: f64) -> Result<f64, Error> {
        let error = Error::new("WindageProfile", "bow_area");
        let bow_filter = Bound::new(lbp * 0.85, lbp).map_err(|e| error.pass_with("bow_bound", e))?;
        let tan_trim = trim.tan();
        let mut total_bow_area = 0.0;

        for (i, x_bound) in self.x_bounds.iter().enumerate() {
            let ratio = x_bound.part_ratio(&bow_filter).unwrap_or(0.0);
            if ratio <= 0.0 { continue; }

            let x = x_bound.center().unwrap_or(0.0);
            let local_draught = draught + (x - x_ref) * tan_trim;
            let (z_min, z_max) = self.z_bounds[i];

            let top = z_max;
            let bottom = z_min.max(local_draught);

            if top > bottom {
                total_bow_area += (top - bottom) * x_bound.length().unwrap_or(0.0) * ratio;
            }
        }
        Ok(total_bow_area)
    }
}
/// Расчет площади проекции по правилу дополнительного запаса плавучести в носу
/// [https://github.com/a-givertzman/sss/blob/master/design/algorithm/part03_draft/chapter02_draftCriteria/section04_bowBuoyancy.md]
/// Возвращает набор вокселей для кэша обрезанных по длине по длине в корме 0.15LBP от носового перпендикуляра
/// и в нос носовым перпендикуляром
fn get_bow_area_values(
    profile: &WindageProfile,
    lbp: f64,
    target_bounds: &Bounds,
    draught: f64,
) -> Result<Vec<f64>, Error> {
    let error = Error::new("Windage", "get_bow_area");
    let bow_filter = Bound::new(lbp * 0.85, lbp).map_err(|e| error.pass_with("bow_bound", e))?;

    // Считаем площади полосок, обрезанные и по осадке, и по продольному фильтру носа
    let src_values: Vec<f64> = profile.x_bounds.iter().enumerate()
        .map(|(i, &x)| {
            let (z_min, z_max) = profile.z_bounds[i];
            let strip_bound = Bound::new(x - profile.scale_dx / 2.0, x + profile.scale_dx / 2.0).unwrap();
            
            // Насколько полоска попадает в зону [0.85LBP - 1.0LBP]
            let ratio = strip_bound.part_ratio(&bow_filter).unwrap_or(0.0);
            
            let top = z_max;
            let bottom = z_min.max(draught);
            if top > bottom && ratio > 0.0 {
                (top - bottom) * profile.scale_dx * ratio
            } else {
                0.0
            }
        })
        .collect();

    let src_bounds = profile.as_bounds()?;
    target_bounds.intersect(&src_bounds, &src_values)
}
/// Пересчет в распределение суммарных площадей по х
fn get_bounds_area(
    profile: &WindageProfile,
    target_bounds: &Bounds,
    draught: f64,
) -> Result<Vec<f64>, Error> {
    let error = Error::new("Windage", "get_bounds_area");

    // 1. Получаем площади полосок с учетом текущей осадки
    let src_values: Vec<f64> = profile.z_bounds.iter()
        .map(|&(z_min, z_max)| {
            let top = z_max;
            let bottom = z_min.max(draught);
            if top > bottom { (top - bottom) * profile.scale_dx } else { 0.0 }
        })
        .collect();

    // 2. Пересчитываем в целевую сетку через intersect
    let src_bounds = profile.as_bounds().map_err(|e| error.pass_with("profile.as_bounds", e))?;
    target_bounds.intersect(&src_bounds, &src_values)
}
