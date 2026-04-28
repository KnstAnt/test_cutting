use parry3d_f64::shape::TriMesh;
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
    /// Смещение миделя по Х относительно начала координат
    pub dx: f64,
    /// Минимальная осадка
    pub draught_min: f64,
    /// Длинна между перпендикулярами
    pub lbp: f64,
    /// Описание сетки по X (от кормы до носа)
    pub x_bounds: Bounds,
    /// Высоты (min_z, max_z) для каждой части x_bounds
    pub z_bounds: Vec<(f64, f64)>,
}

impl WindageProfile {
    pub fn new(mesh: &parry3d_f64::shape::TriMesh, dx: f64, draught_min: f64, lbp: f64, resolution: u32) -> Self {
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

        Self { draught_min, dx, lbp, x_bounds, z_bounds: z_data }
    }
    /// Расчет площади и центра площади парусности для минимальной осадки
    /// результат: (area_v, moment_av_x, moment_av_z, area_sub_shift_z)
    pub fn _windage_area(&self, draught: f64, trim: f64) -> Result<(f64, f64, f64, f64), Error> {
    //    let error = Error::new(&self.dbg, "_windage_area");

        let trim = trim.to_radians().tan();  

        let mut av = 0.0;
        let mut mv_x = 0.0;
        let mut mv_z = 0.0;

        // Переменные для подводной части
        let mut sub_area = 0.0;
        let mut sub_mz = 0.0;

        for (i, x_bound) in self.x_bounds.iter().enumerate() {
            let x = x_bound.center().unwrap_or(0.0);
            let dx = x_bound.length().unwrap_or(0.0);
            let (z_min, z_max) = self.z_bounds[i];

            // Локальные осадки
            let local_draught = draught + (x - dx) * trim;

            // Расчет для текущей осадки
            let h_curr = (z_max - z_min.max(local_draught)).max(0.0);
            let a_curr = h_curr * dx;
            av += a_curr;
            mv_x += x * a_curr;
            mv_z += ((z_max + z_min.max(local_draught)) * 0.5) * a_curr;

            // Расчет подводной проекции (area_volume_z)
            let s_top = z_max.min(local_draught);
            if s_top > z_min {
                let a_sub = (s_top - z_min) * dx;
                sub_area += a_sub;
                sub_mz += ((s_top + z_min) * 0.5) * a_sub;
            }
        }

        let area_sub_shift_z = if sub_area > f64::MIN {sub_mz/sub_area} else { 0. };

        Ok((av, mv_x, mv_z, area_sub_shift_z))
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
    pub fn windage_area(&self, draught: f64, trim: f64) -> Result<AreaResult, Error> {
  //      let error = Error::new("Bounds", "intersect");
        let (av_cs_dmin, mv_x_cs_dmin, mv_z_cs_dmin, _) = self._windage_area(self.draught_min, trim)?;
        let (av_current, mv_x_current, mv_z_current, area_volume_z) = self._windage_area(draught, trim)?;

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
    /// Расчет площади проекции по правилу дополнительного запаса плавучести в носу
    pub fn bow_area(&self, draught: f64, trim: f64) -> Result<f64, Error> {
        let error = Error::new("WindageProfile", "bow_area");
        let bow_filter = Bound::new(self.lbp * 0.85, self.lbp).map_err(|e| error.pass_with("bow_bound", e))?;
        let trim = trim.to_radians().tan();
        let area: f64 = self.x_bounds.iter().zip(self.z_bounds.iter()).map(|(x, (z_min, z_max))| {
            let local_draught = draught + (x.center().unwrap_or(0.0) - self.dx) * trim;
            let local_area = (z_max - z_min.max(local_draught)).max(0.)*x.length().unwrap_or(0.0);
            x.part_ratio(&bow_filter).map(|ratio| local_area *ratio).unwrap_or(0.)
        }).sum();
        Ok(area)
    }
}
