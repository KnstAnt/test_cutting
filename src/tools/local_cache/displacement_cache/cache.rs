use sal_core::{dbg::Dbg, error::Error};
use std::{
    path::{Path, PathBuf},
};
use crate::tools::{local_cache::cache::Cache, *};
///
/// Pre-calculated cache for floating position algorithm.
/// contains keys: [heel, trim, draught]
/// values:[volume, x, y, z, area, x, y, z, waterline_x, waterline_y]
pub struct DisplacementCache {
    dbg: Dbg,
    cache_path: PathBuf,
    /// Cache read from `self.file_path`.
    cache: Option<Cache<f64>>,
}
//
//
impl DisplacementCache {
    ///
    /// Creates a new instance.
    /// - cache_dir - folder contains all cache files
    pub fn new(
        parent: &Dbg,
        cache_path: PathBuf,
    ) -> Self {
        let dbg = Dbg::new(parent, "DisplacementCache");
        Self {
            cache: None,
            cache_path,
            dbg,
        }
    }
    /// Получение данных кэша для текущего положения
    /// Итерационно подбирает значение водоизмещения по осадке    
    pub fn get(
        &self,
        heel: f64,
        trim: f64,
        volume: f64,
        epsilon: f64,
    ) -> Result<DisplacementCacheResult, Error> {
        let error = Error::new(&self.dbg, "get");
        let cache = self.cache.as_ref().ok_or(error.pass("no cache"))?;
        let (draught, result) = get_from_volume(
            &self.dbg,
            cache,
            &[heel, trim],
            volume,
            None,
            None,
            3,
            epsilon,
        )
        .map_err(|err| error.pass(err))?;
        Ok(DisplacementCacheResult {
            heel,
            trim,
            draught,
            volume: result[0],
            volume_center: Position::new(result[1], result[2], result[3]),
            area_wl: result[4],
            area_wl_center: Position::new(result[5], result[6], result[7]),
            inertia_trans_x: result[8],
            inertia_long_y: result[9],
            length_wl: result[10],
            breadth_wl: result[11],
        })
    }
    //
    pub fn get_volume_disp(&self) -> Result<(f64, f64), Error> {
        let error = Error::new(&self.dbg, "get_max_volume");
        let cache = self.cache.as_ref().ok_or(error.pass("no cache"))?;
        Ok(cache.disp(3))
    }
    /// Получение данных кэша для текущего положения
    pub fn get_from_level(
        &self,
        heel: f64,
        trim: f64,
        level: f64,
    ) -> DisplacementCacheResult {
        let cache = self.cache.as_ref().unwrap();
        let result = get_from_level(
            &self.dbg,
            cache,
            &[heel, trim],
            level,
            None,
            3,
        ).unwrap();
        DisplacementCacheResult {
            heel,
            trim,
            draught: level,
            volume: result[0],
            volume_center: Position::new(result[1], result[2], result[3]),
            area_wl: result[4],
            area_wl_center: Position::new(result[5], result[6], result[7]),
            inertia_trans_x: result[8],
            inertia_long_y: result[9],
            length_wl: result[10],
            breadth_wl: result[11],
        }
    }    
    /// Получение данных кэша для текущего положения
    /// Итерационно подбирает значение водоизмещения по осадке    
    pub fn get_from_volume(
        &self,
        heel: f64,
        trim: f64,
        volume: f64,
        epsilon: f64,
    ) -> DisplacementCacheResult {
        let cache = self.cache.as_ref().unwrap();
        let (draught, result) = get_from_volume(
            &self.dbg,
            cache,
            &[heel, trim],
            volume,
            None,
            None,
            3,
            epsilon,
        ).unwrap();
        DisplacementCacheResult {
            heel,
            trim,
            draught,
            volume: result[0],
            volume_center: Position::new(result[1], result[2], result[3]),
            area_wl: result[4],
            area_wl_center: Position::new(result[5], result[6], result[7]),
            inertia_trans_x: result[8],
            inertia_long_y: result[9],
            length_wl: result[10],
            breadth_wl: result[11],
        }
    }    
}
//
impl LocalCache for DisplacementCache {
    //
    fn cache_path(&self) -> PathBuf {
        self.cache_path.clone()
    }
    //
    fn cache(&self) -> Option<&Cache<f64>> {
        self.cache.as_ref()
    }

    fn set_cache(&mut self, cache: Cache<f64>) {
        let _ = self.cache.insert(cache);
    }
    
    fn dbg(&self) -> &Dbg {
        &self.dbg
    }
}
