use nalgebra::Point3;

///
/// Объем погруженной части (Водоизмещение) и Центр величины (Center of Buoyancy - LCB, TCB, VCB)
pub struct Hydrostatics {
    /// Объем погруженной части (Водоизмещение)
    pub volume: f64,
    /// Центр величины (Center of Buoyancy - LCB, TCB, VCB)
    pub center_of_buoyancy: Point3<f64>,
}
