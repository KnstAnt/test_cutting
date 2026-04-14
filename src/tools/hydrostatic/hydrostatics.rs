use parry3d_f64::math::Vec3;


///
/// Объем погруженной части (Водоизмещение) и Центр величины (Center of Buoyancy - LCB, TCB, VCB)
pub struct Hydrostatics {
    /// Объем погруженной части (Водоизмещение)
    pub volume: f64,
    /// Центр величины (Center of Buoyancy - LCB, TCB, VCB)
    pub center_of_buoyancy: Vec3,
}
