use nalgebra::{Point3, Vector3};
use parry3d_f64::shape::TriMesh;

/// Секущая плоскость: (n, p) = d
pub struct Plane {
    pub normal: Vector3<f64>,
    pub d: f64, // Расстояние от начала координат вдоль нормали
}

impl Plane {
    /// Создает плоскость из нормали и точки на плоскости
    pub fn from_point_and_normal(point: Point3<f64>, normal: Vector3<f64>) -> Self {
        let normal = normal.normalize();
        let d = normal.dot(&point.coords);
        Self { normal, d }
    }

    /// Знаковое расстояние от точки до плоскости
    #[inline(always)]
    pub fn distance(&self, p: &Point3<f64>) -> f64 {
        self.normal.dot(&p.coords) - self.d
    }
}