mod hull;
mod slice;
pub use hull::*;
pub use slice::*;
use parry3d_f64::{math::Vec3, shape::TriMesh};

pub fn calculate_strength_full(mesh: TriMesh, frames: &[f64], draughts: &[f64]) -> Vec<(f64, f64)> {
    let displacements = calculate_strength_bounded(mesh, frames, draughts);
    let res: Vec<_> = draughts
        .into_iter()
        .enumerate()
        .map(|(i, z)| 
            (
                *z, 
                displacements.iter().map(|v| v[i].1).sum::<f64>()
            ))
        .collect();
    res
}

pub fn calculate_strength_bounded(mesh: TriMesh, frames: &[f64], draughts: &[f64]) -> Vec<Vec<(f64, f64)>> {
    let hull = HullSlicer::new(mesh);
    let slices = hull.slice(frames);
    slices
        .iter()
        .map(|s| s.calculate_displacements(draughts))
        .collect()
}

#[inline(always)]
fn clip_axis_to_buffer(
    input: &[Vec3],
    output: &mut [Vec3],
    axis: usize,
    val: f64,
    less: bool,
) -> usize {
    let mut n = 0;
    let len = input.len();
    for i in 0..len {
        let p1 = input[i];
        let p2 = input[(i + 1) % len];
        let (v1, v2) = if axis == 0 {
            (p1.x, p2.x)
        } else {
            (p1.z, p2.z)
        };
        let in1 = if less { v1 <= val } else { v1 >= val };
        let in2 = if less { v2 <= val } else { v2 >= val };

        if in1 {
            if in2 {
                output[n] = p2;
                n += 1;
            } else {
                output[n] = intersect(p1, p2, axis, val);
                n += 1;
            }
        } else if in2 {
            output[n] = intersect(p1, p2, axis, val);
            n += 1;
            output[n] = p2;
            n += 1;
        }
    }
    n
}

#[inline(always)]
fn intersect(a: Vec3, b: Vec3, axis: usize, val: f64) -> Vec3 {
    let (va, vb) = if axis == 0 { (a.x, b.x) } else { (a.z, b.z) };
    let t = ((val - va) / (vb - va)).clamp(0.0, 1.0);
    a + (b - a) * t
}
