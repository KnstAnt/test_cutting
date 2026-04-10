mod hull;
mod slice;

use std::path::Path;

pub use hull::*;
pub use slice::*;

use parry3d_f64::{math::Vec3, shape::TriMesh};

use crate::tools::load_stl;

pub fn calculate_strength(mesh: TriMesh, frames: &[f64], draughts: &[f64]) -> Vec<f64> {
    let hull = HullSlicer::new(mesh);
    let slices = hull.slice(frames);

    let displacements: Vec<_> = slices
        .iter()
        .map(|s| s.calculate_displacements(draughts))
        .collect();
    displacements.into_iter().map(|v| v.iter().sum()).collect()
}

/// Оптимизированный сплит: одна плоскость, один проход
fn split_by_x(poly: Vec<Vec3>, x_val: f64) -> (Vec<Vec3>, Vec<Vec3>) {
    let mut left = Vec::with_capacity(poly.len() + 1);
    let mut right = Vec::with_capacity(poly.len() + 1);

    for i in 0..poly.len() {
        let p1 = poly[i];
        let p2 = poly[(i + 1) % poly.len()];

        let v1_in = p1.x <= x_val;
        let v2_in = p2.x <= x_val;

        if v1_in {
            left.push(p1);
        } else {
            right.push(p1);
        }

        if v1_in != v2_in {
            let intersect_pt = intersect(p1, p2, 0, x_val);
            left.push(intersect_pt);
            right.push(intersect_pt);
        }
    }
    (left, right)
}

#[inline(always)]
fn clip_axis(
    input: &[Vec3],
    n_in: usize,
    output: &mut [Vec3],
    axis: usize,
    value: f64,
    keep_less: bool,
) -> usize {
    let mut n_out = 0;
    for i in 0..n_in {
        let p1 = input[i];
        let p2 = input[(i + 1) % n_in];
        let v1_val = if axis == 0 { p1.x } else { p1.z };
        let v2_val = if axis == 0 { p2.x } else { p2.z };

        let v1_inside = if keep_less {
            v1_val <= value
        } else {
            v1_val >= value
        };
        let v2_inside = if keep_less {
            v2_val <= value
        } else {
            v2_val >= value
        };

        if v1_inside {
            if !v2_inside {
                output[n_out] = intersect(p1, p2, axis, value);
                n_out += 1;
            } else {
                output[n_out] = p2;
                n_out += 1;
            }
        } else if v2_inside {
            output[n_out] = intersect(p1, p2, axis, value);
            n_out += 1;
            output[n_out] = p2;
            n_out += 1;
        }
    }
    n_out
}

#[inline(always)]
fn intersect(a: Vec3, b: Vec3, axis: usize, value: f64) -> Vec3 {
    let a_val = if axis == 0 { a.x } else { a.z };
    let b_val = if axis == 0 { b.x } else { b.z };
    let t = (value - a_val) / (b_val - a_val);
    // Используем clamp для t, чтобы избежать проблем с точностью float на гранях
    let t = t.clamp(0.0, 1.0);
    a + (b - a) * t
}
