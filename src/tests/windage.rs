use crate::tools::*;
use parry3d_f64::math::Vec3;
use sal_core::dbg::Dbg;
use std::path::Path;


#[test]
fn windage_interval_sofia() {
    let scale = 0.001f64;
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let target = [
        (5., 1236.268, 60.460, 10.873),
        (6., 1105.868, 59.704, 11.507),
        (7., 973.025, 59.112, 12.190),
        (8., 839.143, 58.369, 12.939),
        (9., 704.458, 57.277, 13.787),
    ];

    let midel_dx = 65.25;
    let lbp = 130.5;
    let draught_min = 2.001;
    let windage = WindageProfile::new(&mesh, midel_dx, draught_min, lbp, 10000);
    for (draught, target_area, target_sx, target_sz) in &target {
        let (res_area, mx, mz, _) = windage.calculate_area(*draught, 0.);
        let res_sx = mx/res_area;
        let res_sz = mz/res_area; 
        println!(
            "draught:{:.3} area:({:.3} {:.3} {:.3}): sx:({:.3} {:.3} {:.3}) sz:({:.3} {:.3} {:.3})",
            draught, 
            target_area, res_area, target_area - res_area, 
            target_sx, res_sx, target_sx - res_sx,
            target_sz, res_sz, target_sz - res_sz,
        );
    }
}

/*
#[test]
fn windage_triangles_sofia() {
    let scale = 0.001f64;
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let target = [
        (5., 1236.268, 60.460, 10.873),
        (6., 1105.868, 59.704, 11.507),
        (7., 973.025, 59.112, 12.190),
        (8., 839.143, 58.369, 12.939),
        (9., 704.458, 57.277, 13.787),
    ];

    let midel_dx = 65.25;
    let windage = windage_triangles::WindageProfile::new(&mesh, midel_dx);
    for (draught, target_area, target_sx, target_sz) in &target {
        let (res_area, mx, mz, _) = windage.calculate_area(*draught, 0.);
        let res_sx = mx/res_area;
        let res_sz = mz/res_area; 
        println!(
            "draught:{:.3} area:({:.3} {:.3} {:.3}): sx:({:.3} {:.3} {:.3}) sz:({:.3} {:.3} {:.3})",
            draught, 
            target_area, res_area, target_area - res_area, 
            target_sx, res_sx, target_sx - res_sx,
            target_sz, res_sz, target_sz - res_sz,
        );
    }
}

#[test]
fn windage_voxels_sofia() {
    let scale = 0.001f64;
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let target = [
        (5., 1236.268, 60.460, 10.873),
        (6., 1105.868, 59.704, 11.507),
        (7., 973.025, 59.112, 12.190),
        (8., 839.143, 58.369, 12.939),
        (9., 704.458, 57.277, 13.787),
    ];
    let lbp = 130.5;
    let draught_min = 2.001;
    let midel_from_stern = 68.82;
    let windage = windage_voxels::WindageProfile::new(&mesh, midel_from_stern, draught_min, lbp, 10000);
    for (draught, target_area, target_sx, target_sz) in &target {
        let (res_area, mx, mz, _) = windage.calculate_area(*draught, 0.).unwrap();
        let res_sx = mx/res_area;
        let res_sz = mz/res_area; 
        println!(
            "draught:{:.3} area:({:.3} {:.3} {:.3}): sx:({:.3} {:.3} {:.3}) sz:({:.3} {:.3} {:.3})",
            draught, 
            target_area, res_area, target_area - res_area, 
            target_sx, res_sx, target_sx - res_sx,
            target_sz, res_sz, target_sz - res_sz,
        );
    }
}

#[test]
fn windage_edge_sofia() {
    let scale = 0.001f64;
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let target = [
        (5., 1236.268, 60.460, 10.873),
        (6., 1105.868, 59.704, 11.507),
        (7., 973.025, 59.112, 12.190),
        (8., 839.143, 58.369, 12.939),
        (9., 704.458, 57.277, 13.787),
    ];
    let lbp = 130.5;
    let draught_min = 2.001;
    let midel_from_stern = 68.82;
    let windage = windage_edge::WindageProfile::new(&mesh, midel_from_stern, draught_min, lbp);
    for (draught, target_area, target_sx, target_sz) in &target {
        let (res_area, mx, mz, _) = windage.calculate_area(*draught, 0.).unwrap();
        let res_sx = mx/res_area;
        let res_sz = mz/res_area; 
        println!(
            "draught:{:.3} area:({:.3} {:.3} {:.3}): sx:({:.3} {:.3} {:.3}) sz:({:.3} {:.3} {:.3})",
            draught, 
            target_area, res_area, target_area - res_area, 
            target_sx, res_sx, target_sx - res_sx,
            target_sz, res_sz, target_sz - res_sz,
        );
    }
}
*/

#[test]
fn windage_sofia2() {
    let dbg = Dbg::new("test", "windage_sofia2");
    let scale = 0.001f64;
    //   let path = "assets/Sofiya_4work.stl";
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));

    let midel_dx = 65.25;
    let lbp = 130.5;
    let draught_min = 2.001;
    let windage = WindageProfile::new(&mesh, midel_dx, draught_min, lbp, 10000);
    let draught_steps: Vec<_> = (2..=18).map(|v| (v as f64) * 0.5).collect();
    for &draught in &draught_steps {
        let (area, mx, mz, az) = windage.calculate_area(draught, 0.);
        let bow = windage.bow_area(draught, 0.).unwrap();
        println!(
            "{:.3}: az:{:.3} area:{:.3} sx:{:.3} mx:{:.3} sz:{:.3} mz:{:.3} bow:{:.3}",
            draught,
            az,
            area,
            mx / area,
            mx,
            mz / area,
            mz,
            bow
        );
    }
}
