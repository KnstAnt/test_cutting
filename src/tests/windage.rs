use crate::tools::*;
use parry3d_f64::math::Vec3;
use sal_core::dbg::Dbg;
use std::path::Path;

/*
#[test]
fn windage_sofia1() {
    let dbg = Dbg::new("test", "windage_sofia1");
    let scale = 0.001f64;
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let dx = 65.25;
    let mut cache = DisplacementCache::new(&dbg, "assets/displacement_cache_hull".into());
    dbg!(cache.init().unwrap());
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let dx = 65.25;
    let heel_steps = vec![
        -20., 0., 20.,
        //        0.,
    ];
    let trim_steps = vec![
        -20., 0., 20.,
        //         0.,
    ];
    let draught_steps: Vec<_> = vec![4.];
    for &heel in &heel_steps {
        for &trim in &trim_steps {
            for &draught in &draught_steps {
                let (ix, iy) = calculate_inertia(&mesh, dx, heel, trim, draught);
                println!(
                    "{:.3} {:.3} {:.3}: ix:{:.3} iy:{:.3}",
                    heel, trim, draught, ix, iy,
                );
            }
        }
    }
}
*/
#[test]
fn windage_sofia2() {
    let dbg = Dbg::new("test", "windage_sofia2");
    let scale = 0.001f64;
    let path = "assets/Sofiya_4work.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let dx = 65.25;
    let draught_min = 2.001;
    let lbp = 130.5;

    let windage = WindageProfile::new(&mesh, dx, draught_min, lbp, 10000);
    let draught_steps: Vec<_> = (2..=18).map(|v| (v as f64) * 0.5).collect();
        for &draught in &draught_steps {
            let (area, mx, mz, az) = windage._windage_area(draught, 0.).unwrap();
            let bow = windage.bow_area(draught, 0.).unwrap();
            println!(
                "{:.3}: az:{:.3} area:{:.3} sx:{:.3} mx:{:.3} sz:{:.3} mz:{:.3} bow:{:.3}",
                draught, az, area, mx/area, mx, mz/area, mz, bow
            );
        }
}
