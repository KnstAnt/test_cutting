use parry3d_f64::math::Vec3;
use sal_core::dbg::Dbg;
use std::path::Path;

use crate::tools::{
    DisplacementCache, LocalCache, calculate_hydrostatic, calculate_hydrostatic_old, load_stl, *,
};

#[test]
fn hydrostatic_sofia1() {
    let dbg = Dbg::new("test", "hydrostatic_sofia");
    let scale = 0.001f64;
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let dx = 65.25;
    let mut cache = DisplacementCache::new(&dbg, "assets/displacement_cache_hull".into());
    dbg!(cache.init().unwrap());
    /*  let heel_steps = vec![
        -60., -50., -45., -40., -35., -30., -25., -20., -15., -10., -5., -2., -1., -0.5, -0.2, 0.,
        0.2, 0.5, 1., 2., 5., 10., 15., 20., 25., 30., 35., 40., 45., 50., 60.,
    ];
    let trim_steps = vec![
        -40., -30., -25., -20., -15., -12.5, -10., -7.5, -5., -3., -2., -1., -0.5, -0.2, 0., 0.2,
        0.5, 1., 2., 3., 5., 7.5, 10., 12.5, 20., 25., 30., 40.,
    ];
    let draught_steps: Vec<_> = (1..=28).map(|v| (v as f64) * 0.5).collect();*/
    let heel_steps = vec![-60., -20., -5., 0., 5., 20., 60.];
    let trim_steps = vec![-40., -10., -2., 0., 2., 10., 40.];
    let draught_steps: Vec<_> = (1..=7).map(|v| (v as f64) * 2.).collect();
    let epsilon_volume_abs = 10.;
    let epsilon_volume_percent = 0.1;
    let epsilon_center_abs = 0.01;
    let epsilon_center_percent = 0.1;
    let mut results = Vec::new();
    for &heel in &heel_steps {
        for &trim in &trim_steps {
            for &draught in &draught_steps {
                let (result_volume, result_center) =
                    calculate_hydrostatic(&mesh, dx, heel, trim, draught);
                let target = cache.get_from_level(heel, trim, draught);
                let check = |text: String,
                             result: f64,
                             target: f64,
                             epsilon_abs: f64,
                             epsilon_percent: f64|
                 -> Option<(f64, f64, String, f64, f64)> {
                    let delta_abs = (result - target).abs();
                    let delta_percent = if target > 0. {
                        delta_abs * 100. / target
                    } else {
                        0.
                    };
                    /*    println!(
                        "{text} result:{result} target:{target} delta_abs:{delta_abs} delta_percent:{delta_percent}"
                    );*/
                    if delta_abs > epsilon_abs && delta_percent > epsilon_percent {
                        return Some((delta_abs, delta_percent, text, result, target));
                    }
                    None
                };
                check(
                    format!("{:.3} {:.3} {:.1} volume", heel, trim, draught),
                    result_volume,
                    target.volume,
                    epsilon_volume_abs,
                    epsilon_volume_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.x", heel, trim, draught),
                    result_center.x,
                    target.volume_center.x(),
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.y", heel, trim, draught),
                    result_center.y,
                    target.volume_center.y(),
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.z", heel, trim, draught),
                    result_center.z,
                    target.volume_center.z(),
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
            }
        }
    }
    results.sort_by(|a, b| (a.1).partial_cmp(&b.1).unwrap());
    //  let _ = results.split_off(100);
    for v in results {
        println!(
            "{} error: result:{} target:{} delta_abs:{} delta_percent:{}",
            v.2, v.3, v.4, v.0, v.1
        );
    }
}

#[test]
fn hydrostatic_sofia2() {
    let dbg = Dbg::new("test", "hydrostatic_sofia");
    let scale = 0.001f64;
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let dx = 65.25;
    let mut cache = DisplacementCache::new(&dbg, "assets/displacement_cache_hull".into());
    dbg!(cache.init().unwrap());
    let heel_steps = vec![-60., -20., -5., 0., 5., 20., 60.];
    let trim_steps = vec![-40., -10., -2., 0., 2., 10., 40.];
    let draught_steps: Vec<_> = (1..=7).map(|v| (v as f64) * 2.).collect();
    let epsilon_volume_abs = 10.;
    let epsilon_volume_percent = 0.1;
    let epsilon_center_abs = 0.01;
    let epsilon_center_percent = 0.1;
    let mut results = Vec::new();
    for &heel in &heel_steps {
        for &trim in &trim_steps {
            for &draught in &draught_steps {
                let (result_volume, result_center) =
                    calculate_hydrostatic(&mesh, dx, heel, trim, draught);
                let (target_volume, target_center) =
                    calculate_hydrostatic_old(mesh.clone(), dx, heel, trim, draught);
                let check = |text: String,
                             result: f64,
                             target: f64,
                             epsilon_abs: f64,
                             epsilon_percent: f64|
                 -> Option<(f64, f64, String, f64, f64)> {
                    let delta_abs = (result - target).abs();
                    let delta_percent = if target > 0. {
                        delta_abs * 100. / target
                    } else {
                        0.
                    };
                    /*    println!(
                        "{text} result:{result} target:{target} delta_abs:{delta_abs} delta_percent:{delta_percent}"
                    );*/
                    if delta_abs > epsilon_abs && delta_percent > epsilon_percent {
                        return Some((delta_abs, delta_percent, text, result, target));
                    }
                    None
                };
                check(
                    format!("{:.3} {:.3} {:.1} volume", heel, trim, draught),
                    result_volume,
                    target_volume,
                    epsilon_volume_abs,
                    epsilon_volume_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.x", heel, trim, draught),
                    result_center.x,
                    target_center.x,
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.y", heel, trim, draught),
                    result_center.y,
                    target_center.y,
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.z", heel, trim, draught),
                    result_center.z,
                    target_center.z,
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
            }
        }
    }
    results.sort_by(|a, b| (a.1).partial_cmp(&b.1).unwrap());
    //  let _ = results.split_off(100);
    for v in results {
        println!(
            "{} error: result:{} target:{} delta_abs:{} delta_percent:{}",
            v.2, v.3, v.4, v.0, v.1
        );
    }
}

#[test]
fn hydrostatic_waterline_sofia1() {
    let dbg = Dbg::new("test", "hydrostatic_sofia");
    let scale = 0.001f64;
    let path = "assets/hull.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let dx = 65.25;
    let mut cache = DisplacementCache::new(&dbg, "assets/displacement_cache_hull".into());
    dbg!(cache.init().unwrap());
      let heel_steps = vec![
        -60., -50., -45., -40., -35., -30., -25., -20., -15., -10., -5., -2., -1., -0.5, -0.2, 0.,
        0.2, 0.5, 1., 2., 5., 10., 15., 20., 25., 30., 35., 40., 45., 50., 60.,
    ];
    let trim_steps = vec![
        -40., -30., -25., -20., -15., -12.5, -10., -7.5, -5., -3., -2., -1., -0.5, -0.2, 0., 0.2,
        0.5, 1., 2., 3., 5., 7.5, 10., 12.5, 20., 25., 30., 40.,
    ];
    let draught_steps: Vec<_> = (1..=28).map(|v| (v as f64) * 0.5).collect();
  //  let heel_steps = vec![-20., 0., 20.];
   // let trim_steps = vec![ -20., 0., 20., ];
 //   let draught_steps: Vec<_> = vec![4.];
    let epsilon_volume_abs = 10.;
    let epsilon_volume_percent = 0.1;
    let epsilon_center_abs = 0.01;
    let epsilon_center_percent = 0.1;
    let mut results = Vec::new();
    for &heel in &heel_steps {
        for &trim in &trim_steps {
            for &draught in &draught_steps {
                let (result_area, result_center) =
                    calculate_waterline(&mesh, dx, heel, trim, draught);
            /*    println!(
                    "{:.3} {:.3} {:.3}: area:{:.3} x:{:.3} y:{:.3} z:{:.3}",
                    heel,
                    trim,
                    draught,
                    result_area,
                    result_center.x,
                    result_center.y,
                    result_center.z,
                );*/
                let target = cache.get_from_level(heel, trim, draught);
                let check = |text: String,
                             result: f64,
                             target: f64,
                             epsilon_abs: f64,
                             epsilon_percent: f64|
                 -> Option<(f64, f64, String, f64, f64)> {
                    let delta_abs = (result - target).abs();
                    let delta_percent = if target > 0. {
                        delta_abs * 100. / target
                    } else {
                        0.
                    };
                    /*    println!(
                        "{text} result:{result} target:{target} delta_abs:{delta_abs} delta_percent:{delta_percent}"
                    );*/
                    if delta_abs > epsilon_abs && delta_percent > epsilon_percent {
                        return Some((delta_abs, delta_percent, text, result, target));
                    }
                    None
                };
                check(
                    format!("{:.3} {:.3} {:.1} area", heel, trim, draught),
                    result_area,
                    target.area_wl,
                    epsilon_volume_abs,
                    epsilon_volume_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.x", heel, trim, draught),
                    result_center.x,
                    target.area_wl_center.x(),
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.y", heel, trim, draught),
                    result_center.y,
                    target.area_wl_center.y(),
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
                check(
                    format!("{:.3} {:.3} {:.1} center.z", heel, trim, draught),
                    result_center.z,
                    target.area_wl_center.z(),
                    epsilon_center_abs,
                    epsilon_center_percent,
                )
                .map(|v| results.push(v));
            }
        }
    }
    results.sort_by(|a, b| (a.1).partial_cmp(&b.1).unwrap());
    let _ = results.split_off(100);
    for v in results {
        println!(
            "{} error: result:{} target:{} delta_abs:{} delta_percent:{}",
            v.2, v.3, v.4, v.0, v.1
        );
    }
}

#[test]
fn hydrostatic_waterline_sofia2() {
    let dbg = Dbg::new("test", "hydrostatic_sofia");
    let scale = 0.001f64;
    let path = "assets/Sofiya_4work.stl";
    let mesh = load_stl(Path::new(path)).scaled(Vec3::new(scale, scale, scale));
    let dx = 65.25;
    let mut cache = DisplacementCache::new(&dbg, "assets/displacement_cache_hull".into());
    dbg!(cache.init().unwrap());
    /*  let heel_steps = vec![
        -60., -50., -45., -40., -35., -30., -25., -20., -15., -10., -5., -2., -1., -0.5, -0.2, 0.,
        0.2, 0.5, 1., 2., 5., 10., 15., 20., 25., 30., 35., 40., 45., 50., 60.,
    ];
    let trim_steps = vec![
        -40., -30., -25., -20., -15., -12.5, -10., -7.5, -5., -3., -2., -1., -0.5, -0.2, 0., 0.2,
        0.5, 1., 2., 3., 5., 7.5, 10., 12.5, 20., 25., 30., 40.,
    ];
    let draught_steps: Vec<_> = (1..=28).map(|v| (v as f64) * 0.5).collect();*/
    let heel_steps = vec![
        -20., 0., 20.,
     //        0.,
    ];
    let trim_steps = vec![
        -20., 0., 20.,
      //         0.,
    ];
    let draught_steps: Vec<_> = (4..=10).map(|v| (v as f64) ).collect();
    for &heel in &heel_steps {
        for &trim in &trim_steps {
            for &draught in &draught_steps {
                let (area, center) = calculate_waterline(&mesh, dx, heel, trim, draught);
                println!(
                    "{:.3} {:.3} {:.3}: area:{:.3} x:{:.3} y:{:.3} z:{:.3}",
                    heel, trim, draught, area, center.x, center.y, center.z,
                );
            }
        }
    }
}
