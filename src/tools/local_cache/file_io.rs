use std::{
    fs::File,
    io::{BufRead, BufReader, Error, Write},
    path::PathBuf,
};


///
/// read cache data from `path` file.
///
/// # Panics
/// Panic occurs if the reader produces a non-comparable value (e. g. _NaN_).
pub fn read(cache_path: &PathBuf) -> Vec<Vec<f64>> {
    let parent_dir = cache_path.parent().unwrap();
    std::fs::create_dir_all(parent_dir).unwrap();
    let callee = "read_from_file";
    let file = File::open(cache_path).unwrap();
    let reader = BufReader::new(file);
    let mut vals = Vec::new();
    for (try_line, line_id) in reader.lines().zip(1..) {
        let mut v = Vec::new();
        let line = try_line.unwrap();
        let ss = line.split_ascii_whitespace();
        for s in ss {
            let val = s.parse().unwrap();
            v.push(val);
        }
        vals.push(v);
    }
    let size = vals
        .first()
        .unwrap()
        .len();
    for v in &vals {
        if v.len() != size {
            panic!("{} | Error: no vals", callee);
        }
    }
    vals
}
///
pub fn save( cache_path: &PathBuf, vals: Vec<Vec<f64>>) -> Result<(), Error> {
    let parent_dir = cache_path.parent().unwrap();
    std::fs::create_dir_all(parent_dir).unwrap();
    let mut file = File::create(cache_path).unwrap();
    for col in vals.iter() {
        let cols_str: Vec<_> = col.iter().map(ToString::to_string).collect();
        let line = cols_str.join("\t");
        writeln!(&mut file, "{}", line).unwrap();
    }
    Ok(())
}
