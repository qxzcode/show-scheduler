mod lazy_d2_costs;
pub mod local_search;
pub mod optimize;
pub mod optimize_complete;

use std::{
    cmp::Reverse,
    collections::{HashMap, HashSet},
};

#[derive(Clone, Debug)]
pub struct Routine {
    pub name: String,
    pub dancers: HashSet<String>,
}

#[derive(Clone, Debug)]
pub struct InputData {
    pub routines: Vec<Routine>,
    pub dancers: Vec<String>,
}

fn load_dancer_dupe_map() -> HashMap<String, String> {
    let path = "../dancer_dupe_map.py";
    let content = std::fs::read_to_string(path).expect("Failed to read dancer_dupe_map.py");

    let mut map = HashMap::new();
    for line in content.lines() {
        let mut parts = line.split('"').peekable();
        let key = if let Some(key) = parts.nth(1) {
            key
        } else {
            continue;
        };
        let value = parts.nth(1).expect("Malformed line in dancer_dupe_map.py");
        map.insert(key.to_string(), value.to_string());
    }
    map
}

fn load_data() -> csv::Result<InputData> {
    let mut rdr = csv::Reader::from_path("../Showcase Names - Sheet1.csv")?;
    let headers = rdr.headers()?.clone();

    let mut routines = Vec::new();
    for h in headers.iter() {
        let name = h.trim().to_string();
        routines.push(Routine { name, dancers: HashSet::new() });
    }

    let dancer_dupe_map = load_dancer_dupe_map();
    for result in rdr.records() {
        let record = result?;
        for (i, dancer) in record.iter().enumerate() {
            let dancer = dancer.trim();
            let dancer = dancer_dupe_map.get(dancer).map(|s| s.as_str()).unwrap_or(dancer);
            if dancer.is_empty() {
                continue;
            }
            routines[i].dancers.insert(dancer.to_string());
        }
    }

    // Add intermission as empty routine
    routines.push(Routine { name: "[Intermission]".to_string(), dancers: HashSet::new() });

    routines.sort_by_key(|r| (Reverse(r.dancers.len()), r.name.clone()));

    let dancer_set: HashSet<&String> = HashSet::from_iter(routines.iter().flat_map(|r| r.dancers.iter()));
    let mut dancers: Vec<String> = dancer_set.into_iter().cloned().collect();
    dancers.sort();

    Ok(InputData { routines, dancers })
}

fn main() {
    let input_data = load_data().expect("Failed to load data");

    for _ in 0..1 {
        let optimized_order = optimize_complete::optimize_order(&input_data.routines);
        println!("Optimal routine order: {optimized_order:?}");
    }

    // println!("{} routines (including intermission):", input_data.routines.len());
    // let max_name_len = input_data.routines.iter().map(|r| r.name.len()).max().unwrap_or(0);
    // for r in &input_data.routines {
    //     println!("    {:max_name_len$}  with {} dancer(s)", r.name, r.dancers.len());
    // }
    // println!();

    // println!("{} dancers:", input_data.dancers.len());
    // let max_name_len = input_data.dancers.iter().map(|d| d.len()).max().unwrap_or(0);
    // for d in &input_data.dancers {
    //     println!("    {:max_name_len$}", d);
    // }
    // println!();

    // let (optimized_order, score) = optimize::optimize_order(&input_data.routines);
    // println!("Optimized routine order:");
    // println!("{optimized_order:?}");
    // for &idx in &optimized_order {
    //     println!("    {}", input_data.routines[idx].name);
    // }
    // println!("Score: {:?}", score);
}
