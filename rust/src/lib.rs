mod optimize;
mod optimize2;
mod randomize;

use std::collections::HashSet;

use serde::{Deserialize, Serialize, Serializer};
use wasm_bindgen::JsValue;
use wasm_bindgen::prelude::*;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Routine {
    pub name: String,
    #[serde(serialize_with = "serialize_sorted")]
    pub dancers: HashSet<String>,
}

impl Routine {
    pub fn is_doubleable(&self) -> bool {
        (1..=2).contains(&self.dancers.len())
    }
}

fn serialize_sorted<I, S>(set: &HashSet<I>, serializer: S) -> Result<S::Ok, S::Error>
where
    I: Ord + Serialize,
    S: Serializer,
{
    let mut sorted: Vec<_> = set.iter().collect();
    sorted.sort();
    sorted.serialize(serializer)
}

#[wasm_bindgen(start)]
pub fn main() {
    console_error_panic_hook::set_once();
}

// ── Serialization types ────────────────────────────────────────────────────

#[derive(Serialize, Deserialize)]
struct SlotResult {
    slot_number: usize,
    routines: Vec<String>,
    dist1_conflicts: Vec<String>,
    dist2_conflicts: Vec<String>,
}

#[derive(Serialize, Deserialize)]
struct OptimizeOutput {
    slots: Vec<SlotResult>,
    score: [usize; 3],
}

// ── Public WASM API ────────────────────────────────────────────────────────

/// Parse a CSV in "Showcase Names" format and return a JSON array of routines.
///
/// CSV format: header row = routine names (columns), subsequent rows = dancer names per column.
/// An `[Intermission]` routine is automatically appended.
///
/// The returned JSON can be passed directly to `optimize()`.
#[wasm_bindgen]
pub fn parse_csv(csv: &str) -> Result<String, String> {
    let routines = do_parse_csv(csv)?;
    serde_json::to_string(&routines).map_err(|e| e.to_string())
}

/// Run the local-search show scheduler optimizer.
///
/// - `routines_json`: JSON array of `{name, dancers}` objects, as returned by `parse_csv`.
/// - `num_slots`: total number of time slots (including intermission).
/// - `num_iterations`: number of random-restart hill-climb iterations.
///
/// Returns JSON: `{ slots: [{slot_number, routines, dist1_conflicts, dist2_conflicts}], score: [d1, d2, mid] }`.
#[wasm_bindgen]
pub fn optimize(routines_json: &str, num_slots: usize, num_iterations: u32) -> Result<String, String> {
    let routines: Vec<Routine> = serde_json::from_str(routines_json).map_err(|e| e.to_string())?;

    if !routines.iter().any(|r| r.name == "[Intermission]") {
        return Err("routines must include an '[Intermission]' entry (call parse_csv to generate it)".into());
    }

    let (order, score) = optimize2::optimize_order(&routines, num_slots, num_iterations);
    let problem_info = optimize::ProblemInfo::new(&routines, num_slots);
    let slots = build_slot_results(&order, &problem_info, &routines);

    let output = OptimizeOutput { slots, score: [score.0, score.1, score.2] };
    serde_json::to_string(&output).map_err(|e| e.to_string())
}

/// Run the show scheduler optimizer continuously, calling `callback` with a JSON result
/// each time a better solution is found.
///
/// - `routines_json`: JSON array of `{name, dancers}` objects, as returned by `parse_csv`.
/// - `num_slots`: total number of time slots (including intermission).
/// - `callback`: called with a JSON result string (`{ slots, score }`) on each improvement.
///
/// Stops automatically after 5 seconds with no improvement (or immediately on a perfect score).
#[wasm_bindgen]
pub fn optimize_streaming(routines_json: &str, num_slots: usize, callback: &js_sys::Function) -> Result<(), String> {
    let routines: Vec<Routine> = serde_json::from_str(routines_json).map_err(|e| e.to_string())?;

    if !routines.iter().any(|r| r.name == "[Intermission]") {
        return Err("routines must include an '[Intermission]' entry (call parse_csv to generate it)".into());
    }

    let problem_info = optimize::ProblemInfo::new(&routines, num_slots);

    optimize2::optimize_order_streaming(&routines, num_slots, |order, score| {
        let slots = build_slot_results(order, &problem_info, &routines);
        let output = OptimizeOutput { slots, score: [score.0, score.1, score.2] };
        if let Ok(json) = serde_json::to_string(&output) {
            let _ = callback.call1(&JsValue::NULL, &JsValue::from_str(&json));
        }
    });

    Ok(())
}

// ── Helpers ────────────────────────────────────────────────────────────────

fn do_parse_csv(csv: &str) -> Result<Vec<Routine>, String> {
    let mut lines = csv.lines();
    let header = lines.next().ok_or("CSV is empty")?;
    let names: Vec<String> = header.split(',').map(|s| s.trim().to_string()).collect();

    let mut routines: Vec<Routine> = names.into_iter().map(|name| Routine { name, dancers: HashSet::new() }).collect();

    for line in lines {
        for (i, cell) in line.split(',').enumerate() {
            let dancer = cell.trim();
            if !dancer.is_empty()
                && let Some(r) = routines.get_mut(i)
            {
                r.dancers.insert(dancer.to_string());
            }
        }
    }

    routines.push(Routine { name: "[Intermission]".to_string(), dancers: HashSet::new() });
    routines.sort_by_key(|r| (std::cmp::Reverse(r.dancers.len()), r.name.clone()));

    Ok(routines)
}

fn build_slot_results(order: &[usize], problem_info: &optimize::ProblemInfo, routines: &[Routine]) -> Vec<SlotResult> {
    let n = order.len();
    (0..n)
        .map(|pos| {
            let mr = &problem_info.meta_routines()[order[pos]];
            let routine_names: Vec<String> = mr.routine_indices().map(|i| routines[i].name.clone()).collect();

            let dancers0: HashSet<&String> = mr.dancers(routines).collect();

            let dist1_conflicts = order
                .get(pos + 1)
                .map(|&next| {
                    let dancers1: HashSet<&String> = problem_info.meta_routines()[next].dancers(routines).collect();
                    let mut v: Vec<String> = dancers0.intersection(&dancers1).map(|s| (*s).clone()).collect();
                    v.sort();
                    v
                })
                .unwrap_or_default();

            let dist2_conflicts = order
                .get(pos + 1)
                .filter(|&&next| next != problem_info.intermission_index())
                .and_then(|_| order.get(pos + 2))
                .map(|&next2| {
                    let dancers2: HashSet<&String> = problem_info.meta_routines()[next2].dancers(routines).collect();
                    let mut v: Vec<String> = dancers0.intersection(&dancers2).map(|s| (*s).clone()).collect();
                    v.sort();
                    v
                })
                .unwrap_or_default();

            SlotResult { slot_number: pos + 1, routines: routine_names, dist1_conflicts, dist2_conflicts }
        })
        .collect()
}
