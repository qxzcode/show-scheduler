use core::f64;
use std::{
    collections::{HashMap, HashSet},
    rc::Rc,
};

use petgraph::{
    dot::{Config, Dot},
    graph::DiGraph,
};
use rand::seq::SliceRandom;
use russcip::{Variable, prelude::*};

use crate::{
    Routine,
    local_search::HillClimbHeuristic,
    optimize::{MetaRoutine, ProblemInfo},
    subtour_elimination::{SubtourElimination, SubtourElimination3},
};

#[derive(Clone)]
pub struct Node {
    pub out_arcs: Vec<Variable>,
    pub is_before_intermission: Variable,
}

pub const EPS: f64 = 1e-6;

pub fn successor(nodes: &[Node], node_index: usize, mut get_value: impl FnMut(&Variable) -> f64) -> Option<usize> {
    nodes[node_index].out_arcs.iter().enumerate().find_map(|(j, arc_var)| {
        let value = get_value(arc_var);
        debug_assert!(
            (value - value.round()).abs() < EPS,
            "solution given to `successor` is expected to be integral, but arc var {node_index}->{j} has value {value}"
        );
        if value > 0.5 { Some(j) } else { None }
    })
}

pub fn sort_pair(i1: usize, i3: usize) -> (usize, usize) {
    if i1 < i3 { (i1, i3) } else { (i3, i1) }
}

pub const D1_COST_MULTIPLIER: usize = 100 * 1_000;
pub const D2_COST_MULTIPLIER: usize = 100;

/// Finds an optimal ordering of the given routines.
/// Returns a vector of routine indices in the optimized order.
pub fn optimize_order(routines: &[Routine], num_slots: usize) -> Vec<usize> {
    let problem_info = ProblemInfo::new(routines, num_slots);
    let n = problem_info.meta_routines().len() + 1; // +1 for the start/end node
    let end_index = n - 1;

    let start = std::time::Instant::now();

    let mut model = Model::default().minimize();

    // Disable SCIP's default heuristics.
    // The problem-specific heuristic is much better, to the point that the default heuristics just slow things down.
    model = model.set_heuristics(ParamSetting::Off);
    // model = model.set_separating(ParamSetting::Off);
    // model = model.set_presolving(ParamSetting::Fast); // TODO: testing this out; keep?

    // Variables: one binary variable for each arc in the complete directed graph.
    let nodes: Rc<[Node]> = (0..n)
        .map(|i| {
            let out_arcs: Vec<Variable> = (0..n)
                .map(|j| {
                    let cost = if i == j || i == end_index || j == end_index {
                        0.0
                    } else {
                        (problem_info.intersection_count(i, j) * D1_COST_MULTIPLIER) as f64
                    };
                    model.add(var().name(&format!("arc_{i}_to_{j}")).bin().obj(cost))
                })
                .collect();

            let is_before_intermission = model.add(var().name(&format!("is_before_intermission_{i}")).bin());

            Node { out_arcs, is_before_intermission }
        })
        .collect();

    // Forbid a self-loop on the start/end node.
    model.add(cons().coef(&nodes[end_index].out_arcs[end_index], 1.0).eq(0.0));

    // Schedule exactly `num_slots` meta-routines in total.
    // This means the total number of self-loops selected (i.e. meta-routines not scheduled) must be `meta_routines.len() - num_slots`.
    model.add(
        cons()
            .expr((0..(n - 1)).map(|i| (&nodes[i].out_arcs[i], 1.0)))
            .eq((problem_info.meta_routines().len() - num_slots) as f64),
    );

    // Each routine must be scheduled exactly once.
    for r in 0..routines.len() {
        // A meta-routine is scheduled iff its self-loop arc is NOT selected.
        // Exactly one of the meta-routines containing routine `r` must be scheduled.
        let mut constraint = cons();
        let mut rhs = 1.0;
        for (i, mr) in problem_info.meta_routines().iter().enumerate() {
            if mr.contains_routine(r) {
                // Add (1 - loop_arc) to the LHS of the constraint.
                constraint = constraint.coef(&nodes[i].out_arcs[i], -1.0);
                rhs -= 1.0;
            }
        }
        model.add(constraint.eq(rhs));
    }

    for i in 0..n {
        // Constraint: exactly one outgoing arc from each node.
        model.add(cons().expr(nodes[i].out_arcs.iter().map(|arc| (arc, 1.0))).eq(1.0));

        // Constraint: exactly one incoming arc to each node.
        model.add(cons().expr((0..n).map(|j| (&nodes[j].out_arcs[i], 1.0))).eq(1.0));

        // Redundant constraint: at most one arc between each pair of nodes.
        for j in (i + 1)..n {
            model.add(cons().coef(&nodes[i].out_arcs[j], 1.0).coef(&nodes[j].out_arcs[i], 1.0).le(1.0));
        }
    }

    // Distance-2 cost variables and constraints.
    // One variable for each pair of routines with non-zero conflict.
    let mut pair_vars = HashMap::new();
    for r1 in 0..problem_info.routines().len() {
        let dancers1 = &problem_info.routines()[r1].dancers;

        for r3 in (r1 + 1)..problem_info.routines().len() {
            let dancers3 = &problem_info.routines()[r3].dancers;

            let conflict_count = dancers1.intersection(dancers3).count();
            if conflict_count == 0 {
                continue;
            }

            let cost = (conflict_count * D2_COST_MULTIPLIER) as f64;
            let pair_var = model.add(var().name(&format!("d2_pair_({r1},{r3})")).bin().obj(cost));
            pair_vars.insert((r1, r3), pair_var.clone());

            for i1 in 0..problem_info.meta_routines().len() {
                if !problem_info.meta_routines()[i1].contains_routine(r1) {
                    continue;
                }

                for i3 in (i1 + 1)..problem_info.meta_routines().len() {
                    if !problem_info.meta_routines()[i3].contains_routine(r3) {
                        continue;
                    }

                    for i2 in 0..problem_info.meta_routines().len() {
                        if i2 == i1 || i2 == i3 || i2 == problem_info.intermission_index() {
                            continue;
                        }

                        model.add(
                            cons()
                                .coef(&pair_var, -1.0)
                                // i1 -> i2 -> i3:
                                .coef(&nodes[i1].out_arcs[i2], 1.0)
                                .coef(&nodes[i2].out_arcs[i3], 1.0)
                                // i3 -> i2 -> i1:
                                .coef(&nodes[i3].out_arcs[i2], 1.0)
                                .coef(&nodes[i2].out_arcs[i1], 1.0)
                                .le(1.0),
                        );
                    }
                }
            }
        }
    }
    println!("Added {} distance-2 pair variables.", pair_vars.len());

    // Constraints to enforce correctness of `is_before_intermission` variables.
    for i in 0..n {
        let i_before_intermission = &nodes[i].is_before_intermission;
        if i == problem_info.intermission_index() || i == end_index {
            // Intermission itself and the start/end node are not before intermission.
            model.add(cons().coef(i_before_intermission, 1.0).eq(0.0));
        } else {
            // If node `i` has a direct arc to intermission, then `i` is before intermission.
            let arc_to_intermission = &nodes[i].out_arcs[problem_info.intermission_index()];
            model.add(cons().coef(arc_to_intermission, 1.0).coef(i_before_intermission, -1.0).le(0.0));

            // If intermission has a direct arc to node `i`, then `i` is not before intermission.
            let arc_from_intermission = &nodes[problem_info.intermission_index()].out_arcs[i];
            model.add(cons().coef(arc_from_intermission, 1.0).coef(i_before_intermission, 1.0).le(1.0));

            // If node `i` is omitted from the tour (i.e. its self-loop arc is selected), then `i` is not before intermission.
            let self_loop_arc = &nodes[i].out_arcs[i];
            model.add(cons().coef(self_loop_arc, 1.0).coef(i_before_intermission, 1.0).le(1.0));

            for j in 0..n {
                if !(j == i || j == problem_info.intermission_index() || j == end_index) {
                    // If there is an arc from `i` to `j`, then `i` is before intermission iff `j` is before intermission.
                    let arc_ij = &nodes[i].out_arcs[j];
                    let j_before_intermission = &nodes[j].is_before_intermission;

                    // let vars = vec![i_before_intermission, j_before_intermission];
                    // model.add_cons_indicator(arc_ij, vars.clone(), &mut [1.0, -1.0], 0.0, "indicator");
                    // model.add_cons_indicator(arc_ij, vars, &mut [-1.0, 1.0], 0.0, "indicator");

                    model.add(
                        cons()
                            .coef(arc_ij, 1.0)
                            .coef(i_before_intermission, -1.0)
                            .coef(j_before_intermission, 1.0)
                            .le(1.0),
                    );
                    model.add(
                        cons()
                            .coef(arc_ij, 1.0)
                            .coef(i_before_intermission, 1.0)
                            .coef(j_before_intermission, -1.0)
                            .le(1.0),
                    );
                }
            }
        }
    }

    let max_middle_dist = (num_slots - 1) / 2;
    let intermission_middle_distance =
        model.add(var().name("intermission_middle_distance").int(0..=max_middle_dist as isize).obj(1.0));
    // Constraint: intermission_middle_distance >= max_middle_dist - (sum_{i before intermission} 1)
    model.add(
        cons()
            .coef(&intermission_middle_distance, 1.0)
            .expr((0..n).map(|i| (&nodes[i].is_before_intermission, 1.0)))
            .ge(max_middle_dist as f64),
    );
    // Constraint: intermission_middle_distance >= (sum_{i before intermission} 1) - (num_slots / 2)
    model.add(
        cons()
            .coef(&intermission_middle_distance, 1.0)
            .expr((0..n).map(|i| (&nodes[i].is_before_intermission, -1.0)))
            .ge(-((num_slots / 2) as f64)),
    );

    // DEBUG: Constrain the middle distance to not be too large.
    model.add(cons().coef(&intermission_middle_distance, 1.0).le(3.0));

    // Constraint: subtour elimination.
    let cons_handler = SubtourElimination::new(nodes.clone(), num_slots + 1);
    model.include_conshdlr("SEC", "Subtour Elimination Constraint", -1, -1, Box::new(cons_handler));

    // // Constraint: triple elimination.
    // let cons_handler = SubtourElimination3::new(nodes.clone(), num_slots + 1);
    // model.include_conshdlr("TEC", "Triple Elimination Constraint", 1, 1, Box::new(cons_handler));

    // // Constraint: distance-2 costs.
    // let cons_handler = crate::lazy_d2_costs::Distance2Costs::new(problem_info.clone(), nodes.clone());
    // model.include_conshdlr("D2C", "Distance-2 Costs Constraint", -2, -2, Box::new(cons_handler));

    // // Enforce no distance-1 conflicts.
    // for i in 0..routines.len() {
    //     for j in (i + 1)..routines.len() {
    //         if problem_info.intersection_count(i, j) > 0 {
    //             model.add(cons().coef(&nodes[i].out_arcs[j], 1.0).eq(0.0));
    //             model.add(cons().coef(&nodes[j].out_arcs[i], 1.0).eq(0.0));
    //         }
    //     }
    // }

    // // Primal heuristic: hill climbing local search.
    // let heuristic =
    //     HillClimbHeuristic::new(problem_info.clone(), nodes.clone(), pair_vars.clone(), intermission_middle_distance);
    // heuristic.try_find_solution(&model, None, 1_000, {
    //     // Sample random initial solutions for improvement.
    //     let mut rng = rand::rng();
    //     move |order| order.shuffle(&mut rng)
    // });
    // model.add(
    //     heur(heuristic)
    //         .name("HillClimbHeuristic")
    //         .desc("Hill climbing primal heuristic")
    //         .timing(HeurTiming::AFTER_LP_LOOP),
    // );

    println!("Built model in: {:?}", start.elapsed());
    let start = std::time::Instant::now();

    println!("=== solving... ===");
    let solved = model.solve();
    println!("\nSolved in: {:?}", start.elapsed());

    println!();
    dbg!(solved.status());
    let solution = solved.best_sol().expect("no solution found");
    dbg!(solution.obj_val());

    let mut graph = DiGraph::<_, ()>::new();
    let graph_nodes = (0..n).map(|i| graph.add_node(i)).collect::<Vec<_>>();
    for (i, node) in nodes.iter().enumerate() {
        for (j, arc_var) in node.out_arcs.iter().enumerate() {
            if solution.val(arc_var) > 0.5 {
                graph.add_edge(graph_nodes[i], graph_nodes[j], ());
            }
        }
    }
    std::fs::write("debug.dot", format!("{:?}", Dot::with_config(&graph, &[Config::EdgeNoLabel]))).unwrap();

    let mut order = Vec::with_capacity(num_slots);
    let mut current_node = end_index;
    loop {
        current_node = successor(&nodes, current_node, |var| solution.val(var)).unwrap();
        if current_node == end_index {
            break;
        }
        order.push(current_node);
    }
    assert_eq!(order.len(), num_slots);

    // for pos in 0..(order.len() - 2) {
    //     let i1 = order[pos];
    //     let i2 = order[pos + 1];
    //     let i3 = order[pos + 2];
    //     let conflict_count =
    //         if i2 == problem_info.intermission_index() { 0 } else { problem_info.intersection_count(i1, i3) };
    //     println!("  {i1:>2} -> _ -> {i3:>2}: {}", conflict_count);
    // }

    println!();
    for pos in 0..order.len() {
        let mr = &problem_info.meta_routines()[order[pos]];
        print!(
            "    Slot {:>2} (is_before_intermission={}):  ",
            pos + 1,
            solution.val(&nodes[order[pos]].is_before_intermission)
        );
        match mr {
            MetaRoutine::Single(r) => {
                println!("{}", routines[*r].name);
            }
            MetaRoutine::Double(r1, r2) => {
                println!("{},  {}", routines[*r1].name, routines[*r2].name);
            }
        }

        // Show conflicts, if any.
        let dancers0: HashSet<_> = mr.dancers(routines).collect();
        if let Some(mr1) = order.get(pos + 1) {
            let dancers1: HashSet<_> = problem_info.meta_routines()[*mr1].dancers(routines).collect();
            if !dancers0.is_disjoint(&dancers1) {
                let conflicts: Vec<_> = dancers0.intersection(&dancers1).collect();
                println!("        distance-1 conflicts ({}):  {conflicts:?}", conflicts.len());
            }

            if *mr1 != problem_info.intermission_index()
                && let Some(mr2) = order.get(pos + 2)
            {
                let dancers2: HashSet<_> = problem_info.meta_routines()[*mr2].dancers(routines).collect();
                if !dancers0.is_disjoint(&dancers2) {
                    let conflicts: Vec<_> = dancers0.intersection(&dancers2).collect();
                    println!("        distance-2 conflicts ({}):  {conflicts:?}", conflicts.len());
                }
            }
        }
    }
    println!();

    order
}
