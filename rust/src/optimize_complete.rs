use std::{cell::RefCell, collections::HashMap, iter, rc::Rc};

use petgraph::{
    dot::{Config, Dot},
    graph::DiGraph,
};
use russcip::{Solution, Variable, prelude::*};

use crate::{Routine, optimize::ProblemInfo};

#[derive(Clone)]
struct Node {
    out_arcs: Vec<Variable>,
}

fn successor(nodes: &[Node], node_index: usize, mut get_value: impl FnMut(&Variable) -> f64) -> Option<usize> {
    nodes[node_index].out_arcs.iter().enumerate().find_map(|(j, arc_var)| {
        let value = get_value(arc_var);
        debug_assert!(
            (value - value.round()).abs() < 1e-6,
            "solution given to `successor` is expected to be integral, but arc var {node_index}->{j} has value {value}"
        );
        if value > 0.5 { Some(j) } else { None }
    })
}

/// A constraint handler that enforces the TSP subtour elimination constraint.
struct SubtourElimination {
    nodes: Rc<[Node]>,
}

impl SubtourElimination {
    fn cycles(&self, mut get_value: impl FnMut(&Variable) -> f64) -> impl Iterator<Item = Vec<usize>> {
        let n = self.nodes.len();
        let mut visited = vec![false; n];

        (0..n).filter_map(move |start| {
            if visited[start] {
                return None;
            }

            // Collect the cycle starting from `start`.
            let mut cycle = vec![];
            let mut current_node = start;
            loop {
                cycle.push(current_node);
                visited[current_node] = true;

                if let Some(next_node) = successor(&self.nodes, current_node, &mut get_value)
                    && !visited[next_node]
                {
                    current_node = next_node;
                } else {
                    break;
                }
            }

            Some(cycle)
        })
    }
}

impl Conshdlr for SubtourElimination {
    fn check(&mut self, _model: Model<Solving>, _conshdlr: SCIPConshdlr, solution: &Solution) -> bool {
        // println!("SubtourElimination::check");

        let n = self.nodes.len();
        self.cycles(|var| solution.val(var)).all(|cycle| cycle.len() == n)
    }

    fn enforce(&mut self, mut model: Model<Solving>, _conshdlr: SCIPConshdlr) -> ConshdlrResult {
        println!("SubtourElimination::enforce (node {})", model.focus_node().number());

        let n = self.nodes.len();
        let mut added_constraint = false;

        let model = RefCell::new(&mut model);
        for cycle in self.cycles(|var| model.borrow().current_val(var)) {
            if cycle.len() < n {
                // Found a subtour; add a constraint to eliminate it.
                let nodes = &self.nodes;
                let constraint = cons()
                    .expr(cycle.iter().flat_map(|&i| cycle.iter().map(move |&j| (&nodes[i].out_arcs[j], 1.0))))
                    .le((cycle.len() - 1) as f64);
                model.borrow_mut().add(constraint);
                // model.borrow_mut().add_cons_local(&constraint);
                added_constraint = true;
            }
        }

        if added_constraint { ConshdlrResult::ConsAdded } else { ConshdlrResult::Feasible }
    }
}

pub fn optimize_order(routines: &[Routine]) -> Vec<usize> {
    let n = routines.len() + 1; // +1 for the start/end node
    let end_index = n - 1;
    let problem_info = ProblemInfo::new(routines);

    let start = std::time::Instant::now();

    let mut model = Model::default().minimize();
    // model = model.set_display_verbosity(5);

    // Variables: one binary variable for each arc in the complete directed graph.
    let nodes: Rc<[Node]> = (0..n)
        .map(|i| {
            let out_arcs: Vec<Variable> = (0..n)
                .map(|j| {
                    let cost = if i == end_index || j == end_index {
                        0.0
                    } else {
                        problem_info.intersection_count(i, j) as f64
                    };
                    model.add(var().name(&format!("arc_{i}_to_{j}")).bin().obj(cost))
                })
                .collect();

            // Forbid self-loops.
            model.add(cons().coef(&out_arcs[i], 1.0).eq(0.0));

            Node { out_arcs }
        })
        .collect();

    for i in 0..n {
        // Constraint: exactly one outgoing arc from each node.
        model.add(cons().expr(nodes[i].out_arcs.iter().map(|arc| (arc, 1.0))).eq(1.0));

        // Constraint: exactly one incoming arc to each node.
        model.add(cons().expr((0..n).map(|j| (&nodes[j].out_arcs[i], 1.0))).eq(1.0));
    }

    // Distance-2 cost variables and constraints.
    let mut pair_vars = HashMap::new();
    for i1 in 0..routines.len() {
        for i3 in (i1 + 1)..routines.len() {
            let conflict_count = problem_info.intersection_count(i1, i3);
            if conflict_count == 0 {
                continue;
            }

            let pair_var =
                model.add(var().name(&format!("d2_pair_({i1},{i3})")).impl_int(0..=1).obj(conflict_count as f64));
            pair_vars.insert((i1, i3), pair_var.clone());

            for i2 in 0..routines.len() {
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
    println!("Added {} distance-2 pair variables.", pair_vars.len());

    // Constraint: subtour elimination.
    let cons_handler = SubtourElimination { nodes: nodes.clone() };
    model.include_conshdlr("SEC", "Subtour Elimination Constraint", -1, -1, Box::new(cons_handler));

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

    let mut order = Vec::with_capacity(n);
    let mut current_node = end_index;
    loop {
        current_node = successor(&nodes, current_node, |var| solution.val(var)).unwrap();
        if current_node == end_index {
            break;
        }
        order.push(current_node);
    }
    assert_eq!(order.len(), routines.len());

    for pos in 0..(order.len() - 2) {
        let i1 = order[pos];
        let i2 = order[pos + 1];
        let i3 = order[pos + 2];
        let conflict_count =
            if i2 == problem_info.intermission_index() { 0 } else { problem_info.intersection_count(i1, i3) };
        println!("  {i1:>2} -> * -> {i3:>2}: {}", conflict_count);
    }

    order
}
