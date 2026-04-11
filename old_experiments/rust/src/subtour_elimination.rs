use std::{cell::RefCell, rc::Rc};

use russcip::{Solution, Variable, prelude::*};

use crate::optimize_complete::{EPS, Node, successor};

/// A constraint handler that enforces the TSP subtour elimination constraint.
/// Allows self-loops (arc from a node to itself).
pub struct SubtourElimination {
    nodes: Rc<[Node]>,
    tour_len: usize,
}

impl SubtourElimination {
    pub fn new(nodes: Rc<[Node]>, tour_len: usize) -> Self {
        Self { nodes, tour_len }
    }

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

        self.cycles(|var| solution.val(var)).all(|cycle| cycle.len() == 1 || cycle.len() == self.tour_len)
    }

    fn enforce(&mut self, mut model: Model<Solving>, _conshdlr: SCIPConshdlr) -> ConshdlrResult {
        println!(
            "SubtourElimination::enforce (node {} @ depth {})",
            model.focus_node().number(),
            model.focus_node().depth(),
        );

        let mut added_constraint = false;

        let model = RefCell::new(&mut model);
        for cycle in self.cycles(|var| model.borrow().current_val(var)) {
            if cycle.len() != 1 && cycle.len() < self.tour_len {
                // Found a subtour; add a constraint to eliminate it.
                // dbg!(cycle.len());
                let nodes = &self.nodes;
                let constraint = cons()
                    .expr(cycle.iter().flat_map(|&i| {
                        cycle.iter().filter_map(move |&j| (i != j).then_some((&nodes[i].out_arcs[j], 1.0)))
                    }))
                    .le((cycle.len() - 1) as f64);
                model.borrow_mut().add(constraint);
                // model.borrow_mut().add_cons_local(&constraint);
                added_constraint = true;
            }
        }

        if added_constraint { ConshdlrResult::ConsAdded } else { ConshdlrResult::Feasible }
    }
}

/// A constraint handler that enforces the TSP subtour elimination constraint for cycles of length 3.
/// Allows self-loops (arc from a node to itself).
pub struct SubtourElimination3 {
    nodes: Rc<[Node]>,
}

impl SubtourElimination3 {
    pub fn new(nodes: Rc<[Node]>, tour_len: usize) -> Self {
        assert!(tour_len > 3);
        Self { nodes }
    }

    fn bad_triples(&self, mut get_value: impl FnMut(&Variable) -> f64) -> impl Iterator<Item = [usize; 3]> {
        let n = self.nodes.len();
        let all_triples = (0..n).flat_map(|i| (0..i).flat_map(move |j| (0..j).map(move |k| [i, j, k])));
        all_triples.filter(move |&[i, j, k]| {
            get_value(&self.nodes[i].out_arcs[j])
                + get_value(&self.nodes[j].out_arcs[i])
                + get_value(&self.nodes[i].out_arcs[k])
                + get_value(&self.nodes[k].out_arcs[i])
                + get_value(&self.nodes[j].out_arcs[k])
                + get_value(&self.nodes[k].out_arcs[j])
                > 2.0 + EPS
        })
    }
}

impl Conshdlr for SubtourElimination3 {
    fn check(&mut self, _model: Model<Solving>, _conshdlr: SCIPConshdlr, solution: &Solution) -> bool {
        // println!("SubtourElimination3::check");

        self.bad_triples(|var| solution.val(var)).next().is_none()
    }

    fn enforce(&mut self, mut model: Model<Solving>, _conshdlr: SCIPConshdlr) -> ConshdlrResult {
        // println!(
        //     "SubtourElimination3::enforce (node {} @ depth {})",
        //     model.focus_node().number(),
        //     model.focus_node().depth(),
        // );

        let mut added_constraint = false;

        let model = RefCell::new(&mut model);
        for [i, j, k] in self.bad_triples(|var| model.borrow().current_val(var)) {
            // Found a bad triple; add a constraint to eliminate it.
            println!(
                "    Bad triple: sum_arcs({i}, {j}, {k}) = {}",
                model.borrow().current_val(&self.nodes[i].out_arcs[j])
                    + model.borrow().current_val(&self.nodes[j].out_arcs[i])
                    + model.borrow().current_val(&self.nodes[i].out_arcs[k])
                    + model.borrow().current_val(&self.nodes[k].out_arcs[i])
                    + model.borrow().current_val(&self.nodes[j].out_arcs[k])
                    + model.borrow().current_val(&self.nodes[k].out_arcs[j])
            );

            let constraint = cons()
                .coef(&self.nodes[i].out_arcs[j], 1.0)
                .coef(&self.nodes[j].out_arcs[i], 1.0)
                .coef(&self.nodes[i].out_arcs[k], 1.0)
                .coef(&self.nodes[k].out_arcs[i], 1.0)
                .coef(&self.nodes[j].out_arcs[k], 1.0)
                .coef(&self.nodes[k].out_arcs[j], 1.0)
                .le(2.0);
            model.borrow_mut().add(constraint);
            // model.borrow_mut().add_cons_local(&constraint);
            added_constraint = true;
        }

        if added_constraint { ConshdlrResult::ConsAdded } else { ConshdlrResult::Feasible }
    }
}
