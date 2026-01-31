use std::{cell::RefCell, rc::Rc};

use russcip::{Solution, Variable, prelude::*};

use crate::optimize_complete::{Node, successor};

/// A constraint handler that enforces the TSP subtour elimination constraint.
pub struct SubtourElimination {
    nodes: Rc<[Node]>,
}

impl SubtourElimination {
    pub fn new(nodes: Rc<[Node]>) -> Self {
        Self { nodes }
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

        let n = self.nodes.len();
        self.cycles(|var| solution.val(var)).all(|cycle| cycle.len() == n)
    }

    fn enforce(&mut self, mut model: Model<Solving>, _conshdlr: SCIPConshdlr) -> ConshdlrResult {
        println!(
            "SubtourElimination::enforce (node {} @ depth {})",
            model.focus_node().number(),
            model.focus_node().depth(),
        );

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
