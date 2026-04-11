#![allow(unused)] // TODO: Fix this file or delete it.

use std::{cell::RefCell, collections::HashMap, iter, rc::Rc};

use russcip::{Solution, Variable, prelude::*};

use crate::{
    optimize::ProblemInfo,
    optimize_complete::{Node, sort_pair, successor},
};

/// A constraint handler that enforces costs on distance-2 path nodes.
/// NOTE: This is currently broken; adding variables to the model during enforcement is problematic.
/// TODO: Add all the variables upfront (done), and then lazily add their associated constraints to make the
///       initial LP smaller.
pub struct Distance2Costs {
    problem_info: ProblemInfo,
    nodes: Rc<[Node]>,
    pair_vars: HashMap<(usize, usize), Variable>,
}

impl Distance2Costs {
    pub fn new(problem_info: ProblemInfo, nodes: Rc<[Node]>) -> Self {
        Self { problem_info, nodes, pair_vars: HashMap::new() }
    }

    fn get_pair_var(&self, i1: usize, i3: usize) -> Option<&Variable> {
        self.pair_vars.get(&sort_pair(i1, i3))
    }
}

/// Returns an iterator over all triples of routine nodes (i1, i2, i3) in the solution path defined by `get_value`,
/// such that i1 and i3 have at least one dancer in common.
fn iter_triples<'a>(
    nodes: &'a [Node],
    problem_info: &'a ProblemInfo,
    mut get_value: impl FnMut(&Variable) -> f64 + 'a,
) -> impl Iterator<Item = (usize, usize, usize)> + 'a {
    let n = nodes.len();
    let end_index = n - 1;

    let mut i2 = successor(nodes, end_index, &mut get_value).unwrap();
    let mut i3 = successor(nodes, i2, &mut get_value).unwrap();
    iter::from_fn(move || {
        loop {
            let i1;
            (i1, i2, i3) = (i2, i3, successor(nodes, i3, &mut get_value).unwrap());
            if i3 == end_index {
                return None;
            }

            // Enforce distance-2 cost constraint for subpath i1 -> i2 -> i3.
            if problem_info.intersection_count(i1, i3) > 0 {
                return Some((i1, i2, i3));
            }
        }
    })
}

impl Conshdlr for Distance2Costs {
    fn check(&mut self, _model: Model<Solving>, _conshdlr: SCIPConshdlr, solution: &Solution) -> bool {
        iter_triples(&self.nodes, &self.problem_info, |var| solution.val(var)).all(|(i1, _, i3)| {
            // println!(
            //     "    >>> ({i1}, _, {i3}): {:?}",
            //     self.get_pair_var(i1, i3).is_some_and(|var| solution.val(var) == 1.0)
            // );
            self.get_pair_var(i1, i3).is_some_and(|var| solution.val(var) == 1.0)
        })
    }

    fn enforce(&mut self, mut model: Model<Solving>, _conshdlr: SCIPConshdlr) -> ConshdlrResult {
        // println!("Distance2Costs::enforce (node {})", model.focus_node().number());

        let mut added_constraint = false;

        let model = RefCell::new(&mut model);
        for (i1, i2, i3) in iter_triples(&self.nodes, &self.problem_info, |var| model.borrow().current_val(var)) {
            // Enforce distance-2 cost constraint for subpath i1 -> i2 -> i3.
            let key = sort_pair(i1, i3);
            let pair_var = self.pair_vars.entry(key).or_insert_with(|| {
                model.borrow_mut().add(
                    var()
                        .name(&format!("d2_pair_{key:?}"))
                        .bin()
                        .obj(self.problem_info.intersection_count(i1, i3) as f64),
                )
            });

            model.borrow_mut().add(
                cons()
                    .coef(&self.nodes[i1].out_arcs[i2], 1.0)
                    .coef(&self.nodes[i2].out_arcs[i3], 1.0)
                    .coef(pair_var, -1.0)
                    .le(1.0),
            );
            added_constraint = true;
        }

        if added_constraint { ConshdlrResult::ConsAdded } else { ConshdlrResult::Feasible }
    }
}
