use std::{collections::HashMap, rc::Rc};

use ordered_float::NotNan;
use russcip::{ModelStageProblemOrSolving, SolError, Variable, prelude::*};

use crate::{
    optimize::{self, ProblemInfo},
    optimize_complete::{D1_COST_MULTIPLIER, Node, sort_pair},
};

// A primal heuristic that uses hill climbing to find good solutions very quickly.
pub struct HillClimbHeuristic {
    problem_info: ProblemInfo,
    nodes: Rc<[Node]>,
    pair_vars: HashMap<(usize, usize), Variable>,
}

impl HillClimbHeuristic {
    pub fn new(problem_info: ProblemInfo, nodes: Rc<[Node]>, pair_vars: HashMap<(usize, usize), Variable>) -> Self {
        Self { problem_info, nodes, pair_vars }
    }

    pub fn try_find_solution<S: ModelStageProblemOrSolving>(
        &self,
        model: &Model<S>,
        best_obj_val: Option<f64>,
        num_samples: usize,
        mut sample_seed_order: impl FnMut(&mut [usize]),
    ) -> HeurResult {
        // println!("HillClimbHeuristic::try_find_solution");

        let n = self.problem_info.routines().len();
        let end_index = n;

        let mut best_order = (0..n).collect::<Vec<_>>();
        let mut best_heur_obj_val = usize::MAX;
        let mut tmp_order = best_order.clone();
        for _ in 0..num_samples {
            sample_seed_order(&mut tmp_order);

            let (d1_cost, d2_cost, _) = optimize::hill_climb_order(&self.problem_info, &mut tmp_order);
            let obj_val = (d1_cost * D1_COST_MULTIPLIER) + d2_cost;

            if obj_val < best_heur_obj_val {
                best_heur_obj_val = obj_val;
                best_order.clone_from(&tmp_order);
                // println!("New best_heur_obj_val: {best_heur_obj_val:?}");
            }
        }

        let best_obj_val = best_obj_val.unwrap_or(f64::INFINITY);
        if (best_heur_obj_val as f64) < best_obj_val {
            println!("HillClimbHeuristic found solution with obj value: {best_heur_obj_val}");

            let solution = model.create_orig_sol();

            solution.set_val(&self.nodes[end_index].out_arcs[best_order[0]], 1.0);
            for pos in 0..(best_order.len() - 1) {
                let i = best_order[pos];
                let j = best_order[pos + 1];
                solution.set_val(&self.nodes[i].out_arcs[j], 1.0);
                if let Some(&k) = best_order.get(pos + 2)
                    && let Some(pair_var) = self.pair_vars.get(&sort_pair(i, k))
                    && j != self.problem_info.intermission_index()
                {
                    solution.set_val(pair_var, 1.0);
                }
            }
            solution.set_val(&self.nodes[best_order[best_order.len() - 1]].out_arcs[end_index], 1.0);

            debug_assert_eq!(solution.obj_val(), best_heur_obj_val as f64);
            match model.add_sol(solution) {
                Ok(()) => HeurResult::FoundSol,
                Err(SolError::Infeasible) => HeurResult::NoSolFound,
            }
        } else {
            HeurResult::NoSolFound
        }
    }
}

impl Heuristic for HillClimbHeuristic {
    fn execute(&mut self, model: Model<Solving>, _timing: HeurTiming, node_inf: bool) -> HeurResult {
        // println!(
        //     "HillClimbHeuristic::execute (node {} @ depth {})",
        //     model.focus_node().number(),
        //     model.focus_node().depth(),
        // );

        // Skip if the node is infeasible.
        if node_inf {
            return HeurResult::DidNotRun;
        }

        // // Debug output of the current fractional solution:
        // let mut graph = DiGraph::new();
        // let graph_nodes = (0..self.nodes.len()).map(|i| graph.add_node(i)).collect::<Vec<_>>();
        // for (i, node) in self.nodes.iter().enumerate() {
        //     for (j, arc_var) in node.out_arcs.iter().enumerate() {
        //         let value = model.current_val(arc_var);
        //         if value > 1e-10 {
        //             graph.add_edge(graph_nodes[i], graph_nodes[j], format!("{:.2}", value));
        //         }
        //     }
        // }
        // std::fs::write("debug.dot", format!("{:?}", Dot::with_config(&graph, &[]))).unwrap();
        // if rand::rng().random_bool(0.5) {
        //     std::process::exit(0);
        // }

        let best_obj_val = model.best_sol().map(|sol| sol.obj_val());
        self.try_find_solution(&model, best_obj_val, 1, |order| {
            // Generate an order by repeated greedy selection of the next max-value arc in the fractional solution.
            let mut visited = vec![false; self.nodes.len()];
            let end_index = self.problem_info.routines().len();
            let mut node = end_index;
            for order_item in order.iter_mut() {
                visited[node] = true;
                let candidates = self.nodes[node].out_arcs.iter().enumerate().filter(|&(i, _)| !visited[i]);
                let max = candidates.max_by_key(|&(_, arc_var)| {
                    NotNan::new(model.current_val(arc_var)).expect("variable values should never be NaN")
                });
                node = max.unwrap().0;
                assert_ne!(node, end_index);
                *order_item = node;
            }
        })
    }
}
