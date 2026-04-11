use std::{collections::HashMap, rc::Rc};

use ordered_float::NotNan;
use russcip::{ModelStageProblemOrSolving, SolError, Variable, prelude::*};

use crate::{
    optimize::{self, MetaRoutine, ProblemInfo},
    optimize_complete::{D1_COST_MULTIPLIER, Node, sort_pair},
};

// A primal heuristic that uses hill climbing to find good solutions very quickly.
pub struct HillClimbHeuristic {
    problem_info: ProblemInfo,
    nodes: Rc<[Node]>,
    pair_vars: HashMap<(usize, usize), Variable>,
    intermission_middle_distance: Variable,
}

impl HillClimbHeuristic {
    pub fn new(
        problem_info: ProblemInfo,
        nodes: Rc<[Node]>,
        pair_vars: HashMap<(usize, usize), Variable>,
        intermission_middle_distance: Variable,
    ) -> Self {
        Self { problem_info, nodes, pair_vars, intermission_middle_distance }
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

            // DEBUG: make a simple, random feasible solution.
            let end_index = self.problem_info.meta_routines().len();

            // let routines = self.problem_info.routines();
            // let mut best_order = Vec::new();
            // best_order.push((routines.len() - 1, None)); // intermission
            // for i in 0..(self.problem_info.num_slots() - 1) {
            //     best_order.push((i, None));
            // }
            // dbg!(self.problem_info.num_slots());
            // dbg!(routines.len());
            // for i in (self.problem_info.num_slots() - 1)..(routines.len() - 1) {
            //     assert!(routines[i].is_doubleable());
            //     best_order.iter_mut().find(|(j, k)| routines[*j].is_doubleable() && k.is_none()).unwrap().1 = Some(i);
            // }
            // let best_order: Vec<usize> = best_order
            //     .into_iter()
            //     .map(|(i, j)| {
            //         let mr = if let Some(j) = j { MetaRoutine::Double(i, j) } else { MetaRoutine::Single(i) };
            //         self.problem_info.meta_routines().iter().position(|&x| x == mr).unwrap()
            //     })
            //     .collect();
            #[rustfmt::skip]
            // let best_order = [0, 32, 19, 10, 15, 1, 31, 8, 24, 7, 17, 30, 12, 20, 29, 13, 23, 22, 9, 26, 11, 28, 3, 4, 6, 5, 2, 34, 21, 16, 18, 14];
            // let best_order = [0, 32, 13, 19, 30, 14, 16, 18, 21, 27, 2, 12, 22, 29, 1, 23, 8, 25, 17, 20, 28, 11, 7, 15, 6, 5, 3, 4, 9, 24, 43, 10];
            // let best_order = [2, 12, 22, 20, 17, 25, 31, 14, 26, 30, 9, 21, 4, 3, 5, 6, 45, 23, 1, 8, 28, 11, 7, 15, 10, 24, 18, 16, 13, 19, 32, 0];
               let best_order = [0, 30, 32, 1, 29, 27, 7, 10, 15, 24, 9, 4, 21, 2, 3, 6, 5, 23, 8, 11, 25, 43, 13, 22, 18, 14, 16, 19, 28, 17, 20, 12];
            // #[rustfmt::skip]
            //    let best_order = [0, 15, 6, 9, 14, 11, 13, 12, 1, 5, 4, 2, 3, 8, 10, 7];

            assert_eq!(best_order.len(), self.problem_info.num_slots());

            solution.set_val(&self.nodes[end_index].out_arcs[best_order[0]], 1.0);
            let mut is_before_intermission = true;
            for pos in 0..(best_order.len() - 1) {
                let i = best_order[pos];
                let j = best_order[pos + 1];
                solution.set_val(&self.nodes[i].out_arcs[j], 1.0);

                if i == self.problem_info.intermission_index() {
                    is_before_intermission = false;

                    // Set the intermission middle distance variable.
                    let n = best_order.len();
                    let dist = usize::max(((n - 1) / 2).saturating_sub(pos), pos.saturating_sub(n / 2));
                    solution.set_val(&self.intermission_middle_distance, dist as f64);
                }
                solution.set_val(&self.nodes[i].is_before_intermission, if is_before_intermission { 1.0 } else { 0.0 });

                if let Some(&k) = best_order.get(pos + 2)
                    && j != self.problem_info.intermission_index()
                {
                    // Set any distance-2 pair variables.
                    for ir in self.problem_info.meta_routines()[i].routine_indices() {
                        for kr in self.problem_info.meta_routines()[k].routine_indices() {
                            if let Some(pair_var) = self.pair_vars.get(&sort_pair(ir, kr)) {
                                solution.set_val(pair_var, 1.0);
                            }
                        }
                    }
                }
            }
            solution.set_val(&self.nodes[best_order[best_order.len() - 1]].out_arcs[end_index], 1.0);

            // Set self-loop arcs to 1 for unvisited nodes.
            for i in 0..self.problem_info.meta_routines().len() {
                if !best_order.contains(&i) {
                    solution.set_val(&self.nodes[i].out_arcs[i], 1.0);
                }
            }

            // debug_assert_eq!(solution.obj_val(), best_heur_obj_val as f64);
            match model.add_sol(solution) {
                Ok(()) => HeurResult::FoundSol,
                Err(SolError::Infeasible) => panic!("solution from HillClimbHeuristic was rejected as infeasible"),
                // Err(SolError::Infeasible) => HeurResult::NoSolFound,
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
